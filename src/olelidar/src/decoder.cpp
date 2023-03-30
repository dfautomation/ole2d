#include "constants.h"

#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/LaserScan.h>

#include <olei_msgs/OleiPacket.h>
#include <olei_msgs/OleiScan.h>
#include <olelidar/OleiPuckConfig.h>

// here maskoff waring of macro 'ROS_LOG'
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"

namespace olelidar
{

  using namespace sensor_msgs;
  using namespace olei_msgs;

  class Decoder
  {
  public:
    explicit Decoder(const ros::NodeHandle &pnh);

    Decoder(const Decoder &) = delete;
    Decoder operator=(const Decoder &) = delete;

    void PacketNormalize(std::vector<uint16_t> &packet_in, std::vector<uint16_t> &packet_out);
    void PacketCb(const OleiPacketConstPtr &packet_msg);
    void ConfigCb(OleiPuckConfig &config, int level);
    void PublishMsg(ros::Publisher *pub, std::vector<uint16_t> &packet_r, std::vector<uint16_t> &packet_i);

  private:
    /// 9.3.1.3 Data Point
    /// A data point is a measurement by one laser channel of a relection of a
    /// laser pulse
    struct DataPoint
    {
      uint16_t azimuth;      //unit: 0.01 degree
      uint16_t distance;     //unit: 1mm
      uint16_t reflectivity; //unit: 1 percent,100 is the reflet of white target
      uint16_t distance2;    //rsv for dual return
    } __attribute__((packed));
    static_assert(sizeof(DataPoint) == 8, "sizeof(DataPoint) != 8");

    struct FiringSequence
    {
      DataPoint points[kFiringsPerSequence]; // 8
    } __attribute__((packed));
    static_assert(sizeof(FiringSequence) == 8, "sizeof(FiringSequence) != 8");

    struct DataHead
    {
      uint8_t magic[4];
      uint8_t version[2];
      uint8_t scale;
      uint8_t oem[3];
      uint8_t model[12];
      uint8_t code[2];
      uint8_t hw[2];
      uint8_t sw[2];
      uint32_t timestamp;
      uint16_t rpm;
      uint8_t flag[2];
      uint8_t rsv[4];
    } __attribute__((packed));
    static_assert(sizeof(DataHead) == 40, "sizeof(DataBlock) != 40");

    struct DataBlock
    {
      FiringSequence sequences[kSequencesPerBlock]; // 8 * 1
    } __attribute__((packed));
    static_assert(sizeof(DataBlock) == 8, "sizeof(DataBlock) != 8");

    /// 9.3.1.5 Packet format for 2d
    struct Packet
    {
      DataHead head;
      DataBlock blocks[kBlocksPerPacket]; // 8 * 150

    } __attribute__((packed));
    static_assert(sizeof(Packet) == 1240, "sizeof(DataBlock) != 1240");
    static_assert(sizeof(Packet) == sizeof(olei_msgs::OleiPacket().data), "sizeof(Packet) != 1240");

    void DecodeAndFill(const Packet *const packet_buf, uint64_t);

  private:
    bool CheckFactoryBytes(const Packet *const packet);
    void Reset();

    // ROS related parameters
    ros::Time start;
    std::string frame_id_;
    int range_max_;
    int ange_start_;
    int ange_end_;
    bool inverted_;
    int poly_;
    ros::NodeHandle pnh_;
    // sub driver topic(msg)
    ros::Subscriber packet_sub_;
    // pub laserscan message
    ros::Publisher scan_pub_;

    // dynamic param server
    dynamic_reconfigure::Server<OleiPuckConfig> cfg_server_;
    OleiPuckConfig config_;

    // add vector for laserscan
    std::vector<uint16_t> scanAngleVec_;
    std::vector<uint16_t> scanRangeVec_;
    std::vector<uint16_t> scanIntensityVec_;

    std::vector<uint16_t> scanAngleInVec_;
    std::vector<uint16_t> scanAngleOutVec_;

    std::vector<uint16_t> scanRangeInVec_;
    std::vector<uint16_t> scanRangeOutVec_;

    std::vector<uint16_t> scanIntensityInVec_;
    std::vector<uint16_t> scanIntensityOutVec_;

    std::vector<float> scanRangeBuffer;
    std::vector<float> scanintensitiesBuffer;
    uint16_t azimuthLast_;
    uint16_t azimuthNow_;
    uint16_t azimuthFirst_;

    // laserscan msg
    uint32_t scanMsgSeq_;
    //电机频率
    float frequency;
    //雷达型号类型
    unsigned char lidarType=0x01; 
    //电机方向定义
    int direction;
  };

  Decoder::Decoder(const ros::NodeHandle &pnh)
      : pnh_(pnh), cfg_server_(pnh)
  {
    // get param from cfg file at node start
    pnh_.param<std::string>("frame_id", frame_id_, "olelidar");
    pnh_.param<int>("r_max", range_max_, 10000);
    ROS_INFO("===========================");
    ROS_INFO("Ole Frame_id: %s", frame_id_.c_str());
    ROS_INFO("Ole Topic: %s/scan", frame_id_.c_str());
    ROS_INFO("Ole RangeMax: %d mm", range_max_);
    ROS_INFO("===========================");
    start = ros::Time::now();
    // dynamic callback
    cfg_server_.setCallback(boost::bind(&Decoder::ConfigCb, this, _1, _2));
    // packet receive
    azimuthLast_ = 0;
    azimuthNow_ = 0;
    azimuthFirst_ = 0xFFFF;
    // laserscan msg init
    scanMsgSeq_ = 0;
    direction = 0;
  }

  void Decoder::PublishMsg(ros::Publisher *pub, std::vector<uint16_t> &packet_r, std::vector<uint16_t> &packet_i)
  {
    sensor_msgs::LaserScan scanMsg;
    uint16_t route = uint16_t(config_.route);

    /*
  std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
  float32 angle_min
  float32 angle_max
  float32 angle_increment
  float32 time_increment
  float32 scan_time
  float32 range_min
  float32 range_max
  float32[] ranges
  float32[] intensities
  */

    int min = config_.angle_min * 100 + 18000;
    int max = config_.angle_max * 100 + 18000;

    scanRangeBuffer.clear();
    scanintensitiesBuffer.clear();
    //ROS_INFO("start to build ranges and intensities");
    for (uint16_t i = 0; i < route; i++)
    {
      // 过滤出指定角度范围内点云
      int angle = scanAngleOutVec_[i];
      if (angle >= min && angle <= max)
      {
        float range = scanRangeOutVec_[i] * 0.001f;
        float intensities = scanIntensityOutVec_[i] * 1.0f;

        scanRangeBuffer.push_back(range);
        scanintensitiesBuffer.push_back(intensities);
      }
    }

    float bufferlen = scanRangeBuffer.size();
    //ROS_INFO("bufferlen:%f",bufferlen);
    scanMsg.ranges.resize(bufferlen / poly_);
    scanMsg.intensities.resize(bufferlen / poly_);
    int index = 0;

    for (uint16_t i = 0; i < bufferlen; i++)
    {
      float range = 0.0f;
      float intent = 0.0f;

      if (direction == 0)
      {
        range = scanRangeBuffer[bufferlen - i - 1];
        intent = scanintensitiesBuffer[bufferlen - i - 1];
      }
      else
      {
        range = scanRangeBuffer[i];
        intent = scanintensitiesBuffer[i];
      }
      if (i % poly_ == 0)
      {
        scanMsg.ranges[index] = range;
        scanMsg.intensities[index] = intent;
        index++;
      }
    }

    float len = scanMsg.ranges.size();
    //扫描顺序自增ID序列
    scanMsg.header.seq = scanMsgSeq_;
    scanMsgSeq_++;
    //激光数据时间戳
    scanMsg.header.stamp = start;
    //FrameId
    scanMsg.header.frame_id = frame_id_.c_str();
    //定义开始角度和结束角度
    scanMsg.angle_min = deg2rad(config_.angle_min);
    scanMsg.angle_max = deg2rad(config_.angle_max);
    //定义测距的最小值和最大值单位:m
    scanMsg.range_min = config_.range_min;
    scanMsg.range_max = config_.range_max;
    //角度分辨率
    scanMsg.angle_increment = deg2rad(config_.step);
    //时间分辨率（相邻两个角度之间耗费时间）
    scanMsg.time_increment = 1/frequency/float(route)*1e-3;
    //扫描的时间间隔
    scanMsg.scan_time = 1/frequency*1e-3; 

    uint16_t size=scanAngleOutVec_.size();
    uint16_t fb=360/(config_.step);
    if (fb==size) 
    {
      pub->publish(scanMsg); //校验当符合点数完整的一帧数据才向外发布话题
    }
    else
    {
      if(scanMsgSeq_==1) return;
      ROS_INFO("pointCloud frame:%d  size:%d  scanMsgSeq_:%d",fb,size,scanMsgSeq_);
    }
    start = ros::Time::now();
  }

  void Decoder::DecodeAndFill(const Packet *const packet_buf, uint64_t time)
  {

    // For each data block, 150 total
    uint16_t azimuth;
    uint16_t range;
    uint16_t intensity;
    for (int iblk = 0; iblk < kBlocksPerPacket; ++iblk)
    {
      const auto &block = packet_buf->blocks[iblk];

      // simple loop
      azimuth = block.sequences[0].points[0].azimuth;
      range = block.sequences[0].points[0].distance;
      intensity = block.sequences[0].points[0].reflectivity;
      //排除异常点云数据
      if (range > range_max_ || range < 10)
      {
        range = 0;
        intensity = 0;
      }
      // azimuth ge 36000 is not valid
      if (azimuth < 0xFF00)
      {
        scanAngleVec_.push_back(azimuth);
        scanRangeVec_.push_back(range);
        scanIntensityVec_.push_back(intensity);
      }
    }
  }

  void Decoder::PacketNormalize(std::vector<uint16_t> &packet_in, std::vector<uint16_t> &packet_out)
  {

    uint16_t size_in = uint16_t(packet_in.size());
    uint16_t size_std = uint16_t(config_.route);
    float div = (size_in - 1) / (size_std - 1);
    uint16_t size_std_dec = size_std - 1;
    uint16_t size_in_dec = size_in - 1;

    float index, index_frac;
    uint16_t index_int;
    float diff;
    float val;

    packet_out.clear();
    packet_out.push_back(packet_in[0]);

    for (uint16_t i = 1; i < size_std_dec; i++)
    {
      index = div * i;
      index_int = uint16_t(index);
      index_frac = index - index_int;
      diff = packet_in[index_int + 1] - packet_in[index_int];
      val = packet_in[index_int] + diff * index_frac;
      packet_out.push_back(uint16_t(val));
    }

    packet_out.push_back(packet_in[size_in_dec]);
  }

  void Decoder::PacketCb(const OleiPacketConstPtr &packet_msg)
  {
    const auto *packet_buf = reinterpret_cast<const Packet *>(&(packet_msg->data[0]));

    azimuthNow_ = packet_buf->blocks[0].sequences[0].points[0].azimuth;

    //取得第一个数据包
    if (azimuthFirst_ == 0xFFFF)
    {
      //雷达型号类型
      lidarType = packet_buf->head.code[1];
      azimuthFirst_ = azimuthNow_;
      //取得转速和转向
      int rpm = (packet_buf->head.rpm) & 0x7FFF;
      direction = (packet_buf->head.rpm) >> 15;
      ROS_INFO("rpm:%d  direction:%d  lidarType:%d", rpm, direction,lidarType);
      if (inverted_) direction = !direction;
    }

    if (azimuthLast_ < azimuthNow_)
    {
      DecodeAndFill(packet_buf, packet_msg->stamp.toNSec());
      azimuthLast_ = azimuthNow_;
      return;
    }
    else
    {
      azimuthLast_ = azimuthNow_;
    }
    // scan first half route
    if (azimuthFirst_ >= 200)
    {
      azimuthFirst_ = azimuthNow_;
      return;
    }
    //ROS_INFO("==============================");
    
    //雷达型号类型
    lidarType = packet_buf->head.code[1];
    //ROS_INFO("type:%d",lidarType);
    //角度分辨率
    config_.step = (scanAngleVec_[1]-scanAngleVec_[0])/100.0;
    //ROS_INFO("config_.step:%f",config_.step);
    //点云数量
    config_.route = scanRangeVec_.size();
    //雷达工作频率
    if(lidarType==0x01)
    {
      int rpm = (packet_buf->head.rpm) & 0x7FFF;
      frequency = rpm / 60.0;
      config_.step=360/float(config_.route);
    }
    else
    {    
      frequency = config_.step * 10000.0/60.0;
    }


    scanAngleInVec_.clear();
    scanRangeInVec_.clear();
    scanIntensityInVec_.clear();

    scanAngleOutVec_.clear();
    scanRangeOutVec_.clear();
    scanIntensityOutVec_.clear();

    scanAngleInVec_.assign(scanAngleVec_.begin(), scanAngleVec_.end());
    scanRangeInVec_.assign(scanRangeVec_.begin(), scanRangeVec_.end());
    scanIntensityInVec_.assign(scanIntensityVec_.begin(), scanIntensityVec_.end());

    scanAngleVec_.clear();
    scanRangeVec_.clear();
    scanIntensityVec_.clear();

    PacketNormalize(scanAngleInVec_, scanAngleOutVec_);
    PacketNormalize(scanRangeInVec_, scanRangeOutVec_);
    PacketNormalize(scanIntensityInVec_, scanIntensityOutVec_);

    //抛出点云数据
    PublishMsg(&scan_pub_, scanRangeOutVec_, scanIntensityOutVec_);

    //解码原始数据包
    DecodeAndFill(packet_buf, packet_msg->stamp.toNSec());
    // Don't forget to reset
    // Reset();
    //ROS_INFO("Time: %f", (ros::Time::now() - start).toSec());
  }

  void Decoder::ConfigCb(OleiPuckConfig &config, int level)
  {
    // config.min_range = std::min(config.min_range, config.max_range);
    //config.route =4000;
    pnh_.param<int>("ang_start", ange_start_, 0);
    pnh_.param<int>("ang_end", ange_end_, 360);
    pnh_.param<int>("r_max", range_max_, 30);
    pnh_.param<bool>("inverted", inverted_, false);
    pnh_.param<int>("poly", poly_, 1);
    config.angle_min = ange_start_;
    config.angle_max = ange_end_;
    config.range_min = 0.01f;
    config.range_max = range_max_;
    ROS_INFO(
        "angle_min: %.2f, angle_max: %.2f  range_min: %.2f",
        config.angle_min,
        config.angle_max,
        config.range_min);
    config_ = config;
    Reset();

    if (level < 0)
    {
      // topic = sacn,msg = LaserScan,queuesize=10
      scan_pub_ = pnh_.advertise<LaserScan>("scan", 1000);
      packet_sub_ = pnh_.subscribe<OleiPacket>("packet", 256, &Decoder::PacketCb, this);
      ROS_INFO("Drive Ver:2.0.10");
      ROS_INFO("Decoder initialized");
    }
  }

  void Decoder::Reset()
  {
  }

} // namespace olelidar

int main(int argc, char **argv)
{
  ros::init(argc, argv, "olelidar_decoder");
  ros::NodeHandle pnh("~");

  olelidar::Decoder node(pnh);
  ros::spin();
}
