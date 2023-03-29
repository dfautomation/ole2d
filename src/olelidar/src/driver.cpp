#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>

#include "constants.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <olei_msgs/OleiPacket.h>
#include <olei_msgs/OleiScan.h>

// here maskoff waring of macro 'ROS_LOG'
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"

namespace olelidar {

using namespace olei_msgs;
using namespace diagnostic_updater;

/// Constants
//static constexpr uint16_t kUdpPort = 2368;
static constexpr size_t kPacketSize = sizeof(OleiPacket().data);
static constexpr int kError = -1;

// p49 8.2.1
// static constexpr double kDelayPerPacketNs = kSequencesPerPacket * kFiringCycleNs;
// static constexpr double kPacketsPerSecond = 1e9 / kDelayPerPacketNs;

class Driver {
 public:
  explicit Driver(const ros::NodeHandle &pnh);
  ~Driver();

  bool Poll();

 private:
  bool OpenUdpPort();
  int ReadPacket(OleiPacket &packet) const;

  // Ethernet relate variables
  std::string device_ip_str_;
  int device_port_;
  in_addr device_ip_;
  int socket_id_{-1};

  // ROS related variables
  ros::NodeHandle pnh_;
  ros::Publisher pub_packet_;

  // Diagnostics updater
  diagnostic_updater::Updater updater_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> topic_diag_;
  std::vector<OleiPacket> buffer_;
  // double freq_;
};

Driver::Driver(const ros::NodeHandle &pnh) : pnh_(pnh) {
  ROS_INFO("packet size: %zu", kPacketSize);
  pnh_.param("device_ip", device_ip_str_, std::string("192.168.1.100"));
  pnh_.param("device_port", device_port_, 2368);
  ROS_INFO("device_ip: %s:%d",device_ip_str_.c_str(), device_port_);

  if (inet_aton(device_ip_str_.c_str(), &device_ip_) == 0) {
    // inet_aton() returns nonzero if the address is valid, zero if not.
    ROS_FATAL("Invalid device ip: %s:%d",device_ip_str_.c_str(), device_port_);
    ros::shutdown();
  }

  // ROS diagnostics
  // mask
  // updater_.setHardwareID("VLP16");

  // 8.2.1 Data Packet Rate
  // There are 24 firing cycles in a data packet.
  // 24 x 55.296 μs = 1.327 ms is the accumulation delay per packet.
  // 1 packet/1.327 ms = 753.5 packets/second
  //// freq_ = kPacketsPerSecond;
  //// ROS_INFO("expected frequency: %.3f (Hz)", freq_);

  ////topic_diag_.reset(new TopicDiagnostic(
  ////    "packet", updater_, FrequencyStatusParam(&freq_, &freq_, 0.1, 100),
  ////    TimeStampStatusParam(-0.1, 0.1)));

  // Output
  pub_packet_ = pnh_.advertise<OleiPacket>("packet", 10);

  if (!OpenUdpPort()) {
    ROS_ERROR("Failed to open UDP Port");
  }

  ROS_INFO("Successfully opened UDP");
}

Driver::~Driver() {
  if (close(socket_id_)) {
    ROS_INFO("Close socket %d at %s", socket_id_, device_ip_str_.c_str());
  } else {
    ROS_ERROR("Failed to close socket %d at %s", socket_id_,
              device_ip_str_.c_str());
  }
}

bool Driver::OpenUdpPort() {
  socket_id_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (socket_id_ == -1) {
    perror("socket");
    ROS_ERROR("Failed to create socket");
    return false;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(uint16_t(device_port_));    // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(socket_id_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    perror("bind");  // TODO: ROS_ERROR errno
    ROS_ERROR("Failed to bind to socket %d", socket_id_);
    return false;
  }

  if (fcntl(socket_id_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    ROS_ERROR("Failed to set socket to non-blocking");
    return false;
  }

  return true;
}

int Driver::ReadPacket(OleiPacket &packet) const {
  const auto time_before = ros::Time::now();

  struct pollfd fds[1];
  fds[0].fd = socket_id_;
  fds[0].events = POLLIN;
  const int timeout_ms = 1000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true) {
    // Unfortunately, the Linux kernel recvfrom() implementation
    // uses a non-interruptible sleep() when waiting for data,
    // which would cause this method to hang if the device is not
    // providing data.  We poll() the device first to make sure
    // the recvfrom() will not block.
    //
    // Note, however, that there is a known Linux kernel bug:
    //
    //   Under Linux, select() may report a socket file descriptor
    //   as "ready for reading", while nevertheless a subsequent
    //   read blocks.  This could for example happen when data has
    //   arrived but upon examination has wrong checksum and is
    //   discarded.  There may be other circumstances in which a
    //   file descriptor is spuriously reported as ready.  Thus it
    //   may be safer to use O_NONBLOCK on sockets that should not
    //   block.

    // poll() until input available
    do {
      const int retval = poll(fds, 1, timeout_ms);

      if (retval < 0) {
        // poll() error?
        if (errno != EINTR) ROS_ERROR("poll() error: %s", strerror(errno));
        return kError;
      } else if (retval == 0) {
        // poll() timeout?
        ROS_WARN("olamlidar poll() timeout");
        return kError;
      }

      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
          (fds[0].revents & POLLNVAL)) {
        // device error?
        ROS_ERROR("poll() reports olamlidar error");
        return kError;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    // Receive packets that should now be available from the
    // socket using a blocking read.
    const ssize_t nbytes =
        recvfrom(socket_id_, &packet.data[0], kPacketSize, 0,
                 (sockaddr *)&sender_address, &sender_address_len);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        perror("recvfail");
        ROS_ERROR("Failed to read from socket");
        return kError;
      }
    } else if (static_cast<size_t>(nbytes) == kPacketSize) {
      //else if ((size_t)nbytes == kPacketSize) {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (device_ip_str_ != "" &&
          sender_address.sin_addr.s_addr != device_ip_.s_addr)
        continue;
      else
        break;  // done
    }

    ROS_DEBUG_STREAM("incomplete Olei packet read: " << nbytes << " bytes");
  }

  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred.
  //  const auto time_after = ros::Time::now();
  // Usually take around 0.0012s on my pc
  //  ROS_INFO("time: %f", time_after - time_before);
  //  packet.stamp = ros::Time((time_after + time_before) / 2.0);
  // The ros velodyne driver uses average time to as the time of the packet.
  // We just use the starting time, and the actual time is this time + some time
  // delay of communication
  packet.stamp = time_before;

  return 0;
}

bool Driver::Poll() {
  OleiPacket::Ptr packet(new OleiPacket);

  while (true) {
    // keep reading until full packet received
    const int rc = ReadPacket(*packet);
    if (rc == 0) break;        // got a full packet?
    if (rc < 0) return false;  // end of file reached?
  }

  // publish message using time of last packet read
  pub_packet_.publish(packet);

  // notify diagnostics that a message has been published, updating
  // its status
  // remark below for driver debug,here error
  // topic_diag_->tick(packet->stamp);
  // updater_.update();

  return true;
}

}  // namespace olei_puck

int main(int argc, char **argv) {
  ros::init(argc, argv, "olelidar_driver");
  ros::NodeHandle pnh("~");

  olelidar::Driver node(pnh);

  while (ros::ok()) {
    // poll device until end of file
    if (!node.Poll()) {
      ROS_WARN("Failed to poll device");
      //break;
    }
    ros::spinOnce();
  }
  return 0;
}
