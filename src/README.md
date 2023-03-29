## LR-1BS ROS Drive and LaserScan  ##

#### build ####
1. make catkin workspace at your ros machine

    > mkdir -p catkin_ws/src

2. get olei-lidar.tgz from vendor tech support

3. unzip  zipfile at catkin_ws/src

    >tar zxvf olei-lidar.tgz

4. install depend

    >rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

5. build

    >catkin_make

#### run ####
1. source to path

    >source devel/setup.bash

2. new terminal,then run roscore

    >roscore

3. connect lidar and host,lidar ip default 192.168.1.100,port default 2368

    host ip default 192.168.1.10

    these ip and port can be modified by vendor config software

4. run lidar drive at first terminal

	roslaunch olelidar scan.launch

#### rviz view ####
1. new terminal,then run rviz
    
    >rosrun rviz rviz -f olelidar
    
2. at rviz,add topic olelidar/scan/LaserScan
























