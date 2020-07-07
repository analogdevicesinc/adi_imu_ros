# ROS node for ADI IMU

A C++ ROS node that read sensor data from ADI IMU and publishes message `sensor_msgs/Imu` to `/adi_imu/data_raw` topic.


## Setup instructions

1. Git clone to your catkin workspace (inside `catkin_ws/src/`)
```bash
$ git clone --recursive -b master git@gitlab.analog.com:spalani1/adi_imu_ros.git
```

2. Build
```bash
$ cd catkin_ws
$ catkin_make
```

3. Run
```bash
$ ./devel/lib/adi_imu_ros/adi_imu_ros_node
```

4. Test
* In new terminal, run roscore
```bash
$ roscore
```
* In new terminal, run
```bash
$ rostopic echo /adi_imu/data_raw
```
