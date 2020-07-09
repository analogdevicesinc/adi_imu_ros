# ROS node for ADI IMU

A C++ ROS node that read sensor data from ADI IMU and publishes message to `/adi_imu/data_raw` topic.
The node can publish the data as a standard `sensor_msgs/Imu` message, or a custom `adi_imu_ros/AdiImu` message.
It can alternatively save the data to a csv file, bypassing ROS messaging altogether.


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

3. Test

* In one terminal, run roscore
```bash
$ roscore
```
* In new terminal, wihtin the catkin_ws folder you created, run
```bash
$ source devel/setup.bash
$ rosrun adi_imu_ros adi_imu_ros_node
```
Alternatively, you can launch the node with the provided launch file.
The most common parameters you might want to change are listed in the launch file.
You can add more as needed. Look for the parameters list in the c++ file `adi_imu_ros.cpp`.
```bash
$ source devel/setup.bash
$ roslaunch adi_imu_ros adis16545.launch
```
* In new terminal, wihtin the catkin_ws folder you created, run
```bash
$ rostopic echo /adi_imu/data_raw
```
You will need to source the devel folder in this terminal prior to running rostopic, if you choose 
to publish the measurements using our custom message type `adi_imu_ros/AdiImu`. 
