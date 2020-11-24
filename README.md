# ROS node for ADI IMU

A C++ ROS node that read sensor data from ADI IMU and publishes message to `/adi_imu/data_raw` topic.

See below for various message types supported.


## Setup instructions

1. Clone repo to your catkin workspace (inside `catkin_ws/src/`)
```bash
$ git clone --recursive -b master git@gitlab.analog.com:spalani1/adi_imu_ros.git
```

2. Build and install
```bash
$ cd catkin_ws
$ catkin_make install
```

3. Test

* In one terminal, run roscore
```bash
$ roscore
```
* In new terminal, run
```bash
$ source <catkin_ws>/devel/setup.bash
$ rosrun adi_imu_ros adi_imu_ros_node
```
Alternatively, you can launch the node with the provided launch file.
The most common parameters you might want to change are listed in the launch file.
You can add more as needed. Look for the parameters list in the c++ file `adi_imu_ros.cpp`.
```bash
$ source <catkin_ws>/devel/setup.bash
$ roslaunch adi_imu_ros adis16495.launch [args]
```
`args`:  
* `csv_folder` - Location where the csv file will be stored [default: `adi_imu_ros/data`]
* `imu_frame` - Name of the frame associated with the IMU [default: `imu`]
* `spi_dev` - SPI port on the host computer [default: `/dev/spidev0.1`]
* [`message_type`](#message-types-supported) - Message type to be published [default: `adi`]
* `output_rate` - Desired frame rate of the IMU in Hz [default: `2000`]
* `en_isensor_buffer` - Enable when using iSensor SPI buffer between host and IMU [default: `false`]

* In new terminal, wihtin the catkin_ws folder you created, run
```bash
$ source <catkin_ws>/devel/setup.bash
$ rostopic echo /adis16495/data_raw
```
You will need to source the devel folder in this terminal prior to running rostopic, if you choose 
to publish the measurements using our custom message type `adi_imu_ros/AdiImu` or `adi_imu_ros/AdiImuRaw`. 

## Message types supported

* `std` - same as message [`sensor_msgs/Imu`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)
* `adi` - similar to `sensor_msgs/Imu` but contains mainly accelerometer and gyro outputs. [See for more details](./msg/AdiImu.msg)
* `adi_raw` - similar to `adi` but contains raw hex outputs from IMU. [See for more details](./msg/AdiImuRaw.msg)
* `csv` - writes all the outputs to CSV file 
* `csv_raw` - writes all the raw hex outputs to CSV file 
