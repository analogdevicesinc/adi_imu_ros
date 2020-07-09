#pragma once

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/Imu.h"
#include "adi_imu_ros/AdiImu.h"
#include "adi_imu_driver.h"
#include "spi_driver.h"
#include <cmath>
#include <fstream>
#include <string>

class AdiImuRos
{
private:
	// ROS variables
  ros::Publisher  _pub_imu;
  ros::NodeHandle _nh;
  std::string _imu_frame;
  // IMU variables
  adi_imu_Device_t _imu;
  uint64_t _driver_count, _imu_count;
  uint16_t _rollover;
  float _acclLSB, _gyroLSB, _tempLSB;
  // Logger and publisher variables
  std::ofstream _csv_stream;
  std::string _msg_type;

public:
	AdiImuRos(const ros::NodeHandle nh);
  ~AdiImuRos()
  {
    if(_csv_stream.is_open())
      _csv_stream.close();
  };
  void run(const std::function<void(const ros::Time, const ros::Time, const adi_imu_BurstOutput_t)>& pub_func);
  void publish_std_msg(const ros::Time t0, const ros::Time t1, const adi_imu_BurstOutput_t data);
  void publish_adi_msg(const ros::Time t0, const ros::Time t1, const adi_imu_BurstOutput_t data);
  void save_csv_file(const ros::Time t0, const ros::Time t1, const adi_imu_BurstOutput_t data);
};

