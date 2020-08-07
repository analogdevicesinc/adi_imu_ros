#pragma once

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/Imu.h"
#include "adi_imu_ros/AdiImu.h"
#include "adi_imu_ros/AdiImuRaw.h"
#include "adi_imu_driver.h"
#include "spi_driver.h"
#include "imu_spi_buffer.h"
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
  bool _en_isensor_buffer, _en_isensor_burst_mode;
  uint64_t _driver_count, _imu_count;
  uint32_t _stall_time_capture, _stall_time_config;
  uint16_t _rollover;
  // Logger and publisher variables
  std::ofstream _csv_stream;
  std::string _msg_type;

public:
  AdiImuRos(const ros::NodeHandle nh);
  ~AdiImuRos();
  void run(const std::function<void(const ros::Time, const ros::Time, const void*)>& pub_func);
  void publish_std_msg(const ros::Time t0, const ros::Time t1, const void* scaled_data);
  void publish_adi_msg(const ros::Time t0, const ros::Time t1, const void* scaled_data);
  void publish_adi_raw_msg(const ros::Time t0, const ros::Time t1, const void* raw_data);
  void save_csv_file(const ros::Time t0, const ros::Time t1, const void* scaled_data);
  void save_csv_raw_file(const ros::Time t0, const ros::Time t1, const void* raw_data);
};
