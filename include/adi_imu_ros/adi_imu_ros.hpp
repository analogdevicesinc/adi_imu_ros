#pragma once

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

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
  //adi_imu_BurstOutput_t _data;
  uint64_t _count, _last_imu_count;
  uint16_t _rollover;
  // Logger and publisher variables
  bool _save_to_file;
  std::ofstream _csv_stream;
  std::string _msg_type;

public:
	AdiImuRos(const ros::NodeHandle nh);
  ~AdiImuRos()
  {
    if(_csv_stream.is_open())
      _csv_stream.close();
  };
  void run(void);
  // void publish_std_msg();
  // void publish_adi_msg();
  // void save_to_file(const ros::Time t1);
};

