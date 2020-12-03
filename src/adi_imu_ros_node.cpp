/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		adi_imu_ros_node.cpp
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @author		Audren Cloitre (audren.cloitre@analog.com)
  * @brief 		ADI IMU Ros node main.
 **/

#include <ros/ros.h>
#include <adi_imu_ros/adi_imu_ros.hpp>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "adi_imu_ros");
  ros::NodeHandle nh("~");

  // AdiImuRos object.
  AdiImuRos adi_imu_ros(nh);

  // run..
  ros::spin();

  return 0;
} // end main()
