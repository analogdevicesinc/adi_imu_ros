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
