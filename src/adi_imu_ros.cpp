/*******************************************************************************
 *   @file   adi_imu_ros.cpp
 *   @brief  ROS node for ADI IMU ADIS16xxx
 *   @author Sundar Palani <sundar.palani@analog.com>
 *   @author Audren CLoitre <audren.cloitre@analog.com>
********************************************************************************/

#include <adi_imu_ros/adi_imu_ros.hpp>


AdiImuRos::AdiImuRos(const ros::NodeHandle nh) : 
	_nh(nh), 
	_driver_count(0), 
	_imu_count(0),
	_rollover(0)
{
	// Load ROS parameters
	int prodId, spiSpeed, spiMode, spiBitsPerWord, spiDelay, outputRate;
	std::string spiDev, csv_folder;
	_nh.param<std::string>("csv_folder", csv_folder, "/tmp");
	_nh.param<std::string>("imu_frame", _imu_frame, "imu");
	_nh.param<std::string>("spi_dev", spiDev, "/dev/spidev0.0");
	_nh.param<std::string>("message_type", _msg_type, "std"); // "std", "adi", or "csv"
	_nh.param<int>("prod_id", prodId, 16545);
	_nh.param<int>("spi_speed", spiSpeed, 2000000);
	_nh.param<int>("spi_mode", spiMode, 3);
	_nh.param<int>("spi_bits_per_word", spiBitsPerWord, 8);
	_nh.param<int>("spi_delay", spiDelay, 0);
	_nh.param<int>("output_rate", outputRate, 10);

	// Pass the parameters to the IMU object
	_imu.prodId = static_cast<uint16_t>(prodId);
	_imu.spiDev = spiDev.c_str();
	_imu.spiSpeed = static_cast<uint32_t>(spiSpeed);
	_imu.spiMode = static_cast<uint8_t>(spiMode);
	_imu.spiBitsPerWord = static_cast<uint8_t>(spiBitsPerWord);
	_imu.spiDelay = static_cast<uint32_t>(spiDelay);

	// Set scaling factors for sensor outputs
	// NOTE: These should likely be given by the adi_imu_driver, based on the prodId
	_acclLSB  = 0.25 * 9.81 / 65536000; /* 0.25mg/2^16 */
	_gyroLSB  = (4 * 10000 * 0.00625 / 655360000) * ( M_PI / 180); /* 0.00625 deg / 2^16 */
	_tempLSB = (1.0/140);

	// Initialize IMU
	int ret = adi_imu_Init(&_imu);
	if (ret != adi_imu_Success_e) 
	{
		ROS_ERROR("Could not initialize the IMU.");
		return;
	}

	// Set sensors output rate
	uint16_t output_rate = static_cast<uint16_t>(outputRate);
	uint16_t dec_rate = (uint16_t)(4000 / output_rate) - 1;
	if ((ret = adi_imu_SetDecimationRate(&_imu, dec_rate)) < 0)
	{
		ROS_ERROR("Could not set the decimation rate.");
		return;
	}
	
	// Read and print IMU info and config
	adi_imu_DevInfo_t imuInfo;
	if ((ret = adi_imu_GetDevInfo(&_imu, &imuInfo)) < 0) 
	{
		ROS_ERROR("Could not get the IMU info.");
		return;
	}
	if ((ret = adi_imu_PrintDevInfo(&_imu, &imuInfo)) < 0) 
	{
		ROS_ERROR("Could not print the IMU info.");
		return;
	}

	// Sleep 2 seconds to let the IMU initialize
	ROS_INFO("Waiting 1 second for the IMU to finish to initialize.");
	ros::Duration(1.0).sleep();

	// Setup the publisher and pass its corresponding member-function to the 'run' function
	if (_msg_type.compare("std") == 0)
	{
		_pub_imu = _nh.advertise<sensor_msgs::Imu>("data_raw", 1000);
		run(std::bind(&AdiImuRos::publish_std_msg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	else if (_msg_type.compare("adi") == 0)
	{
		_pub_imu = _nh.advertise<adi_imu_ros::AdiImu>("data_raw", 1000);
		run(std::bind(&AdiImuRos::publish_adi_msg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	else if (_msg_type.compare("csv") == 0)
	{
		// Create the file name
		const std::string time_str = std::to_string(ros::Time::now().toNSec());
		const std::string csv_filename = csv_folder + "/imu_" + time_str + ".csv";

		// Open the file and fill-in the header
		_csv_stream.open(csv_filename.c_str(), std::ofstream::out | std::ofstream::trunc);
		if(_csv_stream.is_open())
		{
			_csv_stream << "error_flag,imu_count,driver_count,t_request(ns),t_receive(ns),temp(C),";
			_csv_stream << "accX(m/s/s),accY(m/s/s),accZ(m/s/s),";
			_csv_stream << "gyrX(rad/s),gyrY(rad/s),gyrZ(rad/s)";
			_csv_stream << std::endl;
			_csv_stream << std::scientific;
		}
		run(std::bind(&AdiImuRos::save_csv_file, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	else
		ROS_ERROR("The 'message_type' parameter should either be 'std', 'adi', or 'csv'. Shutting down.");
	return;
}


// void AdiImuRos::run(void (* pub_func)(const ros::Time, const ros::Time, const adi_imu_BurstOutput_t))
void AdiImuRos::run(const std::function<void(const ros::Time, const ros::Time, const adi_imu_BurstOutput_t)>& pub_func)
{
	// Enter the loop
	while (ros::ok())
	{
		// Restart the loop if you don't get a valid data
		adi_imu_BurstOutput_t data;
		const ros::Time t_request = ros::Time::now();
		const int ret = adi_imu_ReadBurst(&_imu, &data);
		const ros::Time t_receive = ros::Time::now();
		if (ret < 0)
			continue;

		// If this the first measurement, force match the driver count with the IMU's
		if (_driver_count == 0 && data.dataCntOrTimeStamp > 0)
			_driver_count = data.dataCntOrTimeStamp - 1;

		// Account for rollover before comparing the data count
		if((_imu_count > data.dataCntOrTimeStamp  + 65535*_rollover) && (_driver_count > 65000))
			++_rollover;
		const uint64_t current_imu_count = data.dataCntOrTimeStamp + 65535*_rollover;

		// Restart the loop if we already read this measurement
		if (current_imu_count <= _driver_count)
			continue;

		// Otherwise, it's a valid new measurement, update the counts
		++_driver_count;
		_imu_count = current_imu_count;

		// Publish to the appropriate publisher, communicate with the user, and loop around
		pub_func(t_request, t_receive, data);
		ROS_INFO_THROTTLE(1, "IMU status: %d  IMU data cnt: %ld  Driver data cnt: %ld\n", data.sysEFlag, _imu_count, _driver_count);
	}
}


void AdiImuRos::publish_std_msg(const ros::Time t0, const ros::Time t1, const adi_imu_BurstOutput_t data)
{
	// Scale the measurement
	const double accelX = data.accl.x * _acclLSB;
	const double accelY = data.accl.y * _acclLSB;
	const double accelZ = data.accl.z * _acclLSB;
	const double gyroX = data.gyro.x * _gyroLSB;
	const double gyroY = data.gyro.y * _gyroLSB;
	const double gyroZ = data.gyro.z * _gyroLSB;

	// Compute the timestamp
	const ros::Time timestamp = t0 + (t1 - t0)*0.5;

	// Build the message
	sensor_msgs::Imu msg;
	msg.header.stamp = timestamp;
	msg.header.frame_id = _imu_frame;
	msg.orientation.x = 0.0;
	msg.orientation.y = 0.0;
	msg.orientation.z = 0.0;
	msg.orientation.w = 1.0;
	msg.orientation_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	msg.angular_velocity_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	msg.linear_acceleration_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	msg.angular_velocity.x = gyroX;
	msg.angular_velocity.y = gyroY;
	msg.angular_velocity.z = gyroZ;
	msg.linear_acceleration.x = accelX;
	msg.linear_acceleration.y = accelY;
	msg.linear_acceleration.z = accelZ;

	// Publish
	_pub_imu.publish(msg);
}


void AdiImuRos::publish_adi_msg(const ros::Time t0, const ros::Time t1, const adi_imu_BurstOutput_t data)
{
	// Scale the measurement
	const double accelX = data.accl.x * _acclLSB;
	const double accelY = data.accl.y * _acclLSB;
	const double accelZ = data.accl.z * _acclLSB;
	const double gyroX = data.gyro.x * _gyroLSB;
	const double gyroY = data.gyro.y * _gyroLSB;
	const double gyroZ = data.gyro.z * _gyroLSB;
	const float temperature = data.tempOut * _tempLSB + 25.0f;

	// Compute the timestamp
	const ros::Time timestamp = t0 + (t1 - t0)*0.5;

	// Build the message
	adi_imu_ros::AdiImu msg;
	msg.header.stamp = timestamp;
	msg.header.frame_id = _imu_frame;
	msg.t_request = t0;
	msg.t_receive = t1;
	msg.imu_count = _imu_count;
	msg.driver_count = _driver_count;
	msg.temperature = temperature;
	msg.error_flag = data.sysEFlag;
	msg.angular_velocity.x = gyroX;
	msg.angular_velocity.y = gyroY;
	msg.angular_velocity.z = gyroZ;
	msg.linear_acceleration.x = accelX;
	msg.linear_acceleration.y = accelY;
	msg.linear_acceleration.z = accelZ;

	// Publish
	_pub_imu.publish(msg);
}


void AdiImuRos::save_csv_file(const ros::Time t0, const ros::Time t1, const adi_imu_BurstOutput_t data)
{
	// Scale the measurement
	const double accelX = data.accl.x * _acclLSB;
	const double accelY = data.accl.y * _acclLSB;
	const double accelZ = data.accl.z * _acclLSB;
	const double gyroX = data.gyro.x * _gyroLSB;
	const double gyroY = data.gyro.y * _gyroLSB;
	const double gyroZ = data.gyro.z * _gyroLSB;
	const float temperature = data.tempOut * _tempLSB + 25.0f;

	// Compute the timestamp
	const ros::Time timestamp = t0 + (t1 - t0)*0.5;

	// Save the measurements to file if the stream is open
  if(_csv_stream.is_open())
  {
    _csv_stream << data.sysEFlag << "," << _imu_count << "," << _driver_count << ",";
    _csv_stream << t0.toNSec() << "," << t1.toNSec() << ",";
    _csv_stream << std::setprecision(6);
    _csv_stream << temperature << ",";
    _csv_stream << std::setprecision(18);
    _csv_stream << accelX << "," << accelY << "," << accelZ << ",";
    _csv_stream << gyroX << "," << gyroY << "," << gyroZ << ",";
    _csv_stream << std::endl;
  }
}
