/*******************************************************************************
 *   @file   adi_imu_ros.cpp
 *   @brief  ROS node for ADI IMU ADIS16xxx
 *   @author Sundar Palani <sundar.palani@analog.com>
 *   @author Audren CLoitre <audren.cloitre@analog.com>
********************************************************************************/

#include <adi_imu_ros/adi_imu_ros.hpp>

AdiImuRos::AdiImuRos(const ros::NodeHandle nh) : _nh(nh), _count(0)
{
	// Load ROS parameters
	int prodId, spiSpeed, spiMode, spiBitsPerWord, spiDelay, outputRate;
	std::string spiDev, output_filename;
	_nh.param<std::string>("output_filename", output_filename, "/tmp/imu.csv");
	_nh.param<std::string>("imu_frame", _imu_frame, "imu");
	_nh.param<std::string>("spi_dev", spiDev, "/dev/spidev0.0");
	_nh.param<std::string>("message_type", _msg_type, "standard"); // "standard" or "adi"
	_nh.param<int>("prod_id", prodId, 16545);
	_nh.param<int>("spi_speed", spiSpeed, 2000000);
	_nh.param<int>("spi_mode", spiMode, 3);
	_nh.param<int>("spi_bits_per_word", spiBitsPerWord, 8);
	_nh.param<int>("spi_delay", spiDelay, 0);
	_nh.param<int>("output_rate", outputRate, 10);
	_nh.param<bool>("save_to_file", _save_to_file, false);

	// Setup the logger
	if(_save_to_file)
	{
		_csv_stream.open(output_filename.c_str(), std::ofstream::out | std::ofstream::trunc);
		if(_csv_stream.is_open())
		{
			_csv_stream << "count,timestamp(ns),accX(m/s/s),accY(m/s/s),accZ(m/s/s),";
			_csv_stream << "gyrX(rad/s),gyrY(rad/s),gyrZ(rad/s),temp(C),status(none)" << std::endl;
		}
	}

	// Pass the parameters to the IMU object
	_imu.prodId = static_cast<uint16_t>(prodId);
	_imu.spiDev = spiDev.c_str();
	_imu.spiSpeed = static_cast<uint32_t>(spiSpeed);
	_imu.spiMode = static_cast<uint8_t>(spiMode);
	_imu.spiBitsPerWord = static_cast<uint8_t>(spiBitsPerWord);
	_imu.spiDelay = static_cast<uint32_t>(spiDelay);

	// Initialize IMU
	int ret = adi_imu_Init(&_imu);
	if (ret != adi_imu_Success_e) 
	{
		ROS_ERROR("Could not initialize the IMU.");
		return;
	}

	// Set sensors output rate
	uint16_t output_rate = static_cast<uint16_t>(outputRate);
	uint16_t dec_rate = (uint16_t)(4250 / output_rate) - 1;
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

	// Setup the publisher
	if (_msg_type.compare("standard") == 0)
		_pub_imu = _nh.advertise<sensor_msgs::Imu>("data_raw", 1000);
	// else if (_msg_type.compare("adi") == 0)
		// _pub_imu = _nh.advertise<adi_imu_ros::AdiImu>("data_raw", 1000);
	else
	{
		ROS_ERROR("The 'message_type' parameter should either be 'standard' or 'adi'.");
		return;
	}

	// Run
	run();
}


void AdiImuRos::run(void)
{
	// Set scaling factors for sensor outputs
	// NOTE: These should likely be given by the adi_imu_driver, based on the prodId
	const float acclLSB  = 0.25 * 9.81 / 65536000; /* 0.25mg/2^16 */
	const float gyroLSB  = (4 * 10000 * 0.00625 / 655360000) * ( M_PI / 180); /* 0.00625 deg / 2^16 */
	const float tempLSB = (1.0/140);

	// Enter the loop
	while (ros::ok())
	{
		// Restart the loop if you don't get a valid data
		ros::Time t_request = ros::Time::now();
		int ret = adi_imu_ReadBurst(&_imu, &_data);
		if (ret < 0)
			continue;
		ros::Time t_response = ros::Time::now();

		// Restart the loop if we already read this measurement
		if (_data.dataCntOrTimeStamp <= _count)
			continue;

		// Otherwise, it's a valid new measurement, and we publish
		++_count;
		const double accelX = _data.accl.x * acclLSB;
		const double accelY = _data.accl.y * acclLSB;
		const double accelZ = _data.accl.z * acclLSB;
		const double gyroX = _data.gyro.x * gyroLSB;
		const double gyroY = _data.gyro.y * gyroLSB;
		const double gyroZ = _data.gyro.z * gyroLSB;
		ROS_INFO_THROTTLE(1, "IMU status: %d  data count: %d  read cnt: %d\n", _data.sysEFlag, _data.dataCntOrTimeStamp, _count);

		/* prepare imu sensor message */
		sensor_msgs::Imu msg;
		msg.header.stamp = t_response;
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

		_pub_imu.publish(msg);
		ros::spinOnce();
	}
	return;
}
