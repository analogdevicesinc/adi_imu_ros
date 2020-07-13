/*******************************************************************************
 *   @file   adi_imu_ros.cpp
 *   @brief  ROS node for ADI IMU ADIS16xxx
 *   @author Sundar Palani <sundar.palani@analog.com>
 *   @author Audren CLoitre <audren.cloitre@analog.com>
********************************************************************************/

#include <adi_imu_ros/adi_imu_ros.hpp>

#define MAX_BUF_LENGTH 1000 // should atleast greater than (imu_output_rate / fetch_rate). Ex: (4000Hz / 10Hz) = 400 
static adi_imu_BurstOutputRaw_t g_imu_buf[MAX_BUF_LENGTH] = {0};

AdiImuRos::AdiImuRos(const ros::NodeHandle nh) : 
	_nh(nh), 
	_en_isensor_buffer(false),
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
	_imu.g = 1.0;
	_imu.spiDev = spiDev.c_str();
	_imu.spiSpeed = static_cast<uint32_t>(spiSpeed);
	_imu.spiMode = static_cast<uint8_t>(spiMode);
	_imu.spiBitsPerWord = static_cast<uint8_t>(spiBitsPerWord);
	_imu.spiDelay = static_cast<uint32_t>(spiDelay);

	// Initialize SPI
	int ret = spi_Init(&_imu);
	if (ret < 0)
	{
		ROS_ERROR("Could not initialize SPI.");
		return;
	}

	if (_en_isensor_buffer)
	{
		/* Initialize IMU BUF first to stop any previous running capture and clear buffer*/
		ret = imubuf_init(&_imu);
		if (ret != adi_imu_Success_e)
		{
			ROS_ERROR("Could not initialize iSensor IMU buffer");
			return;
		}

		/* Read and print iSensor SPI Buffer info and config*/
		imubuf_DevInfo_t imuBufInfo;
		if ((ret = imubuf_GetInfo(&_imu, &imuBufInfo)) < 0)
		{
			ROS_ERROR("Could not get the iSensor IMU buffer info.");
			return;
		}
		if ((ret = imubuf_PrintInfo(&_imu, &imuBufInfo)) < 0)
		{
			ROS_ERROR("Could not print the iSensor IMU buffer info.");
			return;
		}

	}

	// Initialize IMU
	ret = adi_imu_Init(&_imu);
	if (ret != adi_imu_Success_e) 
	{
		ROS_ERROR("Could not initialize the IMU.");
		return;
	}

	// Set sensors output rate
	if ((ret = adi_imu_SetOutputDataRate(&_imu, static_cast<uint16_t>(outputRate))) < 0)
	{
		ROS_ERROR("Could not set the imu output data rate.");
		return;
	}

	// Set DATA ready interrupt pin TODO: add to params ?
	if ((ret = adi_imu_ConfigDataReady(&_imu, DIO1, POSITIVE)) < 0)
	{
		ROS_ERROR("Could not configure data ready interrupt");
		return;
	}
	if ((ret = adi_imu_SetDataReady(&_imu, ENABLE)) < 0)
	{
		ROS_ERROR("Could not enable data ready");
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

	if (_en_isensor_buffer)
	{
		/* set DIO pin config (both input and output) for iSensor SPI buffer */
		imubuf_ImuDioConfig_t dioConfig;
		dioConfig.dataReadyPin = IMUBUF_DIO1; // interrupt from imu to iSensor
		dioConfig.dataReadyPolarity = RISING_EDGE;
		dioConfig.ppsPin = 0x00;
		dioConfig.ppsPolarity = 0x0;
		dioConfig.passThruPin = 0x00;
		dioConfig.watermarkIrqPin = IMUBUF_DIO2; // interrupt from iSensor to host 
		dioConfig.overflowIrqPin = 0x00;
		dioConfig.errorIrqPin = 0x00;
		if ((ret = imubuf_ConfigDio(&_imu, dioConfig)) < 0)
		{
			ROS_ERROR("Could not configure iSensor IMU buffer DIO pins");
			return;
		}
		
		/* Read and print iSensor SPI Buffer info and config*/
		imubuf_DevInfo_t imuBufInfo;
		if ((ret = imubuf_GetInfo(&_imu, &imuBufInfo)) < 0)
		{
			ROS_ERROR("Could not get the iSensor IMU buffer info.");
			return;
		}
		if ((ret = imubuf_PrintInfo(&_imu, &imuBufInfo)) < 0)
		{
			ROS_ERROR("Could not print the iSensor IMU buffer info.");
			return;
		}

		/* set registers to read from IMU for every data ready interrupt */
		uint16_t bufPattern[] = {REG_SYS_E_FLAG, REG_TEMP_OUT, \
                                REG_X_GYRO_LOW, REG_X_GYRO_OUT, \
                                REG_Y_GYRO_LOW, REG_Y_GYRO_OUT, \
                                REG_Z_GYRO_LOW, REG_Z_GYRO_OUT, \
                                REG_X_ACCL_LOW, REG_X_ACCL_OUT, \
                                REG_Y_ACCL_LOW, REG_Y_ACCL_OUT, \
                                REG_Z_ACCL_LOW, REG_Z_ACCL_OUT, \
                                REG_DATA_CNT, REG_CRC_LWR, REG_CRC_UPR};
		uint16_t bufPatternLen = static_cast<uint16_t> (sizeof(bufPattern)/sizeof(uint16_t));
		if ((ret = imubuf_SetPatternAuto(&_imu, bufPatternLen, bufPattern)) < 0)
		{
			ROS_ERROR("Could not set register pattern for iSensor IMU buffer.");
			return;
		}
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
	else if (_msg_type.compare("adi_raw") == 0)
	{
		_pub_imu = _nh.advertise<adi_imu_ros::AdiImuRaw>("data_raw", 1000);
		run(std::bind(&AdiImuRos::publish_adi_raw_msg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	else if ((_msg_type.compare("csv") == 0) || (_msg_type.compare("csv_raw") == 0))
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
		if (_msg_type.compare("csv_raw") == 0)
			run(std::bind(&AdiImuRos::save_csv_raw_file, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		else
			run(std::bind(&AdiImuRos::save_csv_file, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	}
	else
		ROS_ERROR("The 'message_type' parameter should either be 'std', 'adi', 'csv', or 'csv_raw'. Shutting down.");
	return;
}


AdiImuRos::~AdiImuRos()
{
	if (_en_isensor_buffer)
	{
		uint16_t temp = 0;
    	if (imubuf_StopCapture(&_imu, &temp) < 0)
		{
			ROS_ERROR("Could not stop capture for iSensor IMU buffer.");
		}
	}
	if(_csv_stream.is_open())
		_csv_stream.close();
};


// void AdiImuRos::run(void (* pub_func)(const ros::Time, const ros::Time, const adi_imu_BurstOutput_t))
void AdiImuRos::run(const std::function<void(const ros::Time, const ros::Time, const void*)>& pub_func)
{
	adi_imu_BurstOutput_t data;
	unsigned burstStartIdx = 0;
	if (_en_isensor_buffer)
	{
		/* start iSensor buffer capture */
		uint16_t curBufCnt = 0;
		if (imubuf_StartCapture(&_imu, IMUBUF_TRUE, &curBufCnt) < 0)
		{
			ROS_ERROR("Could not start capture for iSensor IMU buffer.");
			return;
		} 
	}

	// Enter the loop
	while (ros::ok())
	{
		// Restart the loop if you don't get a valid data
		const ros::Time t_request = ros::Time::now();
		int ret = 0;
		int32_t read_cnt = 0;
		uint16_t buf_len = 0;

		if (_en_isensor_buffer)
		{
			ret = imubuf_ReadBufferAutoMax(&_imu, 50, &read_cnt, (uint16_t*)g_imu_buf, &buf_len);
		}
		else
		{
			ret = adi_imu_ReadBurst(&_imu, (uint8_t*)g_imu_buf, &data);
			read_cnt = 1;
		}
		const ros::Time t_receive = ros::Time::now();
		if (ret < 0)
			continue;

		for (int n=0; n<read_cnt; n++)
		{
			if (_en_isensor_buffer)
			{
				adi_imu_ScaleBurstOut_2(&_imu, g_imu_buf + n, &data);
			}

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
			if ((_msg_type.compare("adi_raw") == 0) || (_msg_type.compare("csv_raw") == 0))
			{
				pub_func(t_request, t_receive, static_cast<const void*>(g_imu_buf));
			}
			else
			{
				pub_func(t_request, t_receive, static_cast<const void*>(&data));
			}
			ROS_INFO_THROTTLE(1, "IMU status: %d  IMU data cnt: %ld  Driver data cnt: %ld\n", data.sysEFlag, _imu_count, _driver_count);
		}
	}
}


void AdiImuRos::publish_std_msg(const ros::Time t0, const ros::Time t1, const void* scaled_data)
{
	// Compute the timestamp
	const ros::Time timestamp = t0 + (t1 - t0)*0.5;
	const adi_imu_BurstOutput_t *data = static_cast<const adi_imu_BurstOutput_t*>(scaled_data);

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
	msg.angular_velocity.x = data->gyro.x * M_PI/180;
	msg.angular_velocity.y = data->gyro.y * M_PI/180;
	msg.angular_velocity.z = data->gyro.z * M_PI/180;
	msg.linear_acceleration.x = data->accl.x;
	msg.linear_acceleration.y = data->accl.y;
	msg.linear_acceleration.z = data->accl.z;

	// Publish
	_pub_imu.publish(msg);
}


void AdiImuRos::publish_adi_msg(const ros::Time t0, const ros::Time t1, const void* scaled_data)
{
	// Compute the timestamp
	const ros::Time timestamp = t0 + (t1 - t0)*0.5;
	const adi_imu_BurstOutput_t *data = static_cast<const adi_imu_BurstOutput_t*>(scaled_data);

	// Build the message
	adi_imu_ros::AdiImu msg;
	msg.header.stamp = timestamp;
	msg.header.frame_id = _imu_frame;
	msg.t_request = t0;
	msg.t_receive = t1;
	msg.imu_count = _imu_count;
	msg.driver_count = _driver_count;
	msg.temperature = data->tempOut;
	msg.error_flag = data->sysEFlag;
	msg.angular_velocity.x = data->gyro.x * M_PI/180;
	msg.angular_velocity.y = data->gyro.y * M_PI/180;
	msg.angular_velocity.z = data->gyro.z * M_PI/180;
	msg.linear_acceleration.x = data->accl.x;
	msg.linear_acceleration.y = data->accl.y;
	msg.linear_acceleration.z = data->accl.z;

	// Publish
	_pub_imu.publish(msg);
}


void AdiImuRos::publish_adi_raw_msg(const ros::Time t0, const ros::Time t1, const void* raw_data)
{
	// Compute the timestamp
	const ros::Time timestamp = t0 + (t1 - t0)*0.5;
	adi_imu_BurstOutputRaw_t data;
	if (_en_isensor_buffer)
	{
		data = *(static_cast<const adi_imu_BurstOutputRaw_t*>(raw_data));
	}
	else
	{
		adi_imu_ParseBurstOut(&_imu, static_cast<const uint8_t*>(raw_data), &data);
	}

	// Build the message
	adi_imu_ros::AdiImuRaw msg;
	msg.header.stamp = timestamp;
	msg.header.frame_id = _imu_frame;
	msg.t_request = t0;
	msg.t_receive = t1;
	msg.imu_count = _imu_count;
	msg.driver_count = _driver_count;
	msg.error_flag = data.sysEFlag;
	msg.temperature = data.tempOut;
	msg.gyroX = data.gyro.x;
	msg.gyroY = data.gyro.y;
	msg.gyroZ = data.gyro.z;
	msg.acclX = data.accl.x;
	msg.acclY = data.accl.y;
	msg.acclZ = data.accl.z;
	msg.datacnt = data.dataCntOrTimeStamp;
	msg.crc = data.crc;

	// Publish
	_pub_imu.publish(msg);
}


void AdiImuRos::save_csv_file(const ros::Time t0, const ros::Time t1, const void* scaled_data)
{
	// Save the measurements to file if the stream is open
	if(_csv_stream.is_open())
	{
		// Compute the timestamp
		const ros::Time timestamp = t0 + (t1 - t0)*0.5;
		const adi_imu_BurstOutput_t *data = static_cast<const adi_imu_BurstOutput_t*>(scaled_data);

		_csv_stream << data->sysEFlag << "," << _imu_count << "," << _driver_count << ",";
		_csv_stream << t0.toNSec() << "," << t1.toNSec() << ",";
		_csv_stream << std::setprecision(6);
		_csv_stream << data->tempOut << ",";
		_csv_stream << std::setprecision(18);
		_csv_stream << data->accl.x << "," << data->accl.y << "," << data->accl.z << ",";
		_csv_stream << data->gyro.x * M_PI/180 << "," << data->gyro.y * M_PI/180 << "," << data->gyro.z * M_PI/180 << ",";
		_csv_stream << std::endl;
	}
}


void AdiImuRos::save_csv_raw_file(const ros::Time t0, const ros::Time t1, const void* raw_data)
{
	// Save the measurements to file if the stream is open
	if(_csv_stream.is_open())
	{
		// Compute the timestamp
		const ros::Time timestamp = t0 + (t1 - t0)*0.5;

		adi_imu_BurstOutputRaw_t data;
		if (_en_isensor_buffer)
		{
			data = *(static_cast<const adi_imu_BurstOutputRaw_t*>(raw_data));
		}
		else
		{
			adi_imu_ParseBurstOut(&_imu, static_cast<const uint8_t*>(raw_data), &data);
		}

		_csv_stream << data.sysEFlag << "," << _imu_count << "," << _driver_count << ",";
		_csv_stream << t0.toNSec() << "," << t1.toNSec() << ",";
		_csv_stream << std::hex;
		_csv_stream << data.tempOut << ",";
		_csv_stream << data.accl.x << "," << data.accl.y << "," << data.accl.z << ",";
		_csv_stream << data.gyro.x << "," << data.gyro.y << "," << data.gyro.z << ",";
		_csv_stream << std::dec;
		_csv_stream << std::endl;
	}
}
