/*******************************************************************************
 *   @file   adi_imu_ros_node.cpp
 *   @brief  ROS node for ADI IMU ADIS16xxx
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include "adi_imu_driver.h"
#include "spi_driver.h"
#include <cmath>

int main(int argc, char **argv)
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.spiDev = "/dev/spidev0.0";
    imu.spiSpeed = 2000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 0;

    /* initializate ROS */
    ros::init(argc, argv, "adi_imu");
    ros::NodeHandle n("~");
    
    /* create publisher */
    ros::Publisher ad_imu_pub = n.advertise<sensor_msgs::Imu>("data_raw", 1000);

    /* setting data output rate */
    ros::Rate loop_rate(10);

    /* Initialize IMU */
    int ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return -1;

    /* Set sensors output rate */
    uint16_t output_rate = 10; // Hz
    uint16_t dec_rate = (uint16_t)(4250 / output_rate) - 1;
    if ((ret = adi_imu_SetDecimationRate(&imu, dec_rate)) < 0) return ret;
    
    /* Read and print IMU info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return -1;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return -1;

    /* set scaling factors for sensor outputs */
    float acclLSB  = 0.25 * 9.81 / 65536000; /* 0.25mg/2^16 */
    float gyroLSB  = (4 * 10000 * 0.00625 / 655360000) * ( M_PI / 180); /* 0.00625 deg / 2^16 */
    float tempLSB = (1.0/140);

    double accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
    adi_imu_BurstOutput_t out;
    // char imuout[250];

    int count = 0;
    while (ros::ok())
    {
        sensor_msgs::Imu msg;

        for (int i=0; i<10; i++){
            if ((ret = adi_imu_ReadBurst(&imu, &out)) >= 0) {
                accelX = out.accl.x * acclLSB;
                accelY = out.accl.y * acclLSB;
                accelZ = out.accl.z * acclLSB;
                gyroX = out.gyro.x * gyroLSB;
                gyroY = out.gyro.y * gyroLSB;
                gyroZ = out.gyro.z * gyroLSB;
                // sprintf(imuout, "sys_status=%x, temp=%fC, accX=%f, accY=%f, accZ=%f, gyroX=%f, gyroY=%f, gyroZ=%f datacnt_Or_ts=%d crc=0x%x", out.sysEFlag, 25 + out.tempOut * tempLSB, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, out.dataCntOrTimeStamp, out.crc);
                // printf("Pitch = %f deg \n", 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/M_PI);
                // printf("Roll = %f deg\n", 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/M_PI);
            };
        }
        ROS_INFO("IMU status: %d  data count: %d  read cnt: %d\n", out.sysEFlag, out.dataCntOrTimeStamp, count);

        /* prepare imu sensor message */
        msg.header.seq = count;
        msg.header.stamp.sec = 0;
        msg.header.stamp.nsec = 0;
        msg.header.frame_id = "adi_imu";

        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;
        msg.orientation.w = 0;

        msg.orientation_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        msg.angular_velocity_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        msg.linear_acceleration_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        msg.angular_velocity.x = gyroX;
        msg.angular_velocity.y = gyroY;
        msg.angular_velocity.z = gyroZ;
        msg.linear_acceleration.x = accelX;
        msg.linear_acceleration.y = accelY;
        msg.linear_acceleration.z = accelZ;

        ad_imu_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}