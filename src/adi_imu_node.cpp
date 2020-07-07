/*******************************************************************************
 *   @file   adi_imu_node.cpp
 *   @brief  ROS node for ADI IMU ADIS16xxx
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include "adi_imu_driver.h"
#include "spi_driver.h"
#include <cmath>

#define PI 3.141592653589793238

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
    ros::Publisher ad_imu_pub = n.advertise<std_msgs::String>("data_raw", 1000);

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
    float gyroLSB  = (4 * 10000 * 0.00625 / 655360000) * ( PI / 180); /* 0.00625 deg / 2^16 */
    float tempLSB = (1.0/140);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        /* 4. Burst read 10 samples */
        printf("\nPerforming burst read..\n");
        adi_imu_BurstOutput_t out;
        char imuout[250];
        for (int i=0; i<10; i++){
            if ((ret = adi_imu_ReadBurst(&imu, &out)) >= 0) {
                double accelX = out.accl.x * acclLSB;
                double accelY = out.accl.y * acclLSB;
                double accelZ = out.accl.z * acclLSB;
                sprintf(imuout, "sys_status=%x, temp=%fC, accX=%f, accY=%f, accZ=%f, gyroX=%f, gyroY=%f, gyroZ=%f datacnt_Or_ts=%d crc=0x%x", out.sysEFlag, 25 + out.tempOut * tempLSB, accelX, accelY, accelZ, out.gyro.x * gyroLSB, out.gyro.y * gyroLSB, out.gyro.z * gyroLSB, out.dataCntOrTimeStamp, out.crc);
                // printf("Pitch = %f deg \n", 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI);
                // printf("Roll = %f deg\n", 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI);
            };
        }
        msg.data = imuout;
        ROS_INFO("%s", imuout);
        ad_imu_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    
    return 0;
}