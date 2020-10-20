/*
 * Data_fusion.cpp
 * 订阅GPS数据，IMU数据，进行数据融合
 * Created on: 2020年6月27日
 * Author: zzhao
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iomanip>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Point.h"
#include "serial_controllers/GPS.h"


geometry_msgs::Point Present_Point;		//当前

void GPS_Callback(const serial_controllers::GPS & msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7) << "纬度：" << msg.lat << " 经度：" << msg.lon << "\n";
}

void IMU_Callback(const sensor_msgs::Imu & msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Data_fusion_node");
    ros::NodeHandle n;
    ros::Subscriber GPS_rec = n.subscribe("GPS_data", 1000, GPS_Callback);
    ros::Subscriber IMU_rec = n.subscribe("IMU_data", 1000, IMU_Callback);
    ros::Publisher  Point_XY = n.advertise<sensor_msgs::Imu>("imu_data",100);
    ros::spin();
    return 0;
}
