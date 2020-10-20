#include <iostream>
#include <cmath>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <serial_controllers/GPS.h>

using namespace std;
double Lat_Test = 0,lon_Test = 0;
long Test_count = 0;
void GPS_Test(const serial_controllers::GPS::ConstPtr &GPS)
{
	Test_count++;
	Lat_Test = (Lat_Test + GPS->lat)/Test_count;
	lon_Test = (lon_Test + GPS->lon)/Test_count;
	cout<<"count:"<<Test_count<<'\t'<<"lat:"<<Lat_Test<<'\t'<<"lon:"<<lon_Test<<endl;
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "GPS_Test_node");			//初始化GPS串口节点
	ros::NodeHandle n;								//创建节点句柄
	ros::Subscriber GPS_Deal_sub = n.subscribe("GPS_data",1000,GPS_Test);			//创建订阅者节点
	ros::spin();
    return 0;
}
