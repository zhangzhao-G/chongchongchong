/*
 * STM_serial.cpp
 * 与stm32进行通讯
 * 发送时数据末尾需要加上\t\n
 * 接收时：数据头（1字节）+命令（1字节）+数据（n字节）+校验（1字节）
 * 校验位为命令+数据的和
 *  Created on: 2020年6月23日
 *  Author: zzhao
 */
//以下为ros需要的头文件


#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <serial_controllers/Test.h>
//以下为串口通讯需要的头文件
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <serial/serial.h>

#define r_BUFFERSIZE 	21				//接收数组的大小
#define s_BUFFERSIZE	14
#define res_data_length	8				//用于判断接收是否正确
#define ID_rec_vel		0x0D			//命令ID

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::hex;

int USART_RX_STA = 0;					//串口接受控制标致
float rat = 1000.0f;					//转速转换比，执行数度调整比例
float D = 0.5f;							//两轮间距，单位是m
float linear_temp = 0, angular_temp=0;	//暂存的线速度和角速度

unsigned char data_terminal_0 = 0x0d;			//“/r”字符
unsigned char data_terminal_1 = 0x0a;			//“/n”字符
unsigned char controller_data[s_BUFFERSIZE] = {0};	//要发送给串口的数据



serial::Serial my_serial;

//共用体 发送给下位机的左右轮的速度，里程计的坐标和方向
union floatData
{
	int Data;
	unsigned char data[4];
}cmd_data,linear_speed_data,linear_time_data,angular_data,position_x,position_y,vel_linear,vel_angular;

//回调函数向串口发送数据
void send_msg(const serial_controllers::Test::ConstPtr & cmd_input)
{
	cout<<"shoudao"<<endl;
	cmd_data.Data = cmd_input->cmd_test;
	linear_speed_data.Data  =cmd_input->vel_test;
	linear_time_data.Data = cmd_input->time_test;

	//将得到的速度、角度信息填充到发送数组中
	for(int i = 0; i<4; i++)
	{
		controller_data[i] = cmd_data.data[i];
		controller_data[i+4] = linear_speed_data.data[i];
		controller_data[i+8] = linear_time_data.data[i];
	}
	//将结尾标志符写入数组
	controller_data[12] = data_terminal_0;
	controller_data[13] = data_terminal_1;
	//串口发送
	my_serial.write(controller_data,14);
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "STM_serial_node");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("Test_Data",1000,send_msg);
	//打开串口
	try
		{
		    my_serial.setPort("/dev/stm32_serial");
		    my_serial.setBaudrate(115200);
		    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		    my_serial.setTimeout(to);
		    my_serial.open();
		} catch (serial::IOException &e)
			{
				ROS_ERROR_STREAM("Unable to open port ");
				return -1;
			}
		if (my_serial.isOpen())
		{
			ROS_INFO_STREAM("Serial Port initialized");
		} else {
			return -1;
			}

	ros::spin();
}






