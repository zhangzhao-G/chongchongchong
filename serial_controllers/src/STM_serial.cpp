/*
 * STM_serial.cpp
 * 与stm32进行通讯
 * 发送时数据末尾需要加上\t\n在数据头加上‘$’
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
//以下为串口通讯需要的头文件
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <serial/serial.h>
#include "serial_controllers/STM32_control.h"
#include "serial_controllers/Pulse.h"
#include "serial_controllers/AGV_Back.h"

#define r_BUFFERSIZE 	100				//接收数组的大小
#define s_BUFFERSIZE	13				//发送数组的大小
#define ID_sed_gear			0x41			//返回命令ID--档位控制
#define ID_sed_angle		0x42			//返回命令ID--转角控制
#define ID_sed_angle_vel	0x43			//返回命令ID--转动速度控制
#define ID_sed_vel			0x44			//返回命令ID--速度控制
#define ID_sed_bark			0x45			//返回命令ID--制动控制
#define ID_sed_park			0x46			//返回命令ID--泊车控制
#define ID_sed_odom			0x47			//返回命令ID--里程计控制
#define ID_rec_vel			0x0D			//命令ID

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::hex;

int USART_RX_STA = 0;					//串口接受控制标致
float linear_temp = 0, angular_temp=0;	//暂存的线速度和角速度

unsigned char data_begain_0 = 0x24;				//“$”字符
unsigned char data_terminal_0 = 0x0d;			//“/r”字符
unsigned char data_terminal_1 = 0x0a;			//“/n”字符
unsigned char Send_data[s_BUFFERSIZE] = {0x24,0x24,0,0,0,0,0,0,0,0,0,0x0d,0x0a};
unsigned char RX_Buffer[r_BUFFERSIZE];

//串口接收状态
struct RX_STA
{
	unsigned char Heard;
	unsigned char Length;
	unsigned char State;
}RX_STA;

serial::Serial my_serial;
serial_controllers::AGV_Back AGV_back;
serial_controllers::Pulse Wheel_Pulse;	//stm32返回的左右轮脉冲

void callback(const serial_controllers::STM32_control::ConstPtr & cmd_input)
{
	Send_data[2] = cmd_input->Gear;						//档位
	Send_data[3] = (int)(cmd_input->Vel/0.04);			//速度
	Send_data[4] = (int)(cmd_input->Brak/0.390625);		//制动
	Send_data[5] = cmd_input->Park;						//泊车
	Send_data[6] = cmd_input->Odom;						//里程计
	int Angle = ((cmd_input->Angle)+90)/0.043945;					//转角
	Send_data[7] = (char)(Angle&0xff);							//转角
	Send_data[8] = (char)((Angle>>8)&0xff);							//转角
	int Angle_vel = cmd_input->Angle_vel/0.073242;		//转角速度
	Send_data[9] = (char)(Angle_vel&0xff);						//转角速度
	Send_data[10] = (char)((Angle_vel>>8)&0xff);						//转角速度
	Send_data[11] = data_terminal_0;
	my_serial.write(Send_data,s_BUFFERSIZE);
}


int main(int argc,char **argv)
{
	ros::init(argc, argv, "STM_serial_node");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("send_cmd_vel",1000,callback);
	ros::Publisher  pub = n.advertise<serial_controllers::AGV_Back>("AGV_Back",100);
	try
		{
		    //my_serial.setPort("/dev/stm_serial");
			my_serial.setPort("/dev/ttyUSB0");
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

	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		ros::spinOnce();
		if(my_serial.available())				//如果接受到数据
		{
			int len;
			len = my_serial.available();
			if(len<100)
			{
			my_serial.read(RX_Buffer,len);	//将接收到的数据保存在buffer中，返回数据长度
			//for(int i=0;i<len;i++)cout<<hex<<(int)RX_Buffer[i]<<"  ";	//将接收到的数据输出到终端
			//cout<<endl;
			if(RX_Buffer[len-1]==data_terminal_0&&RX_Buffer[len-2]==data_terminal_1)							//与IMU采用相同的通讯协议
			{
				int Sum = 0;
				for(int i=0;i<len-3;i++)
				{
					Sum += RX_Buffer[i+1];
				}if((Sum&0xff) == RX_Buffer[len-3])RX_STA.State = RX_Buffer[0];
			}
			switch(RX_Buffer[0])
			{
				case 0x42:			//角度返回为 命令字0x42 + 角度低8位 + 角度高8位 + 校验和 + 数据尾部2字节
						AGV_back.back_angle = ((int)RX_Buffer[1] + (int)RX_Buffer[2]*256)*0.043975 - 90;
						RX_STA.State = 0;
				break;
			}
			pub.publish(AGV_back);
			}else my_serial.read(len);
		}
			loop_rate.sleep();
			/*
			if(RX_STA.State == ID_rec_vel)								//如果返回的是速度信息
			{
				for(int i=0;i<4;i++)
				{
					//vel_linear.data[i]  = RX_Buffer[i+2];				//将接收到的数据解析出来
					//vel_angular.data[i] = RX_Buffer[i+2+4];				//将接收到的数据解析出来
				}
				geometry_msgs::Twist rec_cmd_vel;
				//rec_cmd_vel.linear.x = vel_linear.Data;
				//rec_cmd_vel.angular.z = vel_angular.Data;
				pub.publish(rec_cmd_vel);								//将返回的数据解析之后发布出去
				RX_STA.Length = 0;
				RX_STA.State = 0;
			}else if(RX_STA.State == ID_sed_angle)						//若返回的是其他的信息
			{

			}
		}
		*/
		/*

		if(my_serial.available()){
			std_msgs::String serial_data;
			int i,len;
			char Res;
		      	len = my_serial.read(serial_data.data,my_serial.available());

				for(int i=0;i<len;i++)cout<<hex<<(int)RX_Buffer[i]<<"  ";	//将接收到的数据输出到终端
				cout<<endl;

			for(i=0;i<len;i++)
			{
				Res=serial_data.data[i];
				if((USART_RX_STA&0x8000)==0)
				{
					if(USART_RX_STA&0x4000)
					{
						if(Res!=0x0a)USART_RX_STA=0;
						else USART_RX_STA|=0x8000;
					}else {
						if(Res==0x0d)USART_RX_STA|=0x4000;
						else{
							RX_Buffer[USART_RX_STA&0X3FFF]=Res ;
							USART_RX_STA++;
							if(USART_RX_STA>(100-1))USART_RX_STA=0;
							}
						}

				}
			}
         	}
		if(USART_RX_STA&0x8000)
		{
			switch(RX_Buffer[0])
			case 0x47:
				if(RX_Buffer[1]>>7)
					Wheel_Pulse.left = int(RX_Buffer[1]&&0x7f);
				else
					Wheel_Pulse.left = - int(RX_Buffer[1]&&0x7f);
				if(RX_Buffer[2]>>7)
					Wheel_Pulse.right = int(RX_Buffer[2]&&0x7f);
				else
					Wheel_Pulse.right = - int(RX_Buffer[1]&&0x7f);
				pub.publish(Wheel_Pulse);
				break;
		}
		ros::spinOnce();
		loop_rate.sleep();
		*/
	}

}






