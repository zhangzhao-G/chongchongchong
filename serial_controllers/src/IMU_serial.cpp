/*
 * IMU_serial.cpp
 * 接收IMU的数据
 * Created on: 2020年6月27日
 * Author: zzhao
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <serial_controllers/IMU_CMD.h>
#define Buffer_length	100
using std::cout;
using std::endl;
using std::string;
using std::hex;

serial::Serial IMU_serial;			//声明串口对象
sensor_msgs::Imu imu_data;			//定义IMU数据

double Last_Acc = 0;
double Current_Vel = 0;
double Last_Vel = 0;
double Unit_Y = 0;
double Current_Y = 0;



unsigned char IMU_RX_Buffer[Buffer_length];
unsigned char IMU_TX_Buffer_1[5] = {0x68,0x04,0x00,0x28,0x2c};
unsigned char IMU_TX_Buffer_2[5] = {0x68,0x05,0x00,0x03,0x14};
struct RX_STA
{
	unsigned char Heard;
	unsigned char Length;
	unsigned char State;
}IMU_RX_STA;
//初始化状态

//回调函数
int count = 0;
void Callback(const std_msgs::String::ConstPtr& msg)
{

	ROS_INFO_STREAM("Writing to IMU_serial port" <<msg->data);
	if(count%2 == 0)IMU_serial.write(IMU_TX_Buffer_1,5);
	if(count%2 == 1)IMU_serial.write(IMU_TX_Buffer_2,5);
	count++;
}

//主函数
int main(int argc, char **argv)
{
	ros::init(argc, argv, "IMU_serial_node");	//初始化节点
	ros::NodeHandle n;
	ros::Subscriber IMU_write_pub = n.subscribe("IMU_command",1000,Callback);
	ros::Publisher  IMU_read_pub = n.advertise<sensor_msgs::Imu>("imu_data",1);
	//打开串口
	try
	{
		IMU_serial.setPort("/dev/ttyS0");
		IMU_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(100);
		IMU_serial.setTimeout(to);
		IMU_serial.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open IMU serial port ");
		return -1;
	}
	if(IMU_serial.isOpen())
	{
		ROS_INFO_STREAM("IMU Serial Port initialized");
	}else{
		return -1;
	}

	ros::Rate loop_rate(50);
	IMU_serial.read(IMU_serial.available());
	while(ros::ok())
	{
		if(IMU_serial.available())				//如果缓冲区有数据
				{
					IMU_RX_STA.Heard = 0;
					IMU_RX_STA.Length = 0;
					IMU_RX_STA.State = 0;

					int len;
					len = IMU_serial.read(IMU_RX_Buffer,IMU_serial.available());	//将接收到的数据保存在buffer中，返回数据长度
					//for(int i=0;i<len;i++)cout<<hex<<(int)IMU_RX_Buffer[i]<<"  ";	//将接收到的数据输出到终端
					//cout<<endl;

					unsigned char IMU_RX_Data[100]; 				//数据拆解
					//校验数据
					if(IMU_RX_Buffer[0] == 0x68)					//若收到的第一个字符为0x68
					{
						int Sum = 0;//校验
						IMU_RX_STA.Length = IMU_RX_Buffer[1];					//读取数据包长度
						if(len >= (IMU_RX_STA.Length + 1)){
						for(int i=0;i<(IMU_RX_STA.Length-1);i++)
						{
							Sum += IMU_RX_Buffer[i+1];
						}
						if((Sum&0xff)==IMU_RX_Buffer[IMU_RX_STA.Length])IMU_RX_STA.State = IMU_RX_Buffer[3];//校验无误后将命令ID传给IMU_RX_STA.State
						}
					}
					//解析数据
					if(IMU_RX_STA.State == 0x84)	//接收到的是角度
					{
						for(int i=0;i<(IMU_RX_STA.Length-1);i++)
						{
							IMU_RX_Data[i*2] = IMU_RX_Buffer[i+1]>>4;			//将数据包中的高4位和低4为分开
							IMU_RX_Data[i*2+1] = IMU_RX_Buffer[i+1]&0x0f;
						}
						sensor_msgs::Imu imu_data;
						imu_data.header.stamp = ros::Time::now();
						imu_data.header.frame_id = "xxx";
						imu_data.linear_acceleration.y = ((float)IMU_RX_Data[13]*10 + (float)IMU_RX_Data[14] + (float)IMU_RX_Data[15]*0.1 + (float)IMU_RX_Data[16]*0.01 + (float)IMU_RX_Data[17]*0.001)*9.8;				//前进加速度
						imu_data.angular_velocity.z = (float)IMU_RX_Data[7]*100 + (float)IMU_RX_Data[8]*10 + (float)IMU_RX_Data[9] + (float)IMU_RX_Data[10]*0.1 + (float)IMU_RX_Data[11]*0.01;						//角速率
						imu_data.orientation.z = (float)IMU_RX_Data[19]*100 + (float)IMU_RX_Data[20]*10 + (float)IMU_RX_Data[21] + (float)IMU_RX_Data[22]*0.1 + (float)IMU_RX_Data[23]*0.01;						//z轴方位角
						if(IMU_RX_Data[6]==1)imu_data.angular_velocity.z = -(imu_data.angular_velocity.z);
						if(IMU_RX_Data[12]==1)imu_data.linear_acceleration.y = -(imu_data.linear_acceleration.y);
						if(IMU_RX_Data[18]==1)imu_data.orientation.z = -(imu_data.orientation.z);

						//Current_Vel = Last_Vel + (Last_Acc + imu_data.linear_acceleration.y)/50;
						//Unit_Y = Current_Vel/25;
						//Current_Y = Current_Y + Unit_Y;
						//Last_Acc = imu_data.linear_acceleration.y;
						//Last_Vel = Current_Vel;
						//cout<<imu_data.linear_acceleration.y<<'\t'<<Current_Vel<<'\t'<<Unit_Y<<'\t'<<Current_Y<<endl;

						//imu_data.linear_acceleration.z = Current_Y;
						IMU_read_pub.publish(imu_data);
						IMU_RX_STA.Length = 0;
						IMU_RX_STA.State = 0;
					}else if(IMU_RX_STA.State==0x8c||IMU_RX_STA.State==0x8b||IMU_RX_STA.State==0x28)							//现实返回数据
						{
							ROS_INFO("return:%d",IMU_RX_STA.State);
						}
				}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

