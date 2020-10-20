/*
 * GPS_serial.cpp
 * 接收GPS的数据
 * Created on: 2020年6月27日
 * Author: zzhao
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include "serial_controllers/GPS.h"
#include <geometry_msgs/Point.h>

#define GPS_to_Point 1			//开启GPS高斯投影转换
#define Point_to_Point 1		//开启局部坐标转换
#define Pub_GPS_MSG 0			//发布GPS消息
#define Pub_XYZ_MSG 0			//发布高斯坐标消息
#define Pub_xyz_MSG 1			//发布局部坐标消息
#define GPS_GNGGA	1			//接收GNGGA格式GPS
#define GPS_GPRMC	0			//接收GPRMC格式GPS
#define GPS_AVR		1			//接收AVR格式GPS
#define PI 3.1415926535
#define	Original_0_lat 		30.610013426694443694	//原点
#define Original_0_lon 		114.35320271044444951
#define	Original_1_lat		30.609924556944445584	//Y轴上的点
#define Original_1_lon		114.35322082530555576
using namespace std;

struct Point
{
    double X;
    double Y;
    double Angle;
};

struct GPS
{
    double lat;
    double lon;
    double angle;
};

std::string GPS_rec;				//GPS接收到的数据
#if GPS_GNGGA
std::string GPS_start_RMC = "$GNGGA";	//GPS数据头
#endif
#if GPS_GPRMC
std::string GPS_start_RMC = "$GNRMC";	//GPS数据头
#endif
std::string GPS_start_AVR = "$PTNL";	//GPS数据头
std::string GPS_end = "\r\n";			//GPS数据尾

struct  GPS Point_GPS = {0,0,0};		//GPS数据，经纬度
struct  Point Point_XYZ = {0,0,0};		//GPS数据，高斯投影后的坐标
struct  Point Point_xyz = {0,0,0};		//GPS数据，平面坐标转化那阿后的坐标

struct  Point Point_0 = {0,0,0}; 			//新坐标系原点坐标
struct  Point Point_1 = {0,0,0};			//新坐标系Y轴上点的坐标
struct  Point Point_2 = {0,0,0};     		//新坐标系Y轴的方向向量
struct  Point Point_3 = {0,1,0};     		//旧坐标系Y轴的方向向量
struct  Point Point_4 = {0,0,0};     		//向量表示
double Transport_Angle;                 	//坐标系的旋转角度
double Transport_Angle_du;
int zonewide = 3;							//定义投影带
int PTR = 0;								//是否受到固定差分信号
int count_PTR;								//连续受到的固定差分信号

void Transport_parameter()
{
    Point_2.X = Point_1.X - Point_0.X;
    Point_2.Y = Point_1.Y - Point_0.Y;      //得到转换后Y轴的方向向量
	Transport_Angle = (double)acos((Point_2.Y)/sqrt(pow(Point_2.X,2)+pow(Point_2.Y,2)));
    if(Point_2.X>0)Transport_Angle=2*PI-Transport_Angle;
    Transport_Angle_du = Transport_Angle/0.0174532925199433333333;
}

//输入经纬度（单位为度）
void transport_GPS_to_XYZ(const double &lat,const double &lon,const double &angle,double &X,double &Y,double &Angle)
{
    double B = lat;
    double L = lon;
    double L0;  //中央经线度数
    double N;   //卯酉圈曲率半径

    double a = 6378137.0;              //单位M
    //double b = 6356752.3142;           //单位M
    double e1_2 = 0.006694379901333;   //椭球第一偏心率的平方
    double e2_2 = 0.00673949674427;    //椭球第二偏心率的平方
    //double f = 1/298.25722356333;      //参考椭球体扁率
    double IPI = 0.0174532925199433333333;  //没一度对应的弧度
    double S;   //从赤道到纬度位B的平行圈子午线的弧度长
    double n;
    double l;
    double X1,X2,X3,Y1,Y2,Y3;
    int prjno = 0; //投影带号
    if (zonewide == 6){
            prjno = (int) (L / zonewide) + 1;
            L0 = prjno * zonewide - 3;
        }else
        {
            prjno = (int) ((L - 1.5) / 3) + 1;
            L0 = prjno * 3;
        }

    L0  = L0*IPI;       // 转为弧度
    L   = L*IPI;        // 转为弧度
    B   = B*IPI;        // 转为弧度

    N = a / sqrt(1 - e1_2*pow(sin(B),2));
    S = (double)111132.9525*B/IPI - (double)16038.4201*sin(2*B) + (double)16.8325*sin(4*B) - (double)0.022*sin(6*B) + (double)0.00003*sin(8*B);
    l = (L - L0);    //将秒转换为弧度
    n = e2_2*pow(cos(B),2); //辅助变量

    X1 = N*sin(B)*cos(B)*pow(l,2)/2;
    X2 = N*sin(B)*pow(cos(B),3)*(5-pow(tan(B),2) + 9*n + 4*pow(e2_2,2))*pow(l,4)/24;
    X3 = N*sin(B)*pow(cos(B),5)*(61-58*pow(tan(B),2)+pow(tan(B),4))*pow(l,6)/720;
    Y1 = N*cos(B)*l;
    Y2 = N*pow(cos(B),3)*(1-pow(tan(B),2)+n)*pow(l,3)/6;
    Y3 = N*pow(cos(B),5)*(5-18*pow(tan(B),2)+pow(tan(B),4)+14*n-58*pow(tan(B),2)*n)*pow(l,5)/120;
    X  = S + X1 + X2 + X3;
    Y  = Y1 + Y2 + Y3 + 500000;
    Angle = angle;
}

void transport_XYZ_to_xyz(const double &X,const double &Y,const double &Angle,double &x,double &y,double &angle)
{
    Point_4.X = (X - Point_0.X);
    Point_4.Y = (Y - Point_0.Y);
    x = -(Point_4.X*cos(Transport_Angle) + Point_4.Y*sin(Transport_Angle));
    y = -Point_4.X*sin(Transport_Angle) + Point_4.Y*cos(Transport_Angle);
    angle = 90 + Angle-Transport_Angle_du;
    if(angle > 180)angle = angle - 360;
}

int Campare_Start(int a,int b)
{
	if (a == -1) return b;
	else
	{
		if (b == -1)return a;
		else
		{
			if(a < b)return a;
			else return b;
		}
	}
}

//解析GPS数据
int GPS_data_deal(const std::string s, double &lat,double &lon,double &angle)	//传入完整的GPS数据包（包括数据头和数据尾），返回经纬度
{
	std::vector<std::string> GPS_vector;		//定义容器
	std::string::size_type pos_1,pos_2;
	pos_2 = s.find(",");						//搜索“，”
	pos_1 = 0;
	while( std::string::npos !=pos_2)			//pos_2不等于size_type的最大值时
	{
		GPS_vector.push_back(s.substr(pos_1,pos_2-pos_1));	//pos_到pos_2中的内容放入容器
		pos_1 = pos_2 + 1;								//pos_1移位到pos_2后一位
		pos_2 = s.find(",",pos_1);							//pos_2从pos_1开始再次搜索“，”
	}
	if( pos_1 != s.length())		//如果pos_1不等于字符串长度
	{
		GPS_vector.push_back(s.substr(pos_1));	//将pos_1后面的放入容器中
	}
#if GPS_GPRMC
	if(GPS_vector[0] == "$GPRMC")	//检验GPS数据
	{
		if(GPS_vector[2] == "A")
		{
			if(GPS_vector[3] !="")
			{
				lat = ((double)std::atof(GPS_vector[3].c_str()))/100;		//解析出纬度
				int ilat = (int)floor(lat);
				lat = ilat + (lat - ilat)*100/60;
				cout<<setprecision(20)<<lat<<endl;
			}
			if(GPS_vector[5] != "")
			{
				lon = ((double)std::atof(GPS_vector[5].c_str()))/100;		//解析出经度
				int ilon = (int)floor(lon);
				lon = ilon + (lon - ilon)*100/60;
				cout<<setprecision(20)<<lon<<endl;
			}
		}
	}else if(GPS_vector[1] == "AVR")
		{
			angle = std::atof(GPS_vector[3].c_str());
		}
#endif

#if GPS_GNGGA
	if(GPS_vector[0] == "$GNGGA")	//检验GPS数据
	{
		cout<<GPS_vector[7]<<endl;
		if(GPS_vector[6] !="0")
		{
			lat = (double)std::atof(GPS_vector[2].c_str())/100;		//解析出纬度
			int ilat = (int)floor(lat);
			lat = ilat + (lat - ilat)*100/60;
			lon = (double)std::atof(GPS_vector[4].c_str())/100;		//解析出经度
			int ilon = (int)floor(lon);
			lon = ilon + (lon - ilon)*100/60;
		}
		if(GPS_vector[6] == "4")count_PTR++;
		else count_PTR=0;
		if(count_PTR > 10)
			{
			count_PTR--;
			PTR = 1;
			}else PTR =0;
	}else if(GPS_vector[1] == "AVR")
		{
			angle = std::atof(GPS_vector[3].c_str());
		}
#endif

	return 0;
}


int main(int argc,char **argv)
{
	serial_controllers::GPS GPS_data;
	geometry_msgs::Point AGV;		//定义AGV坐标
	serial::Serial GPS_serial;			//定义一个串口对象
	ros::init(argc, argv, "GPS_serial_node");		//初始化GPS串口节点
	ros::NodeHandle n;								//创建节点句柄
	ros::Publisher GPS_data_pub = n.advertise<serial_controllers::GPS>("GPS_data",1000);	//创建发布者和消息类型
	ros::Publisher GPS_Deal_pub = n.advertise<geometry_msgs::Point>("GPS_XYZ",1000);		//创建发布者和消息类型
	//串口配置
	transport_GPS_to_XYZ(Original_0_lat,Original_0_lon,0,Point_0.X,Point_0.Y,Point_0.Angle);
	transport_GPS_to_XYZ(Original_1_lat,Original_1_lon,0,Point_1.X,Point_1.Y,Point_1.Angle);
	Transport_parameter();

	cout<<Transport_Angle<<endl;

	cout<<Point_xyz.X<<'\t'<<Point_xyz.Y<<endl;
	try
	{
		GPS_serial.setPort("/dev/ttyS1");							//设置串口号
		GPS_serial.setBaudrate(115200);								//设置串口波特率
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);	//设置等待时间
		GPS_serial.setTimeout(to);
		GPS_serial.open();	//打开串口
	}catch(serial::IOException& e)	//如果打开串口失败
	{
		ROS_ERROR_STREAM("Unable to open GPS_Serial Port !");
		return -1;
	}
	if(GPS_serial.isOpen())	//如果串口已经打开
	{
		ROS_INFO_STREAM("GPS_Serial Port initialized");
	}else
	{
		return -1;
	}

	ros::Rate loop_rate(20);
	//循环接收
	while(ros::ok())
	{
		if(GPS_serial.available())	//是否有数据传入
		{
			GPS_rec +=GPS_serial.read(GPS_serial.available());//存储接收到的数据
			int i = 0, start = -1 ,start_RMC = -1,start_AVR = -1 ,end = -1;
			bool GPS_STA = 0;					//接收标志位	为0即没有收到完整的数据包
			while(i < GPS_rec.length())
			{
				start_RMC = GPS_rec.find(GPS_start_RMC);	//找到数据头
				start_AVR = GPS_rec.find(GPS_start_AVR);	//找到数据头
				start = Campare_Start(start_RMC,start_AVR);
				if( start == -1)	//如果没有找到，将保存的数据只保留最后四位，其余的全部去掉
				{
					if(GPS_rec.length()>4)
						GPS_rec = GPS_rec.substr(GPS_rec.length()-4);
					break;
				}
				else				//如果找到了数据头，则寻找数据尾
				{
					end = GPS_rec.find(GPS_end);
					if(end == -1||end < start)	//	如果没有数据尾，则将数据头前面的数据丢弃
					{
							GPS_rec = GPS_rec.substr(start);
						break;
					}else 	//接收到一个完整的数据包
						{
							i = end;
							GPS_STA = 1;//找到了一个完整的数据包
							GPS_data_deal(GPS_rec.substr(start, end-start+2),Point_GPS.lat,Point_GPS.lon,Point_GPS.angle);//进行数据解析得到经纬度
							//if(PTR == 1){
							#if	Pub_GPS_MSG//GPS数据发布
							GPS_data.lat = Point_GPS.lat;				//填充消息——纬度
							GPS_data.lon = Point_GPS.lon;				//填充消息——经度
							GPS_data_pub.publish(GPS_data);				//发布GPS消息
							#endif

							#if GPS_to_Point
							transport_GPS_to_XYZ(Point_GPS.lat,Point_GPS.lon,Point_GPS.angle,Point_XYZ.X,Point_XYZ.Y,Point_XYZ.Angle);
							#endif

							#if Pub_XYZ_MSG//XYZ数据发布
							AGV.x = Point_XYZ.X;
						    AGV.y = Point_XYZ.Y;
						    AGV.z = Point_XYZ.Angle;
						    GPS_Deal_pub.publish(AGV);
							#endif

							#if Point_to_Point
						    transport_XYZ_to_xyz(Point_XYZ.X,Point_XYZ.Y,Point_XYZ.Angle,Point_xyz.X,Point_xyz.Y,Point_xyz.Angle);
							#endif

						    #if Pub_xyz_MSG//xyz数据发布
						    AGV.x = Point_xyz.X;
						    AGV.y = Point_xyz.Y;
						    AGV.z = 0 - Point_xyz.Angle;
						    GPS_Deal_pub.publish(AGV);
						    #endif
							//}
							if(i+5<GPS_rec.length())			//如果数据尾后面的数据大于4个则继续循环寻找数据，否着跳出循环
							{
								GPS_rec = GPS_rec.substr(end+2);
								i = 0;
							}
							else
							{
								GPS_rec = GPS_rec.substr(end+2);
								break;
							}

						}

				}
			}

		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}
