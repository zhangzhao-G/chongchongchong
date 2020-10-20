#include "ros/ros.h"
#include "serial_controllers/STM32_control.h"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Imu.h"
#include "serial_controllers/Pulse.h"
#include <geometry_msgs/Point.h>
#include <math.h>

#define IPI 0.0174532925199433333333  //没一度对应的弧度
#define LL  3
struct Point
{
	float X;
	float Y;
	float Angle;
}Aim_Point,Path_1,Path_2,Path_3,Path_4;

struct AGV_Mod{
    int Wheel_Dia;      //轮子直径
    int WheelBase;      //轴距
    int tread;          //轮距
    int Vel_Max;        //最大速度
    int Rad_Min;        //最小转弯半径
};

struct AGV_STA
{
	float Current_Angle_GPS;	//当前角度
	float Current_Angle_IMU;	//当前角度
	float Current_X;			//当前X
	float Current_Y;			//当前Y
	float Unit_Y;				//单位时间前进Y
	float Current_Vel;			//当前速度
	float unit_Dis;
	bool GET_0;				//是否到达目标点
	bool GET_1;				//
	bool Get_b;
};

struct AGV_Mod MY_AGV = {
    .Wheel_Dia = 420,
    .WheelBase = 850,
    .tread = 606,
    .Vel_Max = 2,
    .Rad_Min = 2300,
};

using namespace std;
double PI = 3.14159265;
serial_controllers::STM32_control Test_cmd;
double Path_Point[500][2];
float Path_Control[4][2];
AGV_STA AGV_POS = {0,0,0,0,0,0,0,0,0,0};
int time_count = 0;

int ptr_60 = 0;
int ptr_150 = 0;

double control_angle(double Current_X,double Current_Y,double Current_Angle,struct Point *Aim)
{
	double result;
    double Angle;       //atan2(y2-y1,x2-x1) 计算向量去Y轴正向夹角（弧度-PI-PI）
    double a;
    double ld;          //当前AGV后轮中点与目标点之间的距离
    a = atan2(Aim->Y-Current_Y,Aim->X-Current_X) - PI/2;
    if(a < - PI/2) a = PI*3/2 + a;
    Angle = a - Current_Angle*IPI;
    ld = sqrt(pow((Aim->Y-Current_Y)*1000,2) + pow((Aim->X-Current_X)*1000,2));
    result = atan(MY_AGV.WheelBase*sin(Angle)*2.0/ld);
    return result/IPI;
}

void Callback_Imu(const sensor_msgs::Imu& Imu_msg)
{
	AGV_POS.Current_Angle_IMU = Imu_msg.orientation.z;
}

float Get_Dis(double X,double Y,struct Point &Point2)
{
	return sqrt(pow((Point2.X -X),2)+pow((Point2.Y - Y),2));
}

void Build_Path_Test1(struct Point Point1,struct Point Point2,struct Point Point3,struct Point Point4)//跑道
{
	float R ;
	R = Get_Dis(Point2.X,Point2.Y,Point3)/2;
	for(int i = 0; i < 100 ;i++)
	{
		Path_Point[i][0] = Point1.X + i*(Point2.X - Point1.X)/100;
		Path_Point[i][1] = Point1.Y + i*(Point2.Y - Point1.Y)/100;
	}
	for(int j = 0; j < 100 ;j++)
	{
		Path_Point[j+100][0] = Point2.X + R*(1 - cos(j*PI/100));
		Path_Point[j+100][1] = Point2.Y + R*sin(j*PI/100);
	}
	for(int k = 0; k < 100 ;k++)
	{
		Path_Point[k+200][0] = Point3.X + k*(Point4.X - Point3.X)/100;
		Path_Point[k+200][1] = Point3.Y + k*(Point4.Y - Point3.Y)/100;
	}
	for(int l = 0; l < 100 ;l++)
	{
		Path_Point[l+300][0] = Point4.X - R*(1 - cos(l*PI/100));
		Path_Point[l+300][1] = Point4.Y - R*sin(l*PI/100);
	}
	Path_Point[400][0] = Point1.X;
	Path_Point[400][1] = Point1.Y;
	for(int k = 0;k<400;k++)
		cout<<Path_Point[k][0]<<'\t'<<Path_Point[k][1]<<endl;
}

void Build_Path_Test2()
{
	for(int i=0 ;i<250;i++ )
	{
		Path_Point[i][0] = 0;
		Path_Point[i][1] = -3+i*0.1;
	}
}

int Find_min(double Point[],int num)
{
	    int pos = 0;                    	//最小位置
	    double min__ = Point[0];                 //最小值
	    if( num > 1 )
	    {
	        for(int i=1; i<num; i++)
	        {
	            if( Point[i] < min__ )
	            {
	                min__ = Point[i];
	                pos = i;
	            }
	        }
	    }
	    return pos;
}

int Find_Point(double Point[][2],int num)		//正向
{
	double _x[num],_y[num],dir[num];
	int ptr = 0;
	double L = 0;
	for(int i=0;i<num;i++)
	{
		_x[i] = Point[i][0] - AGV_POS.Current_X;
		_y[i] = Point[i][1] - AGV_POS.Current_Y;
		dir[i] = sqrt(pow(_x[i],2)+pow(_y[i],2));
	}
	ptr = Find_min(dir,num);
	L = sqrt(pow((AGV_POS.Current_X - Point[ptr][0]),2)+pow((AGV_POS.Current_Y - Point[ptr][1]),2));
	while(L<LL&&ptr + 1<num)
	{
		double x = Point[ptr+1][0] - Point[ptr][0];
		double y = Point[ptr+1][1] - Point[ptr][1];
		L = L+sqrt(pow(x,2)+pow(y,2));
		ptr++;
	}

	if(ptr == num)
	{
		double x = AGV_POS.Current_X - Point[num][0];
		double y = AGV_POS.Current_Y - Point[num][1];
		if(sqrt(pow(x,2)+pow(y,2))<0.02)AGV_POS.GET_0 = 1;
	}
	return ptr;
}

int Find_Point_(double Point[][2],int num)		//反向
{
	double _x[num],_y[num],dir[num];
	int ptr = 0;
	double L = 0;
	for(int i=0;i<num;i++)
	{
		_x[i] = Point[i][0] - AGV_POS.Current_X;
		_y[i] = Point[i][1] - AGV_POS.Current_Y;
		dir[i] = sqrt(pow(_x[i],2)+pow(_y[i],2));
	}
	ptr = Find_min(dir,num);
	cout<<ptr<<endl;
	while(L<LL&&ptr-1>=0)
	{
		double x = Point[ptr-1][0] - Point[ptr][0];
		double y = Point[ptr-1][1] - Point[ptr][1];
		L = L+sqrt(pow(x,2)+pow(y,2));
		ptr--;
	}
	if(ptr == 0)
	{
		double x = AGV_POS.Current_X - Point[0][0];
		double y = AGV_POS.Current_Y - Point[0][1];
		if(sqrt(pow(x,2)+pow(y,2))<0.02)AGV_POS.GET_1 = 1;
	}
	return ptr;
}

void Set_Aim()
{

}
void Test_1()
{

}

void Callback_GPS(const geometry_msgs::Point& GPS_XYZ)
{
	AGV_POS.Current_X = GPS_XYZ.x;
	AGV_POS.Current_Y = GPS_XYZ.y;
	AGV_POS.Current_Angle_GPS = GPS_XYZ.z;
}


void turn_back()
{
	if(AGV_POS.Current_Angle_GPS<45&&ptr_60==0)
	{
	Test_cmd.Angle = 24;
	Test_cmd.Angle_vel = 10;
	Test_cmd.Brak = 0;
	Test_cmd.Gear = 4;
	Test_cmd.Park = 0;
	Test_cmd.Vel = 0.16;
	}else if(AGV_POS.Current_Angle_GPS<150&&ptr_150==0){
		ptr_60 = 1;
		Test_cmd.Angle = -24;
		Test_cmd.Angle_vel = 10;
		Test_cmd.Brak = 0;
		Test_cmd.Gear = 2;
		Test_cmd.Park = 0;
		Test_cmd.Vel = 0.16;
	}else
		{ptr_150 = 1;AGV_POS.Get_b = 1;}
}


void Callback_CMD(const geometry_msgs::Point& CMD_XYZ)
{
	Aim_Point.X = CMD_XYZ.x;
	Aim_Point.Y = CMD_XYZ.y;
}

int main(int argc,char **argv)
{
	Build_Path_Test2();
	ros::init(argc, argv, "Test_node");
	ros::NodeHandle n;
	ros::Subscriber sub_1 = n.subscribe("imu_data",1,Callback_Imu);
	ros::Subscriber sub_2 = n.subscribe("GPS_XYZ",1,Callback_GPS);
	ros::Subscriber sub_3 = n.subscribe("CMD_XYZ",1,Callback_CMD);
	ros::Publisher  pub = n.advertise<serial_controllers::STM32_control>("send_cmd_vel",100);
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		/*
		int num = Find_Point(&Path_Point[0],100);
		Aim_Point.X = Path_Point[num][0];
		Aim_Point.Y = Path_Point[num][1];
		double angle_test = control_angle(AGV_POS.Current_X,AGV_POS.Current_Y,AGV_POS.Current_Angle_GPS,&Aim_Point);
		if(AGV_POS.Current_Angle_GPS<0)AGV_POS.Current_X = AGV_POS.Current_X - 0.01*sin(AGV_POS.Current_Angle_GPS*IPI);
		else AGV_POS.Current_X = AGV_POS.Current_X - 0.01*sin(AGV_POS.Current_Angle_GPS*IPI);
		AGV_POS.Current_Y = AGV_POS.Current_Y + 0.01*sin(PI/2 - AGV_POS.Current_Angle_GPS*IPI);
		if(angle_test>0)AGV_POS.Current_Angle_GPS = AGV_POS.Current_Angle_GPS + (0.01/RRR)/IPI;
		else AGV_POS.Current_Angle_GPS = AGV_POS.Current_Angle_GPS - (0.01/RRR)/IPI;
		cout<<AGV_POS.Current_X<<'\t'<<AGV_POS.Current_Y<<'\t'<<AGV_POS.Current_Angle_GPS<<'\t'<<angle_test<<'\t'<<RRR<<endl;
		*/
		ros::spinOnce();
		if(AGV_POS.GET_0 == 0)	//正向还为到达
		{
			int num = Find_Point(&Path_Point[0],250);
			Aim_Point.X = Path_Point[num][0];
			Aim_Point.Y = Path_Point[num][1];
			Test_cmd.Angle = (float)control_angle(AGV_POS.Current_X,AGV_POS.Current_Y,AGV_POS.Current_Angle_GPS,&Aim_Point);
			cout<<Test_cmd.Angle<<endl;
			Test_cmd.Angle_vel = 10;
			Test_cmd.Brak = 0;
			Test_cmd.Gear = 4;
			Test_cmd.Park = 0;
			Test_cmd.Vel = 0.2;
		}else if(AGV_POS.Get_b == 0)turn_back();
			else if(AGV_POS.GET_1 ==0){					//正向到达
				int num = Find_Point_(&Path_Point[0],150);
				Aim_Point.X = Path_Point[num][0];
				Aim_Point.Y = Path_Point[num][1];
				Test_cmd.Angle = (float)control_angle(AGV_POS.Current_X,AGV_POS.Current_Y,AGV_POS.Current_Angle_GPS,&Aim_Point);
				Test_cmd.Angle_vel = 10;
				Test_cmd.Brak = 0;
				Test_cmd.Gear = 4;
				Test_cmd.Park = 0;
				Test_cmd.Vel = 0.2;
			}else if(AGV_POS.GET_1 ==1)
			{
				Test_cmd.Angle = 0;
				Test_cmd.Angle_vel = 10;
				Test_cmd.Brak = 80;
				Test_cmd.Gear = 4;
				Test_cmd.Park = 1;
				Test_cmd.Vel = 0;
			}
		pub.publish(Test_cmd);
		loop_rate.sleep();
	}

}
