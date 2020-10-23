#include "ros/ros.h"
#include "serial_controllers/STM32_control.h"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Imu.h"
#include "serial_controllers/Pulse.h"
#include <geometry_msgs/Point.h>
#include <math.h>
#include <serial_controllers/AGV_Back.h>

#define IPI 0.0174532925199433333333  //没一度对应的弧度
#define LL  2.5
#define xxx 0
#define yyy -5
#define XXX 0
#define YYY 20
#define Unit_dis 0.05
#define Target_remote_x 0
#define Target_remote_y 15
#define Target_near_x 0
#define Target_near_y 1.5
struct Point
{
	float X;
	float Y;
	float Angle;
}Aim_Point,Path_1,Path_2,Path_3,Path_4,Terminal_Point,Original_Point,Target_Point_near,Target_Point_remote;

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
	float Last_Angle_IMU;
	float Current_X;			//当前X
	float Current_Y;			//当前Y
	float AGV_Back_Angle;
	float AGV_Back_Vel;
	float Unit_Y;				//单位时间前进Y
	float Current_Vel;			//当前速度
	float unit_Dis;
	int GET;				//当前状态0--停止/1--向前直行/2--远端掉头/3--返回直行/4--进端掉头
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
AGV_STA AGV_POS = {0,0,0,0,0,0,0,0,0,0,0};
int ptr_turnback_remote = 0;
int ptr_turnback_near = 0;
int Path_count = 0;

double control_angle(double Current_X,double Current_Y,double Current_Angle,struct Point *Aim)
{
	double result;
    double Angle;       //atan2(y2-y1,x2-x1) 计算向量去Y轴正向夹角（弧度-PI-PI）
    double a;
    double ld;          //当前AGV后轮中点与目标点之间的距离
    a = atan2(Aim->Y-Current_Y,Aim->X-Current_X) - PI/2;
    //a = atan2(Aim->Y-Current_Y,Aim->X-Current_X);

    if(a < -PI) a = a + 2*PI;
    Angle = a - Current_Angle*IPI;
    ld = sqrt(pow((Aim->Y-Current_Y)*1000,2) + pow((Aim->X-Current_X)*1000,2));
    result = atan(MY_AGV.WheelBase*sin(Angle)*2.0/ld);
    return result/IPI;
}

void Callback_Imu(const sensor_msgs::Imu& Imu_msg)
{
	AGV_POS.Last_Angle_IMU = AGV_POS.Current_Angle_IMU;
	AGV_POS.Current_Angle_IMU = Imu_msg.orientation.z;
	float differ_angle = AGV_POS.Current_Angle_IMU - AGV_POS.Last_Angle_IMU;
	if(differ_angle>300)differ_angle = 360 - differ_angle;
	else if(differ_angle<-300)differ_angle = differ_angle + 360;
	AGV_POS.Current_Angle_GPS = AGV_POS.Current_Angle_GPS + differ_angle;
}

float Get_Dis(double X,double Y,double x,double y)
{
	return sqrt(pow((x -X),2)+pow((y - Y),2));
}

void Build_Path_Test1(struct Point Point1,struct Point Point2,struct Point Point3,struct Point Point4)//跑道
{
	float R ;
	R = Get_Dis(Point2.X,Point2.Y,Point3.X,Point3.Y)/2;
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
}

void Build_Path_Test2(struct Point *_Original_Point,struct Point *Terminal_Point)
{
	Path_count = (int)(Get_Dis(_Original_Point->X,_Original_Point->Y,Terminal_Point->X,Terminal_Point->Y)/Unit_dis) + 1;
	float differ_x = Terminal_Point->X - _Original_Point->X;
	float differ_y = Terminal_Point->Y - _Original_Point->Y;
	for(int i=0;i<Path_count;i++ )
	{
		Path_Point[i][0] = _Original_Point->X + i*differ_x/Path_count;
		Path_Point[i][1] = _Original_Point->Y + i*differ_y/Path_count;
		cout<<i<<'\t'<<Path_Point[i][0]<<'\t'<<Path_Point[i][1]<<endl;
	}
	Path_Point[Path_count][0] = Terminal_Point->X;
	Path_Point[Path_count][1] = Terminal_Point->Y;
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


void turn_back_remote()
{
	//ptr_turnback_remote为0--还未转到50，为1即位转到50
	if(AGV_POS.Current_Angle_GPS>60&&ptr_turnback_remote==0)
		ptr_turnback_remote=1;
	switch(ptr_turnback_remote)
	{
	case 0:
		Test_cmd.Angle = 24;
		Test_cmd.Angle_vel = 10;
		Test_cmd.Brak = 0;
		Test_cmd.Gear = 4;
		Test_cmd.Park = 0;
		Test_cmd.Vel = 0.16;
		break;
	case 1:
		Test_cmd.Angle = -24;
		Test_cmd.Angle_vel = 10;
		Test_cmd.Brak = 0;
		Test_cmd.Gear = 2;
		Test_cmd.Park = 0;
		Test_cmd.Vel = 0.16;
		break;
	}
}

void turn_back_near()
{
	if((AGV_POS.Current_Angle_GPS>-120)&&(AGV_POS.Current_Angle_GPS<0)&&(ptr_turnback_near==0))
		ptr_turnback_near=1;
	switch(ptr_turnback_near)
	{
	case 0:
		Test_cmd.Angle = 24;
		Test_cmd.Angle_vel = 10;
		Test_cmd.Brak = 0;
		Test_cmd.Gear = 4;
		Test_cmd.Park = 0;
		Test_cmd.Vel = 0.16;
		break;
	case 1:
		Test_cmd.Angle = -24;
		Test_cmd.Angle_vel = 10;
		Test_cmd.Brak = 0;
		Test_cmd.Gear = 2;
		Test_cmd.Park = 0;
		Test_cmd.Vel = 0.16;
		break;
	}
}
void Callback_CMD(const geometry_msgs::Point& CMD_XYZ)
{
	Aim_Point.X = CMD_XYZ.x;
	Aim_Point.Y = CMD_XYZ.y;
}

void Callback_AGV_Back(const serial_controllers::AGV_Back _AGV_Back)
{
	AGV_POS.AGV_Back_Angle = _AGV_Back.back_angle;
	AGV_POS.AGV_Back_Vel = _AGV_Back.back_vel;
}

int main(int argc,char **argv)
{
	Terminal_Point.X = XXX;
	Terminal_Point.Y = YYY;
	Original_Point.X = xxx;
	Original_Point.Y = yyy;
	Target_Point_remote.X = Target_remote_x;
	Target_Point_remote.Y = Target_remote_y;
	Target_Point_near.X = Target_near_x;
	Target_Point_near.Y = Target_near_y;
	Build_Path_Test2(&Original_Point,&Terminal_Point);
	ros::init(argc, argv, "Test_node");
	ros::NodeHandle n;
	ros::Subscriber sub_1 = n.subscribe("imu_data",1,Callback_Imu);
	ros::Subscriber sub_2 = n.subscribe("GPS_XYZ",1,Callback_GPS);
	ros::Subscriber sub_3 = n.subscribe("CMD_XYZ",1,Callback_CMD);
	ros::Subscriber sub_4 = n.subscribe("AGV_Back",1,Callback_AGV_Back);
	ros::Publisher  pub = n.advertise<serial_controllers::STM32_control>("send_cmd_vel",100);
	ros::Rate loop_rate(20);
	AGV_POS.GET=1;
	while(ros::ok())
	{
		ros::spinOnce();
		float dis_remote = sqrt(pow(AGV_POS.Current_X - Target_Point_remote.X,2)+pow(AGV_POS.Current_Y - Target_Point_remote.Y,2));
		float dis_near = sqrt(pow(AGV_POS.Current_X - Target_Point_near.X,2)+pow(AGV_POS.Current_Y - Target_Point_near.Y,2));
		float dis_remote_to_near = sqrt(pow(Target_Point_remote.X - Target_Point_near.X,2)+pow(Target_Point_remote.Y - Target_Point_near.Y,2));

		if((-30<AGV_POS.Current_Angle_GPS)&&(AGV_POS.Current_Angle_GPS<30)&&(AGV_POS.GET!=1)&&(dis_remote>5))
			{
				AGV_POS.GET = 1;
				ptr_turnback_remote = 0;
				ptr_turnback_near = 0;//
			}
		if(((145<AGV_POS.Current_Angle_GPS)||(AGV_POS.Current_Angle_GPS<-145))&&(AGV_POS.GET!=3)&&(dis_near>5))
			{
			AGV_POS.GET = 3;
			ptr_turnback_near = 0;
			ptr_turnback_remote = 0;//
			}


		if(AGV_POS.GET==1)	//如果之前状态位前进直行
		{
			if(AGV_POS.Current_Y>Target_remote_y)
				{
					AGV_POS.GET++;
					ptr_turnback_remote = 0;//
					ptr_turnback_near = 0;//
				}
		}else if(AGV_POS.GET==2) //如果之前状态为远端掉头
		{
			if(AGV_POS.Current_Angle_GPS>145)
				{
				AGV_POS.GET++;
				ptr_turnback_remote=0;
				}
		}
		else if(AGV_POS.GET==3)		//如果之前状态为返回直行
		{
			if(AGV_POS.Current_Y<Target_near_y)
				{
					AGV_POS.GET++;
					ptr_turnback_remote = 0;//
					ptr_turnback_near = 0;//
				}
		}
		else if(AGV_POS.GET==4)		//如果之前状态为近端掉头
		{
			if((AGV_POS.Current_Angle_GPS>-45)&&(AGV_POS.Current_Angle_GPS<0))
				{
					AGV_POS.GET = 1;
					ptr_turnback_near = 0;
				}
		}

		switch(AGV_POS.GET)
		{
		case 0:	//停止
			Test_cmd.Angle = 0;
			Test_cmd.Angle_vel = 10;
			Test_cmd.Brak = 0;
			Test_cmd.Gear = 0;
			Test_cmd.Park = 0;
			Test_cmd.Vel = 0;
			break;
		case 1://前进直行
		{
			int num = Find_Point(&Path_Point[0],Path_count);
			Aim_Point.X = Path_Point[num][0];
			Aim_Point.Y = Path_Point[num][1];
			Test_cmd.Angle = (float)control_angle(AGV_POS.Current_X,AGV_POS.Current_Y,AGV_POS.Current_Angle_GPS,&Aim_Point);
			Test_cmd.Angle_vel = 10;
			Test_cmd.Brak = 0;
			Test_cmd.Gear = 4;
			Test_cmd.Park = 0;
			Test_cmd.Vel = 0.2;
			break;
		}
		case 2://掉头
			turn_back_remote();
			break;
		case 3://返回直行
		{
			int num = Find_Point_(&Path_Point[0],Path_count);
			Aim_Point.X = Path_Point[num][0];
			Aim_Point.Y = Path_Point[num][1];
			Test_cmd.Angle = (float)control_angle(AGV_POS.Current_X,AGV_POS.Current_Y,AGV_POS.Current_Angle_GPS,&Aim_Point);
			Test_cmd.Angle_vel = 10;
			Test_cmd.Brak = 0;
			Test_cmd.Gear = 4;
			Test_cmd.Park = 0;
			Test_cmd.Vel = 0.2;
			break;
		}
		case 4://掉头
			turn_back_near();
			break;
		}
		cout<<AGV_POS.GET<<'\t'<<dis_remote<<endl;
		cout<<"current_X:"<<AGV_POS.Current_X<<'\t'<<"current_Y:"<<AGV_POS.Current_Y<<'\t'<<"Aim_X:"<<Aim_Point.X<<'\t'<<"Aim_Y:"<<Aim_Point.Y<<endl;
		cout<<"GPS_angle:"<<AGV_POS.Current_Angle_GPS<<'\t'<<"Back_angle:"<<AGV_POS.AGV_Back_Angle<<'\t'<<"angle_cmd:"<<Test_cmd.Angle<<endl;
		pub.publish(Test_cmd);
		loop_rate.sleep();
	}

}
