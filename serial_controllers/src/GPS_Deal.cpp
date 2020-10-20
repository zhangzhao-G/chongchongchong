#include <iostream>
#include <cmath>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <serial_controllers/GPS.h>

#define Point_to_Point 1
#define PI 3.1415926535

using namespace std;
ros::Publisher 	GPS_Deal_pub;		//定义发布者节点
geometry_msgs::Point 	AGV;		//定义AGV坐标
serial_controllers::GPS GPS_data;	//定义发布的数据
int zonewide = 3;

#if	Point_to_Point
struct Point
{
    double X;
    double Y;
};
struct  Point Point_0 = {-100,100}; //新坐标系原点坐标
struct  Point Point_1 = {-200,100};  //新坐标系Y轴上点的坐标
struct  Point Point_2 = {0,0};      //新坐标系Y轴的方向向量
struct  Point Point_3 = {0,1};      //旧坐标系Y轴的方向向量
struct  Point Point_4 = {11,1};     //向量表示
double Angle;                       //坐标系的旋转角度
#endif

//输入经纬度（单位为度）
void transport_GPS(const serial_controllers::GPS::ConstPtr &GPS)
{
    double B = GPS->lat;
    double L = GPS->lon;
    double L0;  //中央经线度数
    double X;   //高斯平面纵坐标
    double Y;   //高斯平面横坐标
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

#if Point_to_Point
    Point_2.X = Point_1.X - Point_0.X;
    Point_2.Y = Point_1.Y - Point_0.Y;      //得到转换后Y轴的方向向量
    Angle = (double)acos((Point_2.Y)/sqrt(pow(Point_2.X,2)+pow(Point_2.Y,2)));
    if(Point_2.X<0)Angle=Angle+PI;
    Point_4.X = (X - Point_0.X);
    Point_4.Y = (Y - Point_0.Y);
    X = Point_4.X*cos(Angle) - Point_4.Y*sin(Angle);
    Y = Point_4.X*sin(Angle) + Point_4.Y*cos(Angle);
#endif

    AGV.x = X;
    AGV.y = Y;
    GPS_Deal_pub.publish(AGV);
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "GPS_Deal_node");			//初始化GPS串口节点
	ros::NodeHandle n;								//创建节点句柄
	ros::Subscriber GPS_Deal_sub = n.subscribe("GPS_data",1000,transport_GPS);			//创建订阅者节点
	GPS_Deal_pub = n.advertise<geometry_msgs::Point>("GPS_XYZ",1000);	//创建发布者和消息类型
    //cout<<setprecision(10)<<endl;
	ros::spin();
    return 0;
}
