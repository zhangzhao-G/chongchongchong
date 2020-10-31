/*
 *
 *
 *
 */
//#include "ros/ros.h"
//#include "serial_controllers/STM32_control.h"
#include <iostream>
#include "GPS.h"
#include "STM_Class.h"
//#include <nav_msgs/Odometry.h>
//#include "sensor_msgs/Imu.h"
//#include "serial_controllers/Pulse.h"
//#include <geometry_msgs/Point.h>
#include <cmath>
#include <fstream>
class AGV{
public:
    int Wheel_Dia;              //轮子直径
    int WheelBase;              //轴距
    int tread;                  //轮距
    int Vel_Max;                //最大速度
    int Rad_Min;                //最小转弯半径
    float Foresee_Dir;          //前视距离
	float Current_Angle_GPS;	//当前车身角度
	float Current_X_GPS;        //GPS返回的坐标X
	float Current_Y_GPS;        //GPS返回的坐标Y
	float Current_Angle_IMU;	//当前车身角度
	float Current_Angle;        //当前车身角度
	float Current_Vel;			//当前速度
	float Current_X;			//当前X
	float Current_Y;			//当前Y
    float Unit_Dir;             //单位距离
    float Unit_Angle;           //单位角度
	float Aim_Point_X;		    //目标点X
	float Aim_Point_Y;		    //目标点Y
	int Aim_Point_Ptr;          //目标点是路径点中的第几个点
	float Terminal_X;           //终点X
	float Terminal_Y;           //终点Y
	float Path_Terminal_Dis;    //离路径终点的距离
	float (*Path_Point_Ptr)[2] = NULL;//声明空指针,用于NEW一个路径
	int Path_Point_count;       //路径点的个数,从1开始
	float Key_Point[20][4];     //关键点20个、关键点的坐标和在关键点需要转的角度
    int Key_Point_Num;          //关键点的个数,为0--一个关键点都灭有，1--有一个关键点
	int Key_Point_Ptr;          //下一个关键点
    float Current_Angle_Back;   //返回的转向角
    float Turn_Back_InitAngle;  //掉头之前的车身角度
    float Gear_Cmd;             //档位控制信号
	float Vel_Cmd;              //速度控制信号
    float Angle_Vel_Cmd;        //转动速度控制信号
    float Bark_Cmd;             //抱闸控制信号
    float Angle_Cmd;            //转角控制信号
    double IPI;                 //每一度对应的弧度
    double PI;                  //PI

    AGV();
    AGV(int ,int ,int ,int ,int ,float);        //构造函数
    ~AGV();                                     //析构函数
    void Set_Current_Angle_GPS(float);          //设置当前GPS返回的角度
    void Set_Current_XY_GPS(float, float);      //设置当前GPS返回的坐标
	void Set_Current_Angle_IMU(float);          //设置当前IMU返回的角度
	void Set_Current_XY(float,float);           //设置当前的坐标
	int Build_Path(void);                      //生成路径
	int find_Aim_Point(void);                   //搜寻目标前
	double Get_Angle_Cmd(void);                 //得到转角控制信息
	void Set_Key_Point(float [10][2],int);      //设置路径上的关键点
	int Find_min(float Point[],int);            //寻找数组中最小的数
	float Get_Path_Terminal_Dis(void);          //得到离路径终点的距离
	void Set_Key_Point_Angle(void);             //由关键点计算出关键点需要转的角度
	float Get_Line_Angle(float [2],float [2]);  //得到直线相对Y轴的角度
	int Turnback();                             //掉头
	void Set_Turn_Back_InitAngle(float);        //设置掉头前的车身角度
    int Change_Key_Point(float [2],int);        //修改关键点、修改第n个点、返回当前关键点个数
    int Add_Key_Point(float [2]);               //添加关键点、默认在末尾添加、返回当前关键点个数
    int Delete_Key_Point(void);                 //删除关键点、默认删除末尾点、返回当前关键点个数
    int Delete_Key_Point(int);                  //删除关键点、删除第n个关键点、返回当前关键点个数
    int Clear_Key_Point(void);                  //清除关键点、删除所有关键点、返回当前关键点个数
    int Get_Key_Point_Num(void);                //得到关键点个数
    void Set_Key_Point_Ptr(int);                //得到下一个目标关键点
    void Show_Path_Point(void);                 //输出路径点
    bool Judge_To_Path_end();                   //判断前视点是否到达当前路径点的终点
};

AGV::AGV()                          //默认构造函数
{
    Foresee_Dir = 2.5;              //前视距离，单位为m
    Vel_Max = 1.0;                  //最大速度1.0m/s
    WheelBase = 850;                //轴距850mm；
    Rad_Min = 3.0;                  //转弯半径
    Unit_Dir = 0.05;                //细分段的长度，用于计算直线段部分的路径点个数
    Unit_Angle = 1;                 //圆弧段细分度数，用于计算圆弧段部分的路径点个数
    Set_Key_Point_Ptr(1);           //初始时、下一个目标点为第一个关键点
    Clear_Key_Point();              //清楚关键点
    IPI = 0.0174532925199433333333; //没一度对应的弧度
    PI = 3.1415926535;              //PI
}
AGV::AGV(int _Wheel_Dia,int _WheelBase,int _tread,int _Vel_Max,int _Rad_Min,float _Foresee_Dir)
{
    Key_Point_Ptr = 1;              //下一个关键点为第0个关键点
    Clear_Key_Point();
    Wheel_Dia = _Wheel_Dia;         //轮子直径
    WheelBase = _WheelBase;         //轴距
    tread = _tread;                 //轮距
    Vel_Max = _Vel_Max;             //最大速度
    Rad_Min = _Rad_Min;             //最小转弯半径
    Foresee_Dir = _Foresee_Dir;      //前视距离
    IPI = 0.0174532925199433333333; //没一度对应的弧度
    PI = 3.1415926535;              //PI
}
AGV::~AGV()
{
    Path_Point_count = 0;
    delete[] Path_Point_Ptr;
    std::cout<<"已释放内存"<<std::endl;
}
void AGV::Set_Current_Angle_GPS(float _Current_Angle_GPS)
{
    Current_Angle_GPS = _Current_Angle_GPS;
}

void AGV::Set_Current_Angle_IMU(float _Current_Angle_IMU)
{
    Current_Angle_IMU = _Current_Angle_IMU;
}

void AGV::Set_Current_XY_GPS(float _Current_X_GPS,float _Current_Y_GPS)
{
    Current_X_GPS = _Current_X_GPS;
    Current_Y_GPS = _Current_Y_GPS;
}

void AGV::Set_Current_XY(float _Current_X,float _Current_Y)
{
    Current_X = _Current_X;
    Current_Y = _Current_Y;
}

int AGV::Get_Key_Point_Num(void)
{
    return Key_Point_Num;
}

//脱离当前局部路径、离局部路径终点过近则重新生成路径
int AGV::Build_Path()  //根据当前位置、两个关键点、路径长度生成局部路径
{
    delete[] Path_Point_Ptr;  //删除之前的路径
    Path_Point_count = 0;
    int count_point;        //点的个数，包括首尾
    if((0<Key_Point[Key_Point_Ptr][2]&&Key_Point[Key_Point_Ptr][2]<30)||(Key_Point[Key_Point_Ptr][2]<0&&Key_Point[Key_Point_Ptr][2]>-30))//转弯角度小于30°
    {
        float differ_X = Key_Point[Key_Point_Ptr][0]-Key_Point[Key_Point_Ptr-1][0];
        float differ_Y = Key_Point[Key_Point_Ptr][1]-Key_Point[Key_Point_Ptr-1][1];
        float Line_dir = sqrt(pow(differ_X,2)+pow(differ_Y,2)); //计算距离
        count_point = Line_dir/Unit_Dir;    //计算点的个数
        float Unit_X = differ_X/count_point;
        float Unit_Y = differ_Y/count_point;
        Path_Point_Ptr = new float [count_point+1][2];
        for(int n=0;n<count_point;n++)
        {
            Path_Point_Ptr[n][0] = Key_Point[Key_Point_Ptr-1][0] + n*Unit_X;
            Path_Point_Ptr[n][1] = Key_Point[Key_Point_Ptr-1][1] + n*Unit_Y;
            std::cout<<n<<'\t'<<Path_Point_Ptr[n][0]<<'\t'<<Path_Point_Ptr[n][1]<<std::endl;
        }
        Path_Point_Ptr[count_point][0] = Key_Point[Key_Point_Ptr][0];
        Path_Point_Ptr[count_point][1] = Key_Point[Key_Point_Ptr][1];
        std::cout<<count_point<<'\t'<<Path_Point_Ptr[count_point][0]<<'\t'<<Path_Point_Ptr[count_point][1]<<std::endl;
        count_point++;
    }else//转向角大于30°
    {
        float Line_Cut = Rad_Min*tan(fabs(Key_Point[Key_Point_Ptr][2]*IPI)/2);
        float differ_X = Key_Point[Key_Point_Ptr][0]-Key_Point[Key_Point_Ptr-1][0];
        float differ_Y = Key_Point[Key_Point_Ptr][1]-Key_Point[Key_Point_Ptr-1][1];
        float Line_dir = sqrt(pow(differ_X,2)+pow(differ_Y,2)); //计算距离
        int count_point_1 = (Line_dir - Line_Cut)/Unit_Dir;
        int count_point_2 = fabs(Key_Point[Key_Point_Ptr][2])/Unit_Angle;
        float Unit_X = differ_X*(1-Line_Cut/Line_dir)/count_point_1;
        float Unit_Y = differ_Y*(1-Line_Cut/Line_dir)/count_point_1;
        float Unit_Cir_Dis = Rad_Min*sin(Unit_Angle*IPI)/sin((180-Unit_Angle)/2*IPI);
        float Angle_ = atan2(differ_Y,differ_X);                //当前直线与X轴的夹角-PI--PI
        Path_Point_Ptr = new float [count_point_1+count_point_2+2][2];
        for(int n = 0; n<count_point_1;n++)
        {
            Path_Point_Ptr[n][0] = Key_Point[Key_Point_Ptr-1][0] + n*Unit_X;
            Path_Point_Ptr[n][1] = Key_Point[Key_Point_Ptr-1][1] + n*Unit_Y;
            std::cout<<n<<'\t'<<Path_Point_Ptr[n][0]<<'\t'<<Path_Point_Ptr[n][1]<<std::endl;
        }
        Path_Point_Ptr[count_point_1][0] = Key_Point[Key_Point_Ptr-1][0] + differ_X*(1-Line_Cut/Line_dir);
        Path_Point_Ptr[count_point_1][1] = Key_Point[Key_Point_Ptr-1][1] + differ_Y*(1-Line_Cut/Line_dir);
        std::cout<<count_point_1<<'\t'<<Path_Point_Ptr[count_point_1][0]<<'\t'<<Path_Point_Ptr[count_point_1][1]<<std::endl;
        for(int n = 0; n<count_point_2;n++)
        {
            float Angle__ = 0;
            if(Key_Point[Key_Point_Ptr][2]>0)Angle__ = Angle_ + (n*Unit_Angle + Unit_Angle/2)*IPI;               //转向的向量角
            else Angle__ = Angle_ - (n*Unit_Angle + Unit_Angle/2)*IPI;
            Path_Point_Ptr[count_point_1+n+1][0] = Path_Point_Ptr[count_point_1+n][0] + Unit_Cir_Dis*cos(Angle__);
            Path_Point_Ptr[count_point_1+n+1][1] = Path_Point_Ptr[count_point_1+n][1] + Unit_Cir_Dis*sin(Angle__);
            std::cout<<count_point_1+n+1<<'\t'<<Path_Point_Ptr[count_point_1+n+1][0]<<'\t'<<Path_Point_Ptr[count_point_1+n+1][1]<<std::endl;
        }

        count_point = count_point_1 + count_point_2 + 1;
    }
    Path_Point_count = count_point;
    return count_point;
}

void AGV::Set_Key_Point_Angle()                 //由关键点计算出关键点处需要转的角度-180--180
{
    float lineA_Angle = 0;
    float LineB_Angle = 0;
    if(Key_Point_Num>2)
    {
        for(int i = 0; i < Key_Point_Num-2; i++)
        {
            lineA_Angle = Get_Line_Angle(Key_Point[i],Key_Point[i+1]);
            LineB_Angle = Get_Line_Angle(Key_Point[i+1],Key_Point[i+2]);
            Key_Point[i+1][2] = (LineB_Angle - lineA_Angle)/IPI;
            if(Key_Point[i+1][2]>180)Key_Point[i+1][2] = Key_Point[i+1][2] - 360;
        }
    }else
        {
            std::cout<<"关键点个数小于3"<<std::endl;
        }
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"关键点"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }
}

float AGV::Get_Line_Angle(float Sta[2],float End[2])   //得到直线相对Y轴的角度-PI--PI
{
    float a;
    a = atan2(End[1]-Sta[1],End[0]-Sta[0]) - PI/2;
    if(a < - PI) a = PI*2 + a;
    return a;
}

int AGV::find_Aim_Point()
{
    float _x[Path_Point_count],_y[Path_Point_count],dir[Path_Point_count];
	int ptr = 0;
	float L = 0;
	for(int i=0;i<Path_Point_count;i++)
	{
		_x[i] = Path_Point_Ptr[i][0] - Current_X;
		_y[i] = Path_Point_Ptr[i][1] - Current_Y;
		dir[i] = sqrt(pow(_x[i],2)+pow(_y[i],2));
	}
	ptr = AGV::Find_min(dir,Path_Point_count);
	L = sqrt(pow((Current_X - Path_Point_Ptr[ptr][0]),2)+pow((Current_Y - Path_Point_Ptr[ptr][1]),2));
	while(L<Foresee_Dir&&ptr+1<Path_Point_count)
	{
		float x = Path_Point_Ptr[ptr+1][0] - Path_Point_Ptr[ptr][0];
		float y = Path_Point_Ptr[ptr+1][1] - Path_Point_Ptr[ptr][1];
		L = L+sqrt(pow(x,2)+pow(y,2));
		ptr++;
	}
    Aim_Point_X = Path_Point_Ptr[ptr][0];
    Aim_Point_Y = Path_Point_Ptr[ptr][1];
    std::cout<<"当前AGV目标点坐标："<<'\t'<<Aim_Point_X<<'\t'<<Aim_Point_Y<<std::endl;
    Aim_Point_Ptr = ptr;
    return ptr;
}

double AGV::Get_Angle_Cmd()
{
    double Angle;       //atan2(y2-y1,x2-x1) 计算向量去Y轴正向夹角（弧度-PI-PI）
    double a;
    double ld;          //当前AGV后轮中点与目标点之间的距离
    a = atan2(Aim_Point_Y-Current_Y,Aim_Point_X-Current_X) - PI/2;
    if(a < - PI) a = PI*2 + a;
    Angle = a - Current_Angle*IPI;
    ld = sqrt(pow((Aim_Point_Y-Current_Y)*1000,2) + pow((Aim_Point_X-Current_X)*1000,2));
    Angle_Vel_Cmd = atan(WheelBase*sin(Angle)*2.0/ld)/IPI;
    return Angle_Vel_Cmd;
}

void AGV::Set_Key_Point(float _Key_Point[20][2],int Num)
{
    for(int i=0;i<Num;i++)
    {
        Key_Point[i][0] = _Key_Point[i][0];
        Key_Point[i][1] = _Key_Point[i][1];
    }
}

int AGV::Find_min(float Point[],int _num)
{
    int pos = 0;                    	//最小位置
	double min_ = Point[0];             //最小值
	if( _num > 1 )
    {
	   for(int i=1; i<_num; i++)
        {
	       if( Point[i] < min_ )
            {
	                min_ = Point[i];
	                pos = i;
            }
        }
    }
    return pos;
}

float AGV::Get_Path_Terminal_Dis()
{
    Path_Terminal_Dis = sqrt(pow((Current_X - Path_Point_Ptr[Path_Point_count][0]),2)+pow((Current_Y - Path_Point_Ptr[Path_Point_count][1]),2));
    return Path_Terminal_Dis;
}

void AGV::Set_Turn_Back_InitAngle(float _Turn_Back_InitAngle)
{
    Turn_Back_InitAngle = _Turn_Back_InitAngle;
}

int AGV::Turnback() //掉头,返回0--还未掉头完成，返回1--掉头完成
{

    return 0;
}

int AGV::Change_Key_Point(float _Key_Point[2],int Num)           //修改第n个点、如果为最后一个点后一个点，则添加一个点，返回当前关键点个数
{
    if(Num > Key_Point_Num)
    {
        std::cout<<"关键点个数小于插入点"<<std::endl;
    }else if(Num == Key_Point_Num)
    {
        Key_Point[Key_Point_Num][0] = _Key_Point[0];
        Key_Point[Key_Point_Num][1] = _Key_Point[1];
        Key_Point[Key_Point_Num][2] = 0;
        Key_Point_Num++;
        std::cout<<"已修改关键点"<<Num<<':'<<std::endl;
    }else
    {
        Key_Point[Key_Point_Num][0] = _Key_Point[0];
        Key_Point[Key_Point_Num][1] = _Key_Point[1];
        Key_Point[Key_Point_Num][2] = 0;
        std::cout<<"已修改(末尾添加)关键点"<<Num<<':'<<std::endl;
    }
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"关键点"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }

    return Key_Point_Num;
}
int AGV::Add_Key_Point(float _Key_Point[2])               //添加关键点、默认在末尾添加、返回当前关键点个数
{
    Key_Point[Key_Point_Num][0] = _Key_Point[0];
    Key_Point[Key_Point_Num][1] = _Key_Point[1];
    Key_Point[Key_Point_Num][2] = 0;
    Key_Point_Num++;

    std::cout<<"已在末尾添加关键点"<<':'<<std::endl;
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"关键点"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }
    return Key_Point_Num;
}
int AGV::Delete_Key_Point(void)                     //删除关键点、默认删除末尾点、返回当前关键点个数
{
    if(Key_Point_Num>0)
    {
        Key_Point[Key_Point_Num-1][0] = 0;
        Key_Point[Key_Point_Num-1][1] = 0;
        Key_Point[Key_Point_Num-1][2] = 0;
        Key_Point_Num--;
        std::cout<<"在末尾删除一个关键点"<<':'<<std::endl;
    }else
    {
        std::cout<<"关键点个数已为0"<<std::endl;
    }
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"关键点"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }
        return Key_Point_Num;
}
int AGV::Delete_Key_Point(int Num)                  //删除关键点、删除第n个关键点、返回当前关键点个数
{
    if(Num>(Key_Point_Num-1))
    {
        std::cout<<"关键点个数小于Num"<<std::endl;
    }else if(Num == (Key_Point_Num-1))
    {
        Key_Point[Key_Point_Num-1][0] = 0;
        Key_Point[Key_Point_Num-1][1] = 0;
        Key_Point[Key_Point_Num-1][2] = 0;
        Key_Point_Num--;
        std::cout<<"在末尾删除一个关键点"<<':'<<std::endl;
    }else
    {
        for(int i = 0;i<(Key_Point_Num-1-Num);i++)
        {
            Key_Point[Num+i][0] = Key_Point[Num+i+1][0];
            Key_Point[Num+i][1] = Key_Point[Num+i+1][1];
            Key_Point[Num+i][2] = 0;
        }
        Key_Point_Num--;
        std::cout<<"删除关键点"<<':'<<Num<<std::endl;
    }
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"关键点"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }
    return Key_Point_Num;

}
int AGV::Clear_Key_Point()                      //清除关键点、删除所有关键点、返回当前关键点个数
{
    int i = 0;
    for(i = 0; i<Key_Point_Num;i++)
    {
            Key_Point[i][0] = 0;
            Key_Point[i][1] = 0;
            Key_Point[i][2] = 0;
    }
    Key_Point_Num = 0;
    std::cout<<"已清除所有关键点"<<std::endl;
    return Key_Point_Num;
}

void AGV::Set_Key_Point_Ptr(int n)               //设置下一个目标关键点
{
    Key_Point_Ptr = n;
}

void AGV::Show_Path_Point(void)                 //输出路径点
{
    for(int n=0; n < Path_Point_count; n++)
    {
        std::cout<<n<<":\t"<<Path_Point_Ptr[n][0]<<'\t'<<Path_Point_Ptr[n][1]<<std::endl;
    }
}

bool AGV::Judge_To_Path_end()                   //判断前视点是否到达当前路径点的终点,1--已经到达，2--还未到达
{
    if(Aim_Point_Ptr>=Path_Point_count-1)
        return 1;
    else
        return 0;
}

using namespace std;
int main()
{
    AGV My_AGV;
    float a[2] = {0,0};
    float b[2] = {0,10};
    float c[2] = {0,20};
    float d[2] = {20,11};
    My_AGV.Add_Key_Point(a);
    My_AGV.Add_Key_Point(b);
    My_AGV.Add_Key_Point(c);
    My_AGV.Add_Key_Point(d);
    My_AGV.Set_Key_Point_Angle();
    My_AGV.Set_Key_Point_Ptr(1);
    My_AGV.Build_Path();
    My_AGV.Set_Current_XY(0,7);
    cout<<My_AGV.find_Aim_Point()<<endl;
    cout<<My_AGV.Judge_To_Path_end()<<endl;
    GPS My_GPS;
    My_GPS.Set_Current_LatLon(30.609924556944445584,114.35322082530555576);
    My_GPS.Transport_GPS_Guass();
    return 0;
}
