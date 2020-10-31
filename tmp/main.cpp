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
    int Wheel_Dia;              //����ֱ��
    int WheelBase;              //���
    int tread;                  //�־�
    int Vel_Max;                //����ٶ�
    int Rad_Min;                //��Сת��뾶
    float Foresee_Dir;          //ǰ�Ӿ���
	float Current_Angle_GPS;	//��ǰ����Ƕ�
	float Current_X_GPS;        //GPS���ص�����X
	float Current_Y_GPS;        //GPS���ص�����Y
	float Current_Angle_IMU;	//��ǰ����Ƕ�
	float Current_Angle;        //��ǰ����Ƕ�
	float Current_Vel;			//��ǰ�ٶ�
	float Current_X;			//��ǰX
	float Current_Y;			//��ǰY
    float Unit_Dir;             //��λ����
    float Unit_Angle;           //��λ�Ƕ�
	float Aim_Point_X;		    //Ŀ���X
	float Aim_Point_Y;		    //Ŀ���Y
	int Aim_Point_Ptr;          //Ŀ�����·�����еĵڼ�����
	float Terminal_X;           //�յ�X
	float Terminal_Y;           //�յ�Y
	float Path_Terminal_Dis;    //��·���յ�ľ���
	float (*Path_Point_Ptr)[2] = NULL;//������ָ��,����NEWһ��·��
	int Path_Point_count;       //·����ĸ���,��1��ʼ
	float Key_Point[20][4];     //�ؼ���20�����ؼ����������ڹؼ�����Ҫת�ĽǶ�
    int Key_Point_Num;          //�ؼ���ĸ���,Ϊ0--һ���ؼ��㶼���У�1--��һ���ؼ���
	int Key_Point_Ptr;          //��һ���ؼ���
    float Current_Angle_Back;   //���ص�ת���
    float Turn_Back_InitAngle;  //��ͷ֮ǰ�ĳ���Ƕ�
    float Gear_Cmd;             //��λ�����ź�
	float Vel_Cmd;              //�ٶȿ����ź�
    float Angle_Vel_Cmd;        //ת���ٶȿ����ź�
    float Bark_Cmd;             //��բ�����ź�
    float Angle_Cmd;            //ת�ǿ����ź�
    double IPI;                 //ÿһ�ȶ�Ӧ�Ļ���
    double PI;                  //PI

    AGV();
    AGV(int ,int ,int ,int ,int ,float);        //���캯��
    ~AGV();                                     //��������
    void Set_Current_Angle_GPS(float);          //���õ�ǰGPS���صĽǶ�
    void Set_Current_XY_GPS(float, float);      //���õ�ǰGPS���ص�����
	void Set_Current_Angle_IMU(float);          //���õ�ǰIMU���صĽǶ�
	void Set_Current_XY(float,float);           //���õ�ǰ������
	int Build_Path(void);                      //����·��
	int find_Aim_Point(void);                   //��ѰĿ��ǰ
	double Get_Angle_Cmd(void);                 //�õ�ת�ǿ�����Ϣ
	void Set_Key_Point(float [10][2],int);      //����·���ϵĹؼ���
	int Find_min(float Point[],int);            //Ѱ����������С����
	float Get_Path_Terminal_Dis(void);          //�õ���·���յ�ľ���
	void Set_Key_Point_Angle(void);             //�ɹؼ��������ؼ�����Ҫת�ĽǶ�
	float Get_Line_Angle(float [2],float [2]);  //�õ�ֱ�����Y��ĽǶ�
	int Turnback();                             //��ͷ
	void Set_Turn_Back_InitAngle(float);        //���õ�ͷǰ�ĳ���Ƕ�
    int Change_Key_Point(float [2],int);        //�޸Ĺؼ��㡢�޸ĵ�n���㡢���ص�ǰ�ؼ������
    int Add_Key_Point(float [2]);               //��ӹؼ��㡢Ĭ����ĩβ��ӡ����ص�ǰ�ؼ������
    int Delete_Key_Point(void);                 //ɾ���ؼ��㡢Ĭ��ɾ��ĩβ�㡢���ص�ǰ�ؼ������
    int Delete_Key_Point(int);                  //ɾ���ؼ��㡢ɾ����n���ؼ��㡢���ص�ǰ�ؼ������
    int Clear_Key_Point(void);                  //����ؼ��㡢ɾ�����йؼ��㡢���ص�ǰ�ؼ������
    int Get_Key_Point_Num(void);                //�õ��ؼ������
    void Set_Key_Point_Ptr(int);                //�õ���һ��Ŀ��ؼ���
    void Show_Path_Point(void);                 //���·����
    bool Judge_To_Path_end();                   //�ж�ǰ�ӵ��Ƿ񵽴ﵱǰ·������յ�
};

AGV::AGV()                          //Ĭ�Ϲ��캯��
{
    Foresee_Dir = 2.5;              //ǰ�Ӿ��룬��λΪm
    Vel_Max = 1.0;                  //����ٶ�1.0m/s
    WheelBase = 850;                //���850mm��
    Rad_Min = 3.0;                  //ת��뾶
    Unit_Dir = 0.05;                //ϸ�ֶεĳ��ȣ����ڼ���ֱ�߶β��ֵ�·�������
    Unit_Angle = 1;                 //Բ����ϸ�ֶ��������ڼ���Բ���β��ֵ�·�������
    Set_Key_Point_Ptr(1);           //��ʼʱ����һ��Ŀ���Ϊ��һ���ؼ���
    Clear_Key_Point();              //����ؼ���
    IPI = 0.0174532925199433333333; //ûһ�ȶ�Ӧ�Ļ���
    PI = 3.1415926535;              //PI
}
AGV::AGV(int _Wheel_Dia,int _WheelBase,int _tread,int _Vel_Max,int _Rad_Min,float _Foresee_Dir)
{
    Key_Point_Ptr = 1;              //��һ���ؼ���Ϊ��0���ؼ���
    Clear_Key_Point();
    Wheel_Dia = _Wheel_Dia;         //����ֱ��
    WheelBase = _WheelBase;         //���
    tread = _tread;                 //�־�
    Vel_Max = _Vel_Max;             //����ٶ�
    Rad_Min = _Rad_Min;             //��Сת��뾶
    Foresee_Dir = _Foresee_Dir;      //ǰ�Ӿ���
    IPI = 0.0174532925199433333333; //ûһ�ȶ�Ӧ�Ļ���
    PI = 3.1415926535;              //PI
}
AGV::~AGV()
{
    Path_Point_count = 0;
    delete[] Path_Point_Ptr;
    std::cout<<"���ͷ��ڴ�"<<std::endl;
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

//���뵱ǰ�ֲ�·������ֲ�·���յ��������������·��
int AGV::Build_Path()  //���ݵ�ǰλ�á������ؼ��㡢·���������ɾֲ�·��
{
    delete[] Path_Point_Ptr;  //ɾ��֮ǰ��·��
    Path_Point_count = 0;
    int count_point;        //��ĸ�����������β
    if((0<Key_Point[Key_Point_Ptr][2]&&Key_Point[Key_Point_Ptr][2]<30)||(Key_Point[Key_Point_Ptr][2]<0&&Key_Point[Key_Point_Ptr][2]>-30))//ת��Ƕ�С��30��
    {
        float differ_X = Key_Point[Key_Point_Ptr][0]-Key_Point[Key_Point_Ptr-1][0];
        float differ_Y = Key_Point[Key_Point_Ptr][1]-Key_Point[Key_Point_Ptr-1][1];
        float Line_dir = sqrt(pow(differ_X,2)+pow(differ_Y,2)); //�������
        count_point = Line_dir/Unit_Dir;    //�����ĸ���
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
    }else//ת��Ǵ���30��
    {
        float Line_Cut = Rad_Min*tan(fabs(Key_Point[Key_Point_Ptr][2]*IPI)/2);
        float differ_X = Key_Point[Key_Point_Ptr][0]-Key_Point[Key_Point_Ptr-1][0];
        float differ_Y = Key_Point[Key_Point_Ptr][1]-Key_Point[Key_Point_Ptr-1][1];
        float Line_dir = sqrt(pow(differ_X,2)+pow(differ_Y,2)); //�������
        int count_point_1 = (Line_dir - Line_Cut)/Unit_Dir;
        int count_point_2 = fabs(Key_Point[Key_Point_Ptr][2])/Unit_Angle;
        float Unit_X = differ_X*(1-Line_Cut/Line_dir)/count_point_1;
        float Unit_Y = differ_Y*(1-Line_Cut/Line_dir)/count_point_1;
        float Unit_Cir_Dis = Rad_Min*sin(Unit_Angle*IPI)/sin((180-Unit_Angle)/2*IPI);
        float Angle_ = atan2(differ_Y,differ_X);                //��ǰֱ����X��ļн�-PI--PI
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
            if(Key_Point[Key_Point_Ptr][2]>0)Angle__ = Angle_ + (n*Unit_Angle + Unit_Angle/2)*IPI;               //ת���������
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

void AGV::Set_Key_Point_Angle()                 //�ɹؼ��������ؼ��㴦��Ҫת�ĽǶ�-180--180
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
            std::cout<<"�ؼ������С��3"<<std::endl;
        }
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"�ؼ���"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }
}

float AGV::Get_Line_Angle(float Sta[2],float End[2])   //�õ�ֱ�����Y��ĽǶ�-PI--PI
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
    std::cout<<"��ǰAGVĿ������꣺"<<'\t'<<Aim_Point_X<<'\t'<<Aim_Point_Y<<std::endl;
    Aim_Point_Ptr = ptr;
    return ptr;
}

double AGV::Get_Angle_Cmd()
{
    double Angle;       //atan2(y2-y1,x2-x1) ��������ȥY������нǣ�����-PI-PI��
    double a;
    double ld;          //��ǰAGV�����е���Ŀ���֮��ľ���
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
    int pos = 0;                    	//��Сλ��
	double min_ = Point[0];             //��Сֵ
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

int AGV::Turnback() //��ͷ,����0--��δ��ͷ��ɣ�����1--��ͷ���
{

    return 0;
}

int AGV::Change_Key_Point(float _Key_Point[2],int Num)           //�޸ĵ�n���㡢���Ϊ���һ�����һ���㣬�����һ���㣬���ص�ǰ�ؼ������
{
    if(Num > Key_Point_Num)
    {
        std::cout<<"�ؼ������С�ڲ����"<<std::endl;
    }else if(Num == Key_Point_Num)
    {
        Key_Point[Key_Point_Num][0] = _Key_Point[0];
        Key_Point[Key_Point_Num][1] = _Key_Point[1];
        Key_Point[Key_Point_Num][2] = 0;
        Key_Point_Num++;
        std::cout<<"���޸Ĺؼ���"<<Num<<':'<<std::endl;
    }else
    {
        Key_Point[Key_Point_Num][0] = _Key_Point[0];
        Key_Point[Key_Point_Num][1] = _Key_Point[1];
        Key_Point[Key_Point_Num][2] = 0;
        std::cout<<"���޸�(ĩβ���)�ؼ���"<<Num<<':'<<std::endl;
    }
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"�ؼ���"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }

    return Key_Point_Num;
}
int AGV::Add_Key_Point(float _Key_Point[2])               //��ӹؼ��㡢Ĭ����ĩβ��ӡ����ص�ǰ�ؼ������
{
    Key_Point[Key_Point_Num][0] = _Key_Point[0];
    Key_Point[Key_Point_Num][1] = _Key_Point[1];
    Key_Point[Key_Point_Num][2] = 0;
    Key_Point_Num++;

    std::cout<<"����ĩβ��ӹؼ���"<<':'<<std::endl;
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"�ؼ���"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }
    return Key_Point_Num;
}
int AGV::Delete_Key_Point(void)                     //ɾ���ؼ��㡢Ĭ��ɾ��ĩβ�㡢���ص�ǰ�ؼ������
{
    if(Key_Point_Num>0)
    {
        Key_Point[Key_Point_Num-1][0] = 0;
        Key_Point[Key_Point_Num-1][1] = 0;
        Key_Point[Key_Point_Num-1][2] = 0;
        Key_Point_Num--;
        std::cout<<"��ĩβɾ��һ���ؼ���"<<':'<<std::endl;
    }else
    {
        std::cout<<"�ؼ��������Ϊ0"<<std::endl;
    }
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"�ؼ���"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }
        return Key_Point_Num;
}
int AGV::Delete_Key_Point(int Num)                  //ɾ���ؼ��㡢ɾ����n���ؼ��㡢���ص�ǰ�ؼ������
{
    if(Num>(Key_Point_Num-1))
    {
        std::cout<<"�ؼ������С��Num"<<std::endl;
    }else if(Num == (Key_Point_Num-1))
    {
        Key_Point[Key_Point_Num-1][0] = 0;
        Key_Point[Key_Point_Num-1][1] = 0;
        Key_Point[Key_Point_Num-1][2] = 0;
        Key_Point_Num--;
        std::cout<<"��ĩβɾ��һ���ؼ���"<<':'<<std::endl;
    }else
    {
        for(int i = 0;i<(Key_Point_Num-1-Num);i++)
        {
            Key_Point[Num+i][0] = Key_Point[Num+i+1][0];
            Key_Point[Num+i][1] = Key_Point[Num+i+1][1];
            Key_Point[Num+i][2] = 0;
        }
        Key_Point_Num--;
        std::cout<<"ɾ���ؼ���"<<':'<<Num<<std::endl;
    }
    for(int i = 0;i<Key_Point_Num;i++)
    {
        std::cout<<"�ؼ���"<<i<<':'<<'\t'<<Key_Point[i][0]<<'\t'<<Key_Point[i][1]<<'\t'<<Key_Point[i][2]<<std::endl;
    }
    return Key_Point_Num;

}
int AGV::Clear_Key_Point()                      //����ؼ��㡢ɾ�����йؼ��㡢���ص�ǰ�ؼ������
{
    int i = 0;
    for(i = 0; i<Key_Point_Num;i++)
    {
            Key_Point[i][0] = 0;
            Key_Point[i][1] = 0;
            Key_Point[i][2] = 0;
    }
    Key_Point_Num = 0;
    std::cout<<"��������йؼ���"<<std::endl;
    return Key_Point_Num;
}

void AGV::Set_Key_Point_Ptr(int n)               //������һ��Ŀ��ؼ���
{
    Key_Point_Ptr = n;
}

void AGV::Show_Path_Point(void)                 //���·����
{
    for(int n=0; n < Path_Point_count; n++)
    {
        std::cout<<n<<":\t"<<Path_Point_Ptr[n][0]<<'\t'<<Path_Point_Ptr[n][1]<<std::endl;
    }
}

bool AGV::Judge_To_Path_end()                   //�ж�ǰ�ӵ��Ƿ񵽴ﵱǰ·������յ�,1--�Ѿ����2--��δ����
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
