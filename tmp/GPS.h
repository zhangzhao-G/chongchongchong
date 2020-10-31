#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <stdlib.h>
using namespace std;
class GPS
{
public:
    double Lat;                     //GPS返回-纬度
    double Lon;                     //GPS返回-经度
    double Angle;                   //GPS返回-姿态角
    double Gauss_X;                 //高斯投影-X
    double Gauss_Y;                 //高斯投影-Y
    double Local_X;                 //本地坐标-X
    double Local_Y;                 //本地坐标-Y
    double angle;                   //本地坐标与Y轴夹角，-180--180
    int Stable_Ptr;                 //是否收到固定消息
    //创建局部坐标系
    double Origin_Lat;              //原点-纬度
    double Origin_Lon;              //原点-经度
    double Origin_X;                //原点-高斯-X
    double Origin_Y;                //原点-高斯-Y
    double Terminal_Lat;            //Y轴上的控制点-纬度
    double Terminal_Lon;            //Y轴上的控制点-经度
    double Terminal_X;              //Y轴上的控制点-高斯-X
    double Terminal_Y;              //Y轴上的控制点-高斯-Y
    double Transport_Angle;         //高斯坐标--局部坐标旋转角度(弧度)
    double Transport_Angle_du;      //高斯坐标--局部坐标旋转角度(角度)
    //高斯投影参数
    double PI;                      //PI
    double IPI;                     //IPI
    double e1_2;                    //椭球第一偏心率的平方
    double e2_2;                    //椭球第二偏心率的平方
    double L_0;                     //中央经线度数
    double Rad_Earth;               //地球椭球半径
    int prjno;                      //投影带号
    int zonewide;                   //投影带宽度

    string GPS_Rec;
    string GPS_Start_Lat_Lon;
    string GPS_Start_Angle;
    string GPS_Data_End;

    GPS();                                                  //默认构造函数
    void Transport_parameter();                             //计算转换参数
    void Transport_GPS_Guass();                             //高斯投影转换,转换GPS返回的点
    void Transport_GPS_Guass(double, double, double &, double &);
    void Transport_Guass_Local();                           //将高斯坐标XY转换到局部坐标
    void Transport_Guass_Local_Angle();                     //将高斯坐标Angle转换到局部坐标
    void Transport_GPS_Local();
    void Transport_String_GPS(const string &);              //从字符串中解析出经纬度

    void Set_Current_LatLon(double, double);                //设置当前经纬度
    void Set_Current_Angle(double);                         //设置当前高斯姿态角

    double Get_Local_X();                                   //得到当前局部坐标X
    double Get_Local_Y();                                   //得到当前局部坐标Y
    double Get_angle();                                     //得到当前局部姿态角
    double Get_Lat();
    double Get_Lon();
    double Get_GAUSS_X();
    double Get_GAUSS_Y();
    double Get_GAUSS_Angle();
    void Get_Local_XYa(double &, double &, double &);       //得到当前局部坐标XY
};
//默认构造
GPS::GPS()
{
    GPS_Start_Lat_Lon = "$GNGGA";
    GPS_Start_Angle = "$PTNL";
    GPS_Data_End = "\r\n";
    Origin_Lat = 30.610013426694443694;
    Origin_Lon = 114.35320271044444951;
    Terminal_Lat = 30.609924556944445584;
    Terminal_Lon = 114.35322082530555576;
    Rad_Earth = 6378137.0;                  //地球椭球半径
    zonewide = 3;                           //投影带宽度
    PI = 3.1415926535;                      //PI
    IPI = 0.0174532925199433333333;         //IPI
    e1_2 = 0.006694379901333;               //椭球第一偏心率的平方
    e2_2 = 0.00673949674427;                //椭球第二偏心率的平方
    cout<<setprecision(10)<<"原点GPS坐标"<<'\t'<<"Lat:"<<Origin_Lat<<'\t'<<"Lon:"<<Origin_Lon<<endl;
    cout<<"控制点GPS坐标"<<'\t'<<"Lat:"<<Terminal_Lat<<'\t'<<"Lon:"<<Terminal_Lon<<endl;
    Transport_parameter();
}
void GPS::Transport_parameter()           //计算转换参数
{
    //高斯投影转换参数
    if (zonewide == 6){
        prjno = (int) (Origin_Lon / zonewide) + 1;
        L_0 = prjno * zonewide - 3;
    }else{
            prjno = (int) ((Origin_Lon - 1.5) / 3) + 1;
            L_0 = prjno * 3;
        }
    Transport_GPS_Guass(Origin_Lat,Origin_Lon,Origin_X,Origin_Y);
    cout<<"原点Gauss坐标"<<'\t'<<"X:"<<Origin_X<<'\t'<<"Y:"<<Origin_Y<<endl;
    Transport_GPS_Guass(Terminal_Lat,Terminal_Lon,Terminal_X,Terminal_Y);
    cout<<"控制点Gauss坐标"<<'\t'<<"X:"<<Terminal_X<<'\t'<<"Y:"<<Terminal_Y<<endl;
    cout<<"高斯投影带为："<<prjno<<'\t'<<"中央经线为："<<L_0<<endl;
    //原点经纬度--高斯坐标系下
    double Dir_X = Terminal_X - Origin_X;
    double Dir_Y = Terminal_Y - Origin_Y;      //得到转换后Y轴的方向向量
	Transport_Angle = acos((Dir_Y)/sqrt(pow(Dir_X,2)+pow(Dir_Y,2)));
    if(Dir_X>0)Transport_Angle=2*PI-Transport_Angle;
    Transport_Angle_du = Transport_Angle/0.0174532925199433333333;
    cout<<"局部坐标旋转角度(顺时针)："<<Transport_Angle<<'\t'<<Transport_Angle_du<<endl;
}

void GPS::Transport_GPS_Guass()                  //高斯投影转换,转换GPS返回的点
{
    double B = Lat;
    double L = Lon;
    double L0 = L_0;                        //中央经线度数
    double N;                               //卯酉圈曲率半径
    double S;   //从赤道到纬度位B的平行圈子午线的弧度长
    double n;
    double l;
    double X1,X2,X3,Y1,Y2,Y3;

    L0  = L0*IPI;                   // 转为弧度
    L   = L*IPI;                    // 转为弧度
    B   = B*IPI;                    // 转为弧度

    N = Rad_Earth / sqrt(1 - e1_2*pow(sin(B),2));
    S = (double)111132.9525*B/IPI - (double)16038.4201*sin(2*B) + (double)16.8325*sin(4*B) - (double)0.022*sin(6*B) + (double)0.00003*sin(8*B);
    l = (L - L0);                   //将秒转换为弧度
    n = e2_2*pow(cos(B),2);         //辅助变量

    X1 = N*sin(B)*cos(B)*pow(l,2)/2;
    X2 = N*sin(B)*pow(cos(B),3)*(5-pow(tan(B),2) + 9*n + 4*pow(e2_2,2))*pow(l,4)/24;
    X3 = N*sin(B)*pow(cos(B),5)*(61-58*pow(tan(B),2)+pow(tan(B),4))*pow(l,6)/720;
    Y1 = N*cos(B)*l;
    Y2 = N*pow(cos(B),3)*(1-pow(tan(B),2)+n)*pow(l,3)/6;
    Y3 = N*pow(cos(B),5)*(5-18*pow(tan(B),2)+pow(tan(B),4)+14*n-58*pow(tan(B),2)*n)*pow(l,5)/120;
    Gauss_X  = S + X1 + X2 + X3;
    Gauss_Y  = Y1 + Y2 + Y3 + 500000;
}
void GPS::Transport_GPS_Guass(double _Lat, double _Lon, double &_Gauss_X, double &_Gauss_Y)
{
    double B = _Lat;
    double L = _Lon;
    double L0 = L_0;                        //中央经线度数
    double N;                               //卯酉圈曲率半径
    double S;   //从赤道到纬度位B的平行圈子午线的弧度长
    double n;
    double l;
    double X1,X2,X3,Y1,Y2,Y3;

    L0  = L0*IPI;                   // 转为弧度
    L   = L*IPI;                    // 转为弧度
    B   = B*IPI;                    // 转为弧度

    N = Rad_Earth / sqrt(1 - e1_2*pow(sin(B),2));
    S = (double)111132.9525*B/IPI - (double)16038.4201*sin(2*B) + (double)16.8325*sin(4*B) - (double)0.022*sin(6*B) + (double)0.00003*sin(8*B);
    l = (L - L0);                   //将秒转换为弧度
    n = e2_2*pow(cos(B),2);         //辅助变量

    X1 = N*sin(B)*cos(B)*pow(l,2)/2;
    X2 = N*sin(B)*pow(cos(B),3)*(5-pow(tan(B),2) + 9*n + 4*pow(e2_2,2))*pow(l,4)/24;
    X3 = N*sin(B)*pow(cos(B),5)*(61-58*pow(tan(B),2)+pow(tan(B),4))*pow(l,6)/720;
    Y1 = N*cos(B)*l;
    Y2 = N*pow(cos(B),3)*(1-pow(tan(B),2)+n)*pow(l,3)/6;
    Y3 = N*pow(cos(B),5)*(5-18*pow(tan(B),2)+pow(tan(B),4)+14*n-58*pow(tan(B),2)*n)*pow(l,5)/120;
    _Gauss_X  = S + X1 + X2 + X3;
    _Gauss_Y  = Y1 + Y2 + Y3 + 500000;
}

void GPS::Transport_Guass_Local()                           //将高斯坐标转换到局部坐标
{
    double Differ_X = (Gauss_X - Origin_X);
    double Differ_Y = (Gauss_Y - Origin_Y);
    Local_X = -(Differ_X*cos(Transport_Angle) + Differ_Y*sin(Transport_Angle));
    Local_Y = -Differ_X*sin(Transport_Angle) + Differ_Y*cos(Transport_Angle);
}

void GPS::Transport_Guass_Local_Angle()                     //将高斯坐标Angle转换到局部坐标
{
    angle = 90 + Angle-Transport_Angle_du;
    if(angle > 180)angle = angle - 360;
}

void GPS::Transport_GPS_Local()
{
    Transport_GPS_Guass();
    Transport_Guass_Local();
}

void GPS::Transport_String_GPS(const string &rec)                      //从字符串中解析出经纬度
{
    vector<string> GPS_vector;		//定义容器
	string::size_type pos_1,pos_2;
	pos_2 = rec.find(",");						//搜索“，”
	pos_1 = 0;
	while( std::string::npos !=pos_2)			//pos_2不等于size_type的最大值时
	{
		GPS_vector.push_back(rec.substr(pos_1,pos_2-pos_1));	//pos_到pos_2中的内容放入容器
		pos_1 = pos_2 + 1;								//pos_1移位到pos_2后一位
		pos_2 = rec.find(",",pos_1);							//pos_2从pos_1开始再次搜索“，”
	}
	if( pos_1 != rec.length())		//如果pos_1不等于字符串长度
	{
		GPS_vector.push_back(rec.substr(pos_1));	//将pos_1后面的放入容器中
	}

	if("$GNGGA"==GPS_Start_Lat_Lon)
    {
        if("$GNGGA" == GPS_vector[0])	//检验GPS数据
        {
            if(4.0 == atof(GPS_vector[6].c_str())){
                Lat = (double)atof(GPS_vector[2].c_str())/100;		//解析出纬度
                int ilat = (int)floor(Lat);
                Lat = ilat + (Lat - ilat)*100/60;
                Lon = (double)atof(GPS_vector[4].c_str())/100;		//解析出经度
                int ilon = (int)floor(Lon);
                Lon = ilon + (Lon - ilon)*100/60;
                Stable_Ptr ++;
                if(Stable_Ptr >= 3)Stable_Ptr = 3;
            }else Stable_Ptr = 0;
        }else if(GPS_vector[1] == "AVR")
            {
                if(3 == Stable_Ptr)angle = atof(GPS_vector[3].c_str());
            }
    }
}

void GPS::Set_Current_LatLon(double _Lat, double _Lon)       //设置当前经纬度
{
    Lat = _Lat;
    Lon = _Lon;
}
void GPS::Set_Current_Angle(double _Angle)                  //设置当前高斯姿态角
{
    Angle = _Angle;
}


double GPS::Get_Local_X()                                   //得到当前局部坐标X
{
    return Local_X;
}

double GPS::Get_Local_Y()                                   //得到当前局部坐标Y
{
    return Local_Y;
}

double GPS::Get_angle()                                     //得到当前局部姿态角
{
    return angle;
}

double GPS::Get_Lat()
{
	return Lat;
}
double GPS::Get_Lon()
{
	return Lon;
}
double GPS::Get_GAUSS_X()
{
	return Gauss_X;
}

double GPS::Get_GAUSS_Y()
{
	return Gauss_Y;
}
double GPS::Get_GAUSS_Angle()
{
	return Angle;
}

void GPS::Get_Local_XYa(double &_X, double &_Y, double &_a) //得到当前局部坐标XY
{
    _X = Local_X;
    _Y = Local_Y;
    _a = angle;
}
