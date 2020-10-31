#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <stdlib.h>
using namespace std;
class GPS
{
public:
    double Lat;                     //GPS����-γ��
    double Lon;                     //GPS����-����
    double Angle;                   //GPS����-��̬��
    double Gauss_X;                 //��˹ͶӰ-X
    double Gauss_Y;                 //��˹ͶӰ-Y
    double Local_X;                 //��������-X
    double Local_Y;                 //��������-Y
    double angle;                   //����������Y��нǣ�-180--180
    int Stable_Ptr;                 //�Ƿ��յ��̶���Ϣ
    //�����ֲ�����ϵ
    double Origin_Lat;              //ԭ��-γ��
    double Origin_Lon;              //ԭ��-����
    double Origin_X;                //ԭ��-��˹-X
    double Origin_Y;                //ԭ��-��˹-Y
    double Terminal_Lat;            //Y���ϵĿ��Ƶ�-γ��
    double Terminal_Lon;            //Y���ϵĿ��Ƶ�-����
    double Terminal_X;              //Y���ϵĿ��Ƶ�-��˹-X
    double Terminal_Y;              //Y���ϵĿ��Ƶ�-��˹-Y
    double Transport_Angle;         //��˹����--�ֲ�������ת�Ƕ�(����)
    double Transport_Angle_du;      //��˹����--�ֲ�������ת�Ƕ�(�Ƕ�)
    //��˹ͶӰ����
    double PI;                      //PI
    double IPI;                     //IPI
    double e1_2;                    //�����һƫ���ʵ�ƽ��
    double e2_2;                    //����ڶ�ƫ���ʵ�ƽ��
    double L_0;                     //���뾭�߶���
    double Rad_Earth;               //��������뾶
    int prjno;                      //ͶӰ����
    int zonewide;                   //ͶӰ�����

    string GPS_Rec;
    string GPS_Start_Lat_Lon;
    string GPS_Start_Angle;
    string GPS_Data_End;

    GPS();                                                  //Ĭ�Ϲ��캯��
    void Transport_parameter();                             //����ת������
    void Transport_GPS_Guass();                             //��˹ͶӰת��,ת��GPS���صĵ�
    void Transport_GPS_Guass(double, double, double &, double &);
    void Transport_Guass_Local();                           //����˹����XYת�����ֲ�����
    void Transport_Guass_Local_Angle();                     //����˹����Angleת�����ֲ�����
    void Transport_GPS_Local();
    void Transport_String_GPS(const string &);              //���ַ����н�������γ��

    void Set_Current_LatLon(double, double);                //���õ�ǰ��γ��
    void Set_Current_Angle(double);                         //���õ�ǰ��˹��̬��

    double Get_Local_X();                                   //�õ���ǰ�ֲ�����X
    double Get_Local_Y();                                   //�õ���ǰ�ֲ�����Y
    double Get_angle();                                     //�õ���ǰ�ֲ���̬��
    double Get_Lat();
    double Get_Lon();
    double Get_GAUSS_X();
    double Get_GAUSS_Y();
    double Get_GAUSS_Angle();
    void Get_Local_XYa(double &, double &, double &);       //�õ���ǰ�ֲ�����XY
};
//Ĭ�Ϲ���
GPS::GPS()
{
    GPS_Start_Lat_Lon = "$GNGGA";
    GPS_Start_Angle = "$PTNL";
    GPS_Data_End = "\r\n";
    Origin_Lat = 30.610013426694443694;
    Origin_Lon = 114.35320271044444951;
    Terminal_Lat = 30.609924556944445584;
    Terminal_Lon = 114.35322082530555576;
    Rad_Earth = 6378137.0;                  //��������뾶
    zonewide = 3;                           //ͶӰ�����
    PI = 3.1415926535;                      //PI
    IPI = 0.0174532925199433333333;         //IPI
    e1_2 = 0.006694379901333;               //�����һƫ���ʵ�ƽ��
    e2_2 = 0.00673949674427;                //����ڶ�ƫ���ʵ�ƽ��
    cout<<setprecision(10)<<"ԭ��GPS����"<<'\t'<<"Lat:"<<Origin_Lat<<'\t'<<"Lon:"<<Origin_Lon<<endl;
    cout<<"���Ƶ�GPS����"<<'\t'<<"Lat:"<<Terminal_Lat<<'\t'<<"Lon:"<<Terminal_Lon<<endl;
    Transport_parameter();
}
void GPS::Transport_parameter()           //����ת������
{
    //��˹ͶӰת������
    if (zonewide == 6){
        prjno = (int) (Origin_Lon / zonewide) + 1;
        L_0 = prjno * zonewide - 3;
    }else{
            prjno = (int) ((Origin_Lon - 1.5) / 3) + 1;
            L_0 = prjno * 3;
        }
    Transport_GPS_Guass(Origin_Lat,Origin_Lon,Origin_X,Origin_Y);
    cout<<"ԭ��Gauss����"<<'\t'<<"X:"<<Origin_X<<'\t'<<"Y:"<<Origin_Y<<endl;
    Transport_GPS_Guass(Terminal_Lat,Terminal_Lon,Terminal_X,Terminal_Y);
    cout<<"���Ƶ�Gauss����"<<'\t'<<"X:"<<Terminal_X<<'\t'<<"Y:"<<Terminal_Y<<endl;
    cout<<"��˹ͶӰ��Ϊ��"<<prjno<<'\t'<<"���뾭��Ϊ��"<<L_0<<endl;
    //ԭ�㾭γ��--��˹����ϵ��
    double Dir_X = Terminal_X - Origin_X;
    double Dir_Y = Terminal_Y - Origin_Y;      //�õ�ת����Y��ķ�������
	Transport_Angle = acos((Dir_Y)/sqrt(pow(Dir_X,2)+pow(Dir_Y,2)));
    if(Dir_X>0)Transport_Angle=2*PI-Transport_Angle;
    Transport_Angle_du = Transport_Angle/0.0174532925199433333333;
    cout<<"�ֲ�������ת�Ƕ�(˳ʱ��)��"<<Transport_Angle<<'\t'<<Transport_Angle_du<<endl;
}

void GPS::Transport_GPS_Guass()                  //��˹ͶӰת��,ת��GPS���صĵ�
{
    double B = Lat;
    double L = Lon;
    double L0 = L_0;                        //���뾭�߶���
    double N;                               //î��Ȧ���ʰ뾶
    double S;   //�ӳ����γ��λB��ƽ��Ȧ�����ߵĻ��ȳ�
    double n;
    double l;
    double X1,X2,X3,Y1,Y2,Y3;

    L0  = L0*IPI;                   // תΪ����
    L   = L*IPI;                    // תΪ����
    B   = B*IPI;                    // תΪ����

    N = Rad_Earth / sqrt(1 - e1_2*pow(sin(B),2));
    S = (double)111132.9525*B/IPI - (double)16038.4201*sin(2*B) + (double)16.8325*sin(4*B) - (double)0.022*sin(6*B) + (double)0.00003*sin(8*B);
    l = (L - L0);                   //����ת��Ϊ����
    n = e2_2*pow(cos(B),2);         //��������

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
    double L0 = L_0;                        //���뾭�߶���
    double N;                               //î��Ȧ���ʰ뾶
    double S;   //�ӳ����γ��λB��ƽ��Ȧ�����ߵĻ��ȳ�
    double n;
    double l;
    double X1,X2,X3,Y1,Y2,Y3;

    L0  = L0*IPI;                   // תΪ����
    L   = L*IPI;                    // תΪ����
    B   = B*IPI;                    // תΪ����

    N = Rad_Earth / sqrt(1 - e1_2*pow(sin(B),2));
    S = (double)111132.9525*B/IPI - (double)16038.4201*sin(2*B) + (double)16.8325*sin(4*B) - (double)0.022*sin(6*B) + (double)0.00003*sin(8*B);
    l = (L - L0);                   //����ת��Ϊ����
    n = e2_2*pow(cos(B),2);         //��������

    X1 = N*sin(B)*cos(B)*pow(l,2)/2;
    X2 = N*sin(B)*pow(cos(B),3)*(5-pow(tan(B),2) + 9*n + 4*pow(e2_2,2))*pow(l,4)/24;
    X3 = N*sin(B)*pow(cos(B),5)*(61-58*pow(tan(B),2)+pow(tan(B),4))*pow(l,6)/720;
    Y1 = N*cos(B)*l;
    Y2 = N*pow(cos(B),3)*(1-pow(tan(B),2)+n)*pow(l,3)/6;
    Y3 = N*pow(cos(B),5)*(5-18*pow(tan(B),2)+pow(tan(B),4)+14*n-58*pow(tan(B),2)*n)*pow(l,5)/120;
    _Gauss_X  = S + X1 + X2 + X3;
    _Gauss_Y  = Y1 + Y2 + Y3 + 500000;
}

void GPS::Transport_Guass_Local()                           //����˹����ת�����ֲ�����
{
    double Differ_X = (Gauss_X - Origin_X);
    double Differ_Y = (Gauss_Y - Origin_Y);
    Local_X = -(Differ_X*cos(Transport_Angle) + Differ_Y*sin(Transport_Angle));
    Local_Y = -Differ_X*sin(Transport_Angle) + Differ_Y*cos(Transport_Angle);
}

void GPS::Transport_Guass_Local_Angle()                     //����˹����Angleת�����ֲ�����
{
    angle = 90 + Angle-Transport_Angle_du;
    if(angle > 180)angle = angle - 360;
}

void GPS::Transport_GPS_Local()
{
    Transport_GPS_Guass();
    Transport_Guass_Local();
}

void GPS::Transport_String_GPS(const string &rec)                      //���ַ����н�������γ��
{
    vector<string> GPS_vector;		//��������
	string::size_type pos_1,pos_2;
	pos_2 = rec.find(",");						//����������
	pos_1 = 0;
	while( std::string::npos !=pos_2)			//pos_2������size_type�����ֵʱ
	{
		GPS_vector.push_back(rec.substr(pos_1,pos_2-pos_1));	//pos_��pos_2�е����ݷ�������
		pos_1 = pos_2 + 1;								//pos_1��λ��pos_2��һλ
		pos_2 = rec.find(",",pos_1);							//pos_2��pos_1��ʼ�ٴ�����������
	}
	if( pos_1 != rec.length())		//���pos_1�������ַ�������
	{
		GPS_vector.push_back(rec.substr(pos_1));	//��pos_1����ķ���������
	}

	if("$GNGGA"==GPS_Start_Lat_Lon)
    {
        if("$GNGGA" == GPS_vector[0])	//����GPS����
        {
            if(4.0 == atof(GPS_vector[6].c_str())){
                Lat = (double)atof(GPS_vector[2].c_str())/100;		//������γ��
                int ilat = (int)floor(Lat);
                Lat = ilat + (Lat - ilat)*100/60;
                Lon = (double)atof(GPS_vector[4].c_str())/100;		//����������
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

void GPS::Set_Current_LatLon(double _Lat, double _Lon)       //���õ�ǰ��γ��
{
    Lat = _Lat;
    Lon = _Lon;
}
void GPS::Set_Current_Angle(double _Angle)                  //���õ�ǰ��˹��̬��
{
    Angle = _Angle;
}


double GPS::Get_Local_X()                                   //�õ���ǰ�ֲ�����X
{
    return Local_X;
}

double GPS::Get_Local_Y()                                   //�õ���ǰ�ֲ�����Y
{
    return Local_Y;
}

double GPS::Get_angle()                                     //�õ���ǰ�ֲ���̬��
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

void GPS::Get_Local_XYa(double &_X, double &_Y, double &_a) //�õ���ǰ�ֲ�����XY
{
    _X = Local_X;
    _Y = Local_Y;
    _a = angle;
}
