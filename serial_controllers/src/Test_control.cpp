/*
 *
 *
 *
 */

#include "ros/ros.h"
#include "serial_controllers/STM32_control.h"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Imu.h"
#include "serial_controllers/Pulse.h"
#include <geometry_msgs/Point.h>
#include <math.h>


class AGV{
public:
    int Wheel_Dia;      //轮子直径
    int WheelBase;      //轴距
    int tread;          //轮距
    int Vel_Max;        //最大速度
    int Rad_Min;        //最小转弯半径
	double Current_Angle_GPS;	//当前角度
	double Current_Angle_IMU;	//当前角度
	double Current_Vel;			//当前速度
	double Current_X;			//当前X
	double Current_Y;			//当前Y
	double Aim_point_X;			//目标点X
	double Aim_Point_Y;			//目标点Y

	void Build_Path();
	void Get_Aim();
	void Get_Angle();
	void Set_Current_Angle_GPS();
	void Set_Current_Angle_IMU();
	void Set_Current_X();
	void Set_Current_Y();
}
