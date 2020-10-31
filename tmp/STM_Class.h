#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <stdlib.h>

class STM
{
public:
    char Send_Msg[13];                  //发送给STM32的命令
    int Gear;                           //档位
    int Park;                           //驻车
    int Odom;                           //里程计
    float Vel;                          //速度
    float Angle;                        //转角
    float Angle_Vel;                    //转动角速度
    float Brak;                         //刹车
    float Vel_Max;                      //最大速度



};
