#ifndef CHASSIS_H
#define CHASSIS_H

#include "can_rtx.h"
#include "stdint.h"
#include "main.h"

typedef struct 
{
    uint16_t can_id;		//ID号
    int16_t  set_current;		//发送信息
    uint16_t rotor_angle;		//现在的角度
	  uint16_t last_angle;    //上一次电机的速度
    int16_t  rotor_speed;		//现在的转速
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
}motor_info_t;

void motor_current_give(void);

void motor_current_give_speed();

void motor_current_give_angle();

//====================最新的控制代码=================
void motor_init();

void err_angle();

void angle_control_2();

#endif