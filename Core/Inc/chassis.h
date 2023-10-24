#ifndef CHASSIS_H
#define CHASSIS_H

#include "can_rtx.h"
#include "stdint.h"
#include "main.h"

typedef struct 
{
    uint16_t can_id;		//ID��
    int16_t  set_current;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
	  uint16_t last_angle;    //��һ�ε�����ٶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
}motor_info_t;

void motor_current_give(void);

void motor_current_give_speed();

void motor_current_give_angle();

//====================���µĿ��ƴ���=================
void motor_init();

void err_angle();

void angle_control_2();

#endif