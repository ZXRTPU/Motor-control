#include "chassis.h"
#include "can_rtx.h"
#include "pid.h"

#define PI 3.1415926

motor_info_t  motor;//拨盘电机结构体

//设置电机的目标速度和目标角度
int16_t speed_target=1000;
int16_t angle_target=30;

//映射电机目前的角度值
int16_t now_encoder_angle=0;
int16_t now_angle=0;

//电机角度环和速度环的PID
pid_struct_t  motor_pid_speed;
pid_struct_t  motor_pid_angle;

fp32 pid_speed[3]={3,0.5,0};
fp32 pid_angle[3]={20,0,0};


//====================================控制GM6020的代码=====================================

//===========================yaw轴部分控制函数
/* --------------------------功能层------------------------- */
/**
 * @brief 对YAW电机角度进行处理
 * @note  把 机械模式、陀螺仪底盘跟随、小陀螺回正 所用到的yaw电机角度
          都进行处理
 */
/*
float RUD_MotorAngle_Proc(Rudder_Axis_Info_t *str)
{
  int16_t Angle = str->motor_data.CAN_GetData.Motor_Angle;
  
  if(Angle <= str->MECH_Mid_Angle - Motor_180)
    Angle += Motor_360;
  if(Angle >  str->MECH_Mid_Angle + Motor_180)
    Angle -= Motor_360;   

  return (float)Angle;
}
//用于在融合XYZ速度时取xy前进角和Z旋转角的误差
float RUD_DirAngleErr_Proc(float xy,float z)
{
  if(z <= xy - Motor_180)
    z += Motor_360;
  if(z >  xy + Motor_180)
    z -= Motor_360;
  
  return (z - xy);
}
*/

/*将atan得出来的结果由0~360转化为*/
float RUD_Z_atanAngle_Proc()
{
	float Angle;
	Angle=motor.rotor_angle;
  if(Angle > 4096)Angle = Angle - 8192;
  return Angle;
}

//将角度值转化为0~8192之间的值
float RUD_DirAngle_Proc()
{
	float Angle;
	Angle=motor.rotor_angle;
  while (Angle > 8192 || Angle < 0)
  {
    if(Angle < 0)
      Angle += 8192;
    if(Angle > 8192)
      Angle -= 8192; 
  }
  return (float)Angle;
}
//在0~8192之间计算pid误差时，要取就近误差，在PID_GetAngleErr中调用
void RUD_PIDAngleTarget_Proc()
{
  float Target_Angle =(angle_target/360)/8192;
    
  if(Target_Angle <= motor.rotor_angle -4096 - 40)//非半圆划分,为了在180度转的时候同向
    Target_Angle += 8192;
  if(Target_Angle >  motor.rotor_angle + 4096- 40)//非半圆划分
    Target_Angle -= 8192;   
}


//===============映射函数，将绝对编码器的值（0~8191）转换为弧度制
double msp(double x, double in_min, double in_max, double out_min, double out_max)//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

//===============整定GM6020(绝对式编码器)的函数
double k=0; //设置的电机编码器正前方位置
//double n; //编码器的值
int count=0;

uint16_t encoder_zero(uint16_t n)
{
	if(n>k+4096)
	{
		n=n-8192;
	}
	else if((k<=n)&&(n<=4096))
	{
		n=n-k;
	}
	else if(n<(k-4096))
	{
		n=n+8192;
	}
	else
	{
		n=n-k;
	}
	
	return n;
}

int flag=0;

//===============速度环发送电流
void motor_current_give_speed()
{
		pid_init(&motor_pid_speed,pid_speed,10000,10000); //速度pid初始化
		
		if(motor.rotor_speed>=speed_target) //pid限幅
		{
			motor_pid_speed.Kp=0;
			motor_pid_speed.Ki=0;
			motor_pid_speed.Kd=0;
		}
		
		motor.set_current=pid_calc(&motor_pid_speed,motor.rotor_speed,speed_target); //pid输出值，及CAN线发送给电机的控制值
		                                                                             //将电流pid输出控制值赋给电机控制结构体
	//通过CAN线发送电流
	 set_motor_current_can2(0,motor.set_current,motor.set_current,motor.set_current,motor.set_current);
	
	flag++;
}
	

//===============位置环：速度角度双环PID发送电流
void motor_current_give_angle()
{
	  pid_init(&motor_pid_speed,pid_speed,30000,30000); //速度pid初始化
	  pid_init(&motor_pid_speed,pid_angle,0,360); //角度pid初始化
	
	  //整定编码器目前的反馈值
	  now_encoder_angle=encoder_zero(motor.rotor_angle);
	
    //计算当前的编码器角度值，运用msp函数将编码器的值映射为弧度制
		now_angle=msp(now_encoder_angle,0,8192,0,360);
	
	  angle_target=now_angle+angle_target;
	
	  if((now_angle<=360)&&(angle_target>360))
		{
			angle_target=angle_target-360;
		}
		
		speed_target=pid_calc(&motor_pid_angle,motor.rotor_angle,angle_target);
		motor.set_current=pid_calc(&motor_pid_speed,motor.rotor_speed,speed_target);
		
		set_motor_current_can2(0,motor.set_current,motor.set_current,motor.set_current,motor.set_current);
} 

/*
void angle_control_2(double angle)
{
	target_angle_trigger=motor.rotor_angle-angle/360.0*8191;
	
	if(target_angle_trigger>8191)
	{
		target_angle_trigger-=8191;
	}
	else if(target_angle_trigger<0)
	{
		target_angle_trigger += 8191;
	}
	
	target_omega_trigger=pid_pitch_calc(&motor_pid[2],target_angle_trigger,motor_info[2].rotor_angle);
	
	motor_info[2].set_voltage = pid_calc(&motor_pid[2],target_omega_trigger, motor_info[2].rotor_speed);
}
*/

//===========================最新控制代码====================================
float Now_angle=0.0f;   //由编码器映射出的角度值
float delta_angle=0.0f;
uint16_t target_angle_trigger;
uint16_t target_omega_trigger;


void motor_init()
{
	pid_init(&motor_pid_angle,pid_angle,16384,16384);
	pid_init(&motor_pid_speed,pid_speed,16384,16384);
	
	motor.last_angle=motor.rotor_angle;
}

void err_angle()
{
	delta_angle=motor.rotor_angle-motor.last_angle;
	motor.last_angle=motor.rotor_angle;
	
	if(delta_angle>8191/2)
	{
		delta_angle-=8191;
	}
	else if(delta_angle<-8191/2) 
	{
		delta_angle+= 8191;
	}
	
	delta_angle/=36;
	
	Now_angle+=delta_angle;
	
	if(Now_angle>8191)
	{
		Now_angle-=8191;
	}
	else if(Now_angle<0)
	{
		Now_angle+=8191;
	}
}

void angle_control_2()
{		
	err_angle();
	
	target_omega_trigger=pid_pitch_calc(&motor_pid_angle,target_angle_trigger,Now_angle);
	
	motor.set_current= pid_calc(&motor_pid_speed,target_omega_trigger, motor.rotor_speed);
}


//=========================控制M2006或M3508的代码=============================
/*
void motor_current_give()
{
	for(int i=0;i<4;i++)
	{
		pid_init(&motor_pid[i],pid_chassis,1000,1000);
		
		if(motor[i].rotor_speed>=speed_target)
		{
			motor_pid[i].Kp=0;
			motor_pid[i].Ki=0;
			motor_pid[i].Kd=0;
		}
		
		current=pid_calc(&motor_pid[i],motor[i].rotor_speed,speed_target);
		
		motor[i].set_current=current;
	}
	
	set_motor_current_can2(0,motor[0].set_current,motor[1].set_current,motor[2].set_current,motor[3].set_current);
}
*/
