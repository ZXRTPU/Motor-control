#include "chassis.h"
#include "can_rtx.h"
#include "pid.h"

#define PI 3.1415926

motor_info_t  motor;//���̵���ṹ��

//���õ����Ŀ���ٶȺ�Ŀ��Ƕ�
int16_t speed_target=1000;
int16_t angle_target=30;

//ӳ����Ŀǰ�ĽǶ�ֵ
int16_t now_encoder_angle=0;
int16_t now_angle=0;

//����ǶȻ����ٶȻ���PID
pid_struct_t  motor_pid_speed;
pid_struct_t  motor_pid_angle;

fp32 pid_speed[3]={3,0.5,0};
fp32 pid_angle[3]={20,0,0};


//====================================����GM6020�Ĵ���=====================================

//===========================yaw�Ჿ�ֿ��ƺ���
/* --------------------------���ܲ�------------------------- */
/**
 * @brief ��YAW����ǶȽ��д���
 * @note  �� ��еģʽ�������ǵ��̸��桢С���ݻ��� ���õ���yaw����Ƕ�
          �����д���
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
//�������ں�XYZ�ٶ�ʱȡxyǰ���Ǻ�Z��ת�ǵ����
float RUD_DirAngleErr_Proc(float xy,float z)
{
  if(z <= xy - Motor_180)
    z += Motor_360;
  if(z >  xy + Motor_180)
    z -= Motor_360;
  
  return (z - xy);
}
*/

/*��atan�ó����Ľ����0~360ת��Ϊ*/
float RUD_Z_atanAngle_Proc()
{
	float Angle;
	Angle=motor.rotor_angle;
  if(Angle > 4096)Angle = Angle - 8192;
  return Angle;
}

//���Ƕ�ֵת��Ϊ0~8192֮���ֵ
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
//��0~8192֮�����pid���ʱ��Ҫȡ�ͽ�����PID_GetAngleErr�е���
void RUD_PIDAngleTarget_Proc()
{
  float Target_Angle =(angle_target/360)/8192;
    
  if(Target_Angle <= motor.rotor_angle -4096 - 40)//�ǰ�Բ����,Ϊ����180��ת��ʱ��ͬ��
    Target_Angle += 8192;
  if(Target_Angle >  motor.rotor_angle + 4096- 40)//�ǰ�Բ����
    Target_Angle -= 8192;   
}


//===============ӳ�亯���������Ա�������ֵ��0~8191��ת��Ϊ������
double msp(double x, double in_min, double in_max, double out_min, double out_max)//ӳ�亯��������������ֵ��0~8191��ת��Ϊ�����ƵĽǶ�ֵ��-pi~pi��
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

//===============����GM6020(����ʽ������)�ĺ���
double k=0; //���õĵ����������ǰ��λ��
//double n; //��������ֵ
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

//===============�ٶȻ����͵���
void motor_current_give_speed()
{
		pid_init(&motor_pid_speed,pid_speed,10000,10000); //�ٶ�pid��ʼ��
		
		if(motor.rotor_speed>=speed_target) //pid�޷�
		{
			motor_pid_speed.Kp=0;
			motor_pid_speed.Ki=0;
			motor_pid_speed.Kd=0;
		}
		
		motor.set_current=pid_calc(&motor_pid_speed,motor.rotor_speed,speed_target); //pid���ֵ����CAN�߷��͸�����Ŀ���ֵ
		                                                                             //������pid�������ֵ����������ƽṹ��
	//ͨ��CAN�߷��͵���
	 set_motor_current_can2(0,motor.set_current,motor.set_current,motor.set_current,motor.set_current);
	
	flag++;
}
	

//===============λ�û����ٶȽǶ�˫��PID���͵���
void motor_current_give_angle()
{
	  pid_init(&motor_pid_speed,pid_speed,30000,30000); //�ٶ�pid��ʼ��
	  pid_init(&motor_pid_speed,pid_angle,0,360); //�Ƕ�pid��ʼ��
	
	  //����������Ŀǰ�ķ���ֵ
	  now_encoder_angle=encoder_zero(motor.rotor_angle);
	
    //���㵱ǰ�ı������Ƕ�ֵ������msp��������������ֵӳ��Ϊ������
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

//===========================���¿��ƴ���====================================
float Now_angle=0.0f;   //�ɱ�����ӳ����ĽǶ�ֵ
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


//=========================����M2006��M3508�Ĵ���=============================
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
