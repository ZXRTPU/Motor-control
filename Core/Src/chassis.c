#include "chassis.h"
#include "can_rtx.h"
#include "pid.h"

#define PI 3.1415926

motor_info_t  motor;//���̵���ṹ��

//���õ����Ŀ���ٶȺ�Ŀ��Ƕ�
int16_t speed_target=500;
int16_t angle_target=30;

//ӳ����Ŀǰ�ĽǶ�ֵ
int16_t now_encoder_angle=0;
int16_t now_angle=0;

//����ǶȻ����ٶȻ���PID
pid_struct_t  motor_pid_speed;
pid_struct_t  motor_pid_angle;

//int16_t current=0;

fp32 pid_speed[3]={30,0.5,15};
fp32 pid_angle[3]={400,0,0};


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

//===============�ٶȻ����͵���
void motor_current_give_speed()
{
		pid_init(&motor_pid_speed,pid_speed,1000,1000); //�ٶ�pid��ʼ��
		
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
}
	

//===============λ�û����ٶȽǶ�˫��PID���͵���
void motor_current_give_angle()
{
	  pid_init(&motor_pid_speed,pid_speed,30000,30000); //�ٶ�pid��ʼ��
	  pid_init(&motor_pid_speed,pid_angle,0,320); //�Ƕ�pid��ʼ��
	
	  //����������Ŀǰ�ķ���ֵ
	  now_encoder_angle=encoder_zero(motor.rotor_angle);
	
    //���㵱ǰ�ı������Ƕ�ֵ������msp��������������ֵӳ��Ϊ������
		now_angle=msp(now_encoder_angle,0,8191,0,360);
	
	  angle_target=now_angle+angle_target;
	
	  if((now_angle<=360)&&(angle_target>360))
		{
			angle_target=angle_target-360;
		}
		
		speed_target=pid_calc(&motor_pid_angle,motor.rotor_angle,angle_target);
		motor.set_current=pid_calc(&motor_pid_speed,motor.rotor_speed,speed_target);
		
		set_motor_current_can2(0,motor.set_current,motor.set_current,motor.set_current,motor.set_current);
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
