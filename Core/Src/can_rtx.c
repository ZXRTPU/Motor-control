#include  "can_rtx.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern motor_info_t  motor;
uint16_t can_cnt_1=0;

//============================CAN���Ϳ��Ƶ�������ݣ�������==========================================
void set_motor_current_can2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x1ff):(0x2ff);//���id_range==0�����0x200,id_range==1�����0x1ff��ID�ţ�
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
	
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (v1>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

//=================================CAN�߽��ܵ����Ϣ===============================================
extern int count;
extern double k;
	
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//�����жϻص�����
{
  CAN_RxHeaderTypeDef rx_header;

  if(hcan->Instance == CAN1)
  {
     uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
		if(rx_header.StdId==0x55)//��C����C��IMU����
		{	
				
		} 


  }
	
	//�����Ϣ����
	 if(hcan->Instance == CAN2)
  {		
		uint8_t             rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can2 data
		
			if (rx_header.StdId ==0x207)
		 { 		
									// get motor index by can_id
			 motor.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
			
			//uint8_t index = rx_header.StdId - 0x201;
			 motor.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
			
			 motor.torque_current = ((rx_data[4] << 8) | rx_data[5]);
			 motor.temp           =   rx_data[6];
		
		 }
		 
		 count++;
	   if(count==1)
		 {
			 k=motor.rotor_angle;
		 }
	}

}

/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//�����жϻص�����
{
  CAN_RxHeaderTypeDef rx_header;

  if(hcan->Instance == CAN1)
  {
     uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
		if(rx_header.StdId==0x55)//��C����C��IMU����
		{	
				
		} 


  }
	
	//�����Ϣ����
	 if(hcan->Instance == CAN2)
  {		
		uint8_t             rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can2 data
		if ((rx_header.StdId >= 0x205)//205-208
   && (rx_header.StdId <  0x208))                  // �жϱ�ʶ������ʶ��Ϊ0x205+ID
  {
    uint8_t index = rx_header.StdId - 0x205;  
		
                // get motor index by can_id
     motor[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
		
		//uint8_t index = rx_header.StdId - 0x201;
     motor[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
		
     motor[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
     motor[index].temp           =   rx_data[6];
		
		
		if(index==0)
		{can_cnt_1 ++;}
  }
	
	if(rx_header.StdId==0x211)
			{
				
				extern float powerdata[4];
				uint16_t *pPowerdata = (uint16_t *)rx_data;

				powerdata[0] = (float)pPowerdata[0]/100.f;//�����ѹ
				powerdata[1] = (float)pPowerdata[1]/100.f;//���ݵ�ѹ
				powerdata[2] =(float)pPowerdata[2]/100.f;//�������
				powerdata[3] = (float)pPowerdata[3]/100.f;//P
				
				
			
  }}

}*/









