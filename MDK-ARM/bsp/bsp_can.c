#include "main.h"

moto_measure_t moto_chassis[6];
shoot_moto_measure_t moto_friction[2];


GYRO_DATA gyro_data;

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	//CAN1����
	if(hcan == (&hcan1))
	{
		switch (hcan1.pRxMsg->StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			{
				static uint8_t i;
				i = hcan1.pRxMsg->StdId - CAN_3508_M1_ID;
				moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], &hcan1) : encoder_data_handler(&moto_chassis[i], &hcan1);
			}
			break;
			default:
			{
			}
			break;
		}
	}
	//CAN2����
	else
	{
//		LED1 = 20000;
		
		switch (hcan2.pRxMsg->StdId)
		{
			case 101:
			{
				zitai_data_receive(&gyro_data);
			}
			break;
			default:
			{
			}
			break;
		}
//	LED1 = 0;
	}
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
  __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6, GPIO_PIN_RESET);
}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)//4096
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	 ptr->last_total_angle = ptr->total_angle;
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
  
#ifdef CHASSIS_3510
  int32_t temp_sum = 0;
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= FILTER_BUF)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
	ptr->last_filter_rate = ptr->filter_rate;
  ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
  ptr->speed_rpm   = (int16_t)(ptr->filter_rate * 7.324f);
#else
  ptr->speed_rpm     = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
  ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);
#endif

}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler1(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
  
#if CAR_NUM == 5
  if (ptr->ecd - ptr->last_ecd > 4096)
 #else
	if (ptr->ecd - ptr->last_ecd > 6500)
#endif
	{
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
	
#if CAR_NUM == 5
  else if (ptr->ecd - ptr->last_ecd < -4096)
 #else
	else if (ptr->ecd - ptr->last_ecd < -6500)
#endif
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	ptr->last_total_angle = ptr->total_angle;
#if CAR_NUM == 5
	ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO + (float)360.0f;
#else
	ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
#endif
  
#ifdef CHASSIS_3510
  int32_t temp_sum = 0;
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= FILTER_BUF)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
	ptr->last_filter_rate = ptr->filter_rate;
  ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
  ptr->speed_rpm   = (int16_t)(ptr->filter_rate * 7.324f);
#else
  ptr->speed_rpm     = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
  ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);
#endif
}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler2(shoot_moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  int32_t temp_sum = 0;
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= 25)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < 25; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
	ptr->last_filter_rate = ptr->filter_rate;
  ptr->filter_rate = (int32_t)(temp_sum/25);

}

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
    ptr->ecd        = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**	
	*					get gyro angle
	*/ 

void zitai_data_receive(GYRO_DATA* gyro)
{
	static int16_t yaw_connt = 0;
	static int16_t pitch_connt = 0;
	gyro->raw_v_x = hcan2.pRxMsg->Data[0]<<8 | hcan2.pRxMsg->Data[1];
	gyro->raw_v_z = hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3];
	gyro->raw_pitch = hcan2.pRxMsg->Data[4]<<8 | hcan2.pRxMsg->Data[5];
	gyro->raw_yaw   = hcan2.pRxMsg->Data[6]<<8 | hcan2.pRxMsg->Data[7];
	
	gyro->v_x = (float)gyro->raw_v_x * 0.057295f;
	gyro->v_z = (float)gyro->raw_v_z * 0.057295f;
	
	gyro->pitch_angle = (float)gyro->raw_pitch/100;
	gyro->yaw_angle  = (float)gyro->raw_yaw/100;
	
	if(gyro->pitch_angle < 0)
	{
		gyro->pitch_angle = gyro->pitch_angle + 360;
	}
	
	if((gyro->pitch_angle - gyro->last_pitch_angle) > 330)
		pitch_connt--;
	else if((gyro->pitch_angle - gyro->last_pitch_angle) < -330)
		pitch_connt++;
	
	gyro->pitch = gyro->pitch_angle + pitch_connt * 360;
	gyro->last_pitch_angle = gyro->pitch_angle;
	
	if((gyro->yaw_angle - gyro->last_yaw_angle) > 330)
		yaw_connt--;
	else if((gyro->yaw_angle - gyro->last_yaw_angle) < -330)
		yaw_connt++;
	
	gyro->last_yaw = gyro->yaw;
	gyro->yaw = gyro->yaw_angle + yaw_connt * 360;
	gyro->last_yaw_angle = gyro->yaw_angle;
}

/**
	*				get Accelerometer acceleration
	*/
void gyro_data_receive(GYRO_DATA* gyro)
{
	gyro->raw_v_x = hcan2.pRxMsg->Data[0]<<8 | hcan2.pRxMsg->Data[1];
	gyro->raw_v_y = hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3];
	gyro->raw_v_z = hcan2.pRxMsg->Data[4]<<8 | hcan2.pRxMsg->Data[5];

	gyro->v_x = (float)gyro->raw_v_x * 0.057295f;
	gyro->v_y = (float)gyro->raw_v_y * 0.057295f;
	gyro->v_z = (float)gyro->raw_v_z * 0.057295f;
}

void can_receive_start(void)
{
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0); 
}

/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
void send_front_chaais_cur(int16_t iq1, int16_t iq2)
{
  hcan1.pTxMsg->StdId   = 0x1ff;
  hcan1.pTxMsg->IDE     = CAN_ID_STD;
  hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan1.pTxMsg->DLC     = 4;
  /* adding minus due to clockwise rotation of the gimbal motor with positive current */
  hcan1.pTxMsg->Data[0] = iq1 >> 8;
  hcan1.pTxMsg->Data[1] = iq1;
  /* adding minus due to clockwise rotation of the gimbal motor with positive current */
  hcan1.pTxMsg->Data[2] = iq2 >> 8;
  hcan1.pTxMsg->Data[3] = iq2;
  hcan1.pTxMsg->Data[4] = 0;
  hcan1.pTxMsg->Data[5] = 0;
  hcan1.pTxMsg->Data[6] = 0;
  hcan1.pTxMsg->Data[7] = 0;
  HAL_CAN_Transmit(&hcan1, 10);
}

/**
  * @brief  send calculated current to motor
  * @param  3510 motor ESC id
  */
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  hcan1.pTxMsg->StdId   = 0x200;
  hcan1.pTxMsg->IDE     = CAN_ID_STD;
  hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan1.pTxMsg->DLC     = 0x08;
  hcan1.pTxMsg->Data[0] = iq1 >> 8;
  hcan1.pTxMsg->Data[1] = iq1;
  hcan1.pTxMsg->Data[2] = iq2 >> 8;
  hcan1.pTxMsg->Data[3] = iq2;
  hcan1.pTxMsg->Data[4] = iq3 >> 8;
  hcan1.pTxMsg->Data[5] = iq3;
  hcan1.pTxMsg->Data[6] = iq4 >> 8;
  hcan1.pTxMsg->Data[7] = iq4;
  HAL_CAN_Transmit(&hcan1, 10);
}

void send_Gyro(uint8_t mode, uint16_t time)
{
	hcan2.pTxMsg->StdId   = 100;
  hcan2.pTxMsg->IDE     = CAN_ID_STD;
  hcan2.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan2.pTxMsg->DLC     = 3;
  hcan2.pTxMsg->Data[0] = mode;
	hcan2.pTxMsg->Data[1] = time >> 8;
  hcan2.pTxMsg->Data[2] = time ;
  HAL_CAN_Transmit(&hcan2, 10);
}

void Send_Friction(uint32_t ID , uint8_t speed)
{
  hcan1.pTxMsg->StdId   = ID;
  hcan1.pTxMsg->IDE     = CAN_ID_STD;
  hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
  hcan1.pTxMsg->DLC     = 0x08;
  hcan1.pTxMsg->Data[0] = speed;
  HAL_CAN_Transmit(&hcan1, 10);
}
