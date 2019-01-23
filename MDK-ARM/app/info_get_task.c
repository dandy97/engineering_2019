#include "main.h"

UBaseType_t info_stack_surplus;
uint32_t info_time_last;
int info_time_ms;
extern uint8_t ka_dan[10];

void info_get_task(void const *argument)
{
	osEvent event;
	for(;;)
	{
		event = osSignalWait(GIMBAL_INFO_GET_SIGNAL |\
												 INFO_GET_SIGNAL,osWaitForever);
		
		if(event.status == osEventSignal)
		{
			if(event.value.signals & GIMBAL_INFO_GET_SIGNAL)
			{
				taskENTER_CRITICAL();
				
				get_gimbal_info();
				
				taskEXIT_CRITICAL();
			}
			if(event.value.signals & INFO_GET_SIGNAL)
			{
				info_time_ms = HAL_GetTick() - info_time_last;
				info_time_last = HAL_GetTick();
				
				taskENTER_CRITICAL();
				
				get_chassis_info();
				
				taskEXIT_CRITICAL();
				
				//Ni_Ming(0xf1,Robot_angle_ref, gyro_data.yaw, 0, 0);
			}
		}
		
		info_stack_surplus =uxTaskGetStackHighWaterMark (NULL);
	}
}
