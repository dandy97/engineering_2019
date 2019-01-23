#include "main.h"

/* stack usage monitor */
UBaseType_t mode_stack_surplus;

uint8_t ctrl_mode = 0;
uint32_t mode_switch_times, Reset= 0;
ramp_mode_ee ramp_mode = RAMP_FLAT;
extern osTimerId chassis_timer_id;
extern osTimerId gimbal_timer_id;
extern TaskHandle_t info_get_task_t;
void mode_swtich_task(void const *arugment)
{
	osDelay(1500);//��ʱ1.5S���ٿ�����̨�͵�������ʱ
	
	osTimerStart(chassis_timer_id ,CHASSIS_PERIOD);
	osTimerStart(gimbal_timer_id  ,GIMBAL_PERIOD);
	
	uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		/* uint32_t  is :4294967295 ms
								 is :4294967    s
								 is :71582      min
							   is :1193       h
								 is :49         days       */
		mode_switch_times++;  
		led_flicker();
		taskENTER_CRITICAL();
		
		get_main_ctrl_mode();
		
		taskEXIT_CRITICAL();
		
		osSignalSet(info_get_task_t, INFO_GET_SIGNAL);
		
		mode_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
		osDelayUntil(&mode_wake_time, INFO_GET_PERIOD); //�˳�����5ms���ٽ���
	}
}

void get_main_ctrl_mode(void)
{
	static int32_t ccr_1_q,ccr_2_q,ccr_3_q = 0;
	static uint32_t last_Reset = 0;
	
	if((RC_CtrlData.rc.s2 == 1) || (RC_CtrlData.rc.s2 == 2))
	{
		ctrl_mode = REMOTE_CTRL;
//		TIM4->CCR1 = ccr_1_q;
//		TIM4->CCR2 = ccr_2_q;
//		TIM4->CCR3 = ccr_3_q;
		
	}
	else if(RC_CtrlData.rc.s2 == 3)
	{	
		ctrl_mode = KEYBOR_CTRL;
//		TIM4->CCR3 = 20000;
	}
	
	//ÿ1S�ж�Reset��last_Reset �����ȣ�˵��û���յ�ң���ź�
	//Reset��ң�ش���������
	if(mode_switch_times % 20 == 0)
	{
		if(Reset == last_Reset)
		{
			gim.stop = 1;
		}
		else
		{
			gim.stop = 0;
		}
	}
	if((mode_switch_times %10 == 0)  && (mode_switch_times %20 != 0))
	{
		last_Reset = Reset;
	}
}

//LED�Ʒ�ת
void led_flicker(void)
{
	if((mode_switch_times %100 == 0)  && (mode_switch_times %200 != 0))
	{
		LED1 = 1000;
		LED2 = 0;
	}
	else if(mode_switch_times %200 == 0)
	{
		LED1 = 0;
		LED2 = 800;
	}
}
