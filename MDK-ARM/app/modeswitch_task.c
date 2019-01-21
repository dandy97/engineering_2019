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
	osDelay(1500);//延时1.5S后再开启云台和底盘任务定时
	
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
		osDelayUntil(&mode_wake_time, INFO_GET_PERIOD); //退出任务，5ms后再进来
	}
}

void get_main_ctrl_mode(void)
{
	static uint32_t last_Reset = 0;
	
	if((RC_CtrlData.rc.s2 == 1) || (RC_CtrlData.rc.s2 == 2))
	{
		ctrl_mode = REMOTE_CTRL;
	}
	else if(RC_CtrlData.rc.s2 == 3)
	{	
		ctrl_mode = KEYBOR_CTRL;
	}
	
	//每1S判断Reset与last_Reset 如果相等，说明没有收到遥控信号
	//Reset在遥控处理函数里面
	if(mode_switch_times % 400 == 0)
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
	if((mode_switch_times %200 == 0)  && (mode_switch_times %400 != 0))
	{
		last_Reset = Reset;
	}
}

//LED灯翻转
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
