#include "main.h"
#include "math.h"

/* stack usage monitor */
UBaseType_t gimbal_stack_surplus;

/* gimbal task global parameter */
gimbal_t gim;

uint32_t gimbal_time_last;
int gimbal_time_ms;
uint32_t handler_run_time = 0;
extern ramp_t pit_ramp;
extern ramp_t yaw_ramp;
extern TaskHandle_t can_msg_seng_task_t;
extern TaskHandle_t shoot_task_t;
extern TaskHandle_t info_get_task_t;
float Robot_angle_ref;
//static float pidin, ppidp, ppidi, ppidd, spidp, spidi, spidd = 0;
void gimbal_task(void const * argument)
{
	handler_run_time++;
	if(gim.ctrl_mode == GIMBAL_INIT)
	{
		Robot_angle_ref = gyro_data.yaw;
	}
	
	if(handler_run_time > 1000)
	{
		gim.ctrl_mode = GIMBAL_NORMAL;
	}
	
	gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	osSignalSet(info_get_task_t, GIMBAL_INFO_GET_SIGNAL);
}

void gimbal_param_init(void)
{
	memset(&gim, 0, sizeof(gimbal_t));
	
	gim.ctrl_mode 		 = GIMBAL_INIT;
	gim.last_ctrl_mode = GIMBAL_INIT;
}
