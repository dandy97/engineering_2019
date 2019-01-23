#include "main.h"
#include "math.h"

/* stack usage monitor */
UBaseType_t chassis_stack_surplus;

/* chassis task global parameter */
chassis_t chassis = {0};

uint32_t chassis_time_last;
int chassis_time_ms;
extern ramp_t FBSpeedRamp;
extern ramp_t LRSpeedRamp;
extern TaskHandle_t can_msg_seng_task_t;
//static float pidin, ppidp, ppidi, ppidd, spidp, spidi, spidd = 0;

/* chassis task*/
void chassis_task(void const *argument)
{
	chassis_time_ms = HAL_GetTick() - chassis_time_last;
	chassis_time_last = HAL_GetTick();
	
	if(gim.ctrl_mode == GIMBAL_INIT)//chassis dose not follow gimbal when gimbal initializa
	{
		chassis.vw = 0;
	}
	else if(gim.ctrl_mode == GIMBAL_NORMAL)
	{
		chassis.vw = (Robot_angle_ref - gyro_data.yaw) * 2;
	}
	
	chassis.vx = chassis.vx_offset;
	chassis.vy = chassis.vy_offset;
	
	chassis.vx_f =  chassis.vx_f_offset;
	chassis.vy_f =  chassis.vy_f_offset;
	
	if(fabs(chassis.vw) > 45)
	{
		if(chassis.vw > 0)
			chassis.vw = 45;
		else
			chassis.vw = -45;
	}
	
	chassis.wheel_spd_ref[0] = -chassis.vx + chassis.vy - (int16_t)chassis.vw;
	chassis.wheel_spd_ref[1] =  chassis.vx + chassis.vy - (int16_t)chassis.vw;
	chassis.wheel_spd_ref[2] = -chassis.vx - chassis.vy - (int16_t)chassis.vw;
	chassis.wheel_spd_ref[3] =  chassis.vx - chassis.vy - (int16_t)chassis.vw;
	chassis.wheel_spd_ref[4] = -chassis.vx_f + chassis.vy_f + chassis.vw_f;
	chassis.wheel_spd_ref[5] =  chassis.vx_f + chassis.vy_f + chassis.vw_f;
	

	if(gim.stop == 1)//如果失去遥控信后后底盘不动
	{
			for(int i =0; i < 6; i++)
		{
			chassis.wheel_spd_ref[i] = 0;
		}
	}
	

	for(int i =0; i < 6; i++)
	{
		chassis.current[i] = chassis_pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
	}
	
	memcpy(glb_cur.chassis_cur, chassis.current, sizeof(chassis.current));
	osSignalSet(can_msg_seng_task_t, CHASSIS_MOTOR_MSG_SEND);
	
	chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
}

/* chissis Initialization*/
void chassis_param_init(void)
{
	memset(&chassis, 0, sizeof(chassis_t));
	
	chassis.writhe_speed_fac = -1;
	
	/* initializa chassis speed ramp */
	ramp_init(&LRSpeedRamp, MOUSR_LR_RAMP_TICK_COUNT);
	ramp_init(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	
	/* initializa chassis follow gimbal pid */
		PID_struct_init(&pid_rotate, POSITION_PID, 60, 0, 
		1.0, 0, 0.1);//2.0
	
	/* initializa chassis wheels position pid */
//	 for (int k = 0; k < 4; k++)
//  {
//    PID_struct_init(&pid_pos[k], POSITION_PID, 6, 6,
//		0.001, 0, 0); 
//	}
	/* initializa chassis wheels speed pid */
	 for (int k = 0; k < 6; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 8000, 0,
		600, 0, 0); 
	}
}
