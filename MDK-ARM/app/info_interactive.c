#include "main.h"
#include "math.h"

ramp_t pit_ramp 	 = RAMP_GEN_DAFAULT;
ramp_t yaw_ramp    = RAMP_GEN_DAFAULT;
ramp_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ramp_t LRSpeedRamp = RAMP_GEN_DAFAULT;

/************************************		CHASSIS INFO    *****************************************/
/*
																				CHASSIS INFO


																																																*/
/*left wheel:2   right wheel:1 */

/*left wheel:4   right wheel:3 */
void get_chassis_info(void)
{
	/* get chassis wheel fdb positin */
  for (uint8_t i = 0; i < 6; i++)
  {
    chassis.wheel_pos_fdb[i] = moto_chassis[i].total_angle/19.0f;
  }
  /* get chassis wheel fdb speed */
  for (uint8_t i = 0; i < 6; i++)
  {
    chassis.wheel_spd_fdb[i] = moto_chassis[i].filter_rate/19.0;
  }
	
	//chassis.power_surplus = 1;
	
	/* get remote and keyboard chassis control information */
  keyboard_chassis_hook();
  remote_ctrl_chassis_hook();
}

void send_chassis_motor_ctrl_message(int16_t chassis_cur[])
{
	 send_chassis_cur(chassis_cur[0] ,chassis_cur[1] ,chassis_cur[2] ,chassis_cur[3]);
}

void send_front_chassis_motor_ctrl_message(int16_t chassis_cur[])
{
	 send_front_chaais_cur(chassis_cur[4] ,chassis_cur[5]);
}

/************************************* CHASSIS INFO ********************************************/




/************************************* GIMBAL  INFO *********************************************/
/*
																			 GIMBAL  INFO


																																																 */
void get_gimbal_info(void)
{
	if(gim.ctrl_mode == GIMBAL_INIT)
	{
	}
	else if((gim.ctrl_mode == GIMBAL_NORMAL) || (gim.ctrl_mode == GIMBAL_WRITHE))
	{
	}		
	//get trig pos info
	
	chassis.follow_gyro = gyro_data.yaw;
	/* get gimbal relative palstance */
  //the Z axis(yaw) of gimbal coordinate system corresponds to the IMU Z axis
  gim.sensor.yaw_palstance = gyro_data.v_z;
  //the Y axis(pitch) of gimbal coordinate system corresponds to the IMU -X axis
  gim.sensor.pit_palstance = gyro_data.v_x;
	
	/* get remote and keyboard gimbal control information */
}

/************************************* GIMBAL  INFO *********************************************/


static uint32_t turn_time_last = 0;
/******************************** CHASSIS REMOTE  HANDLER ***************************************/

void remote_ctrl_chassis_hook(void)
{
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == REMOTE_CTRL))
	{
		/* get chassis wheel ref speed */
		chassis.vx_offset = RC_CtrlData.rc.ch1 * CHASSIS_REF_FACT;
		chassis.vy_offset = RC_CtrlData.rc.ch0 * CHASSIS_REF_FACT;
		
		chassis.vx_f_offset = RC_CtrlData.rc.ch3 * CHASSIS_REF_FACT;
		Robot_angle_ref -= RC_CtrlData.rc.ch2 * CHASSIS_REF_FACT *  0.02;
	}
}

void keyboard_chassis_hook(void)
{
	static float forward_back_speed = 0;
	static float left_right_speed 	 = 0;
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == KEYBOR_CTRL))
	{
		/*********** speed mode: normal speed/high speed ***********/                   

		if(RC_CtrlData.key.v & SHIFT)																	
		{
			forward_back_speed = HIGH_FORWARD_BACK_SPEED;
			left_right_speed 	 = HIGH_LEFT_RIGHT_SPEED;
		}
		else
		{
			forward_back_speed = HIGH_FORWARD_BACK_SPEED;
			left_right_speed   = HIGH_LEFT_RIGHT_SPEED;
		}
		
		/*********** get forward chassis wheel ref speed ***********/
		if(RC_CtrlData.key.v & W)
		{
			chassis.vx_offset  = chassis.vx_f_offset =  forward_back_speed * ramp_calc(&FBSpeedRamp);
		}
		else if(RC_CtrlData.key.v & S)
		{
			chassis.vx_offset = chassis.vx_f_offset =  -forward_back_speed * ramp_calc(&FBSpeedRamp);
		}
		else
		{
			chassis.vx_offset = chassis.vx_f_offset = 0;
			ramp_init(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
		}
		
//		if(RC_CtrlData.key.v & G)
//		{
//			chassis.vx_f_offset =  forward_back_speed * ramp_calc(&FBSpeedRamp);
//		}
//		else if(RC_CtrlData.key.v & B)
//		{
//			chassis.vx_f_offset = -forward_back_speed * ramp_calc(&FBSpeedRamp);
//		}
//		else
//		{
//			chassis.vx_f_offset = 0;
//			ramp_init(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
//		}
		
		/*********** get rightward chassis wheel ref speed ***********/
		if(RC_CtrlData.key.v & A)
		{
			chassis.vy_offset = -left_right_speed * ramp_calc(&LRSpeedRamp);
		}
		else if(RC_CtrlData.key.v & D)
		{
			chassis.vy_offset = left_right_speed * ramp_calc(&LRSpeedRamp);
		}
		else
		{
			chassis.vy_offset = 0;
			ramp_init(&LRSpeedRamp, MOUSR_LR_RAMP_TICK_COUNT);
		}
		
		/*********** chassis mode is waist ***********/
		if((RC_CtrlData.key.v & X) && (handler_run_time -turn_time_last>350))																
		{
			turn_time_last = handler_run_time;
			if(TIM4->CCR3 == 20000)
				TIM4->CCR3 = 0;
			else
				TIM4->CCR3 = 20000;
		}
		
		if((RC_CtrlData.key.v & C) && (handler_run_time -turn_time_last>350))																
		{
			turn_time_last = handler_run_time;
			if(TIM4->CCR2 == 20000)
				TIM4->CCR2 = 0;
			else
				TIM4->CCR2 = 20000;
		}
		
		if((RC_CtrlData.key.v & V) && (handler_run_time -turn_time_last>350))																
		{
			turn_time_last = handler_run_time;
			if(TIM4->CCR1 == 20000)
				TIM4->CCR1 = 0;
			else
				TIM4->CCR1 = 20000;
		}
		
	}
	
}


