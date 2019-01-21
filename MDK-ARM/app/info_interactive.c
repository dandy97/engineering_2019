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
	/* get gimbal and chassis relative angle */
	chassis.follow_gimbal = moto_yaw.total_angle;

	/* get chassis wheel fdb positin */
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis.wheel_pos_fdb[i] = moto_chassis[i].total_angle/19.0f;
  }
  /* get chassis wheel fdb speed */
  for (uint8_t i = 0; i < 4; i++)
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
	//send_chassis_cur(1000,0,0,0);
	 send_chassis_cur(chassis_cur[0] * chassis.power_surplus, chassis_cur[1] *chassis.power_surplus, 
										chassis_cur[2] * chassis.power_surplus, chassis_cur[3] *chassis.power_surplus);
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
	
	/* get gimbal relative palstance */
  //the Z axis(yaw) of gimbal coordinate system corresponds to the IMU Z axis
  gim.sensor.yaw_palstance = gyro_data.v_z;
  //the Y axis(pitch) of gimbal coordinate system corresponds to the IMU -X axis
  gim.sensor.pit_palstance = gyro_data.v_x;
	
	/* get remote and keyboard gimbal control information */
}

void send_gimbal_motor_ctrl_message(int16_t gimbal_cur[])
{
  /* 0: yaw motor current
     1: pitch motor current
     2: trigger motor current*/
  send_gimbal_cur(-gimbal_cur[0], gimbal_cur[1], gimbal_cur[2]);
}

/************************************* GIMBAL  INFO *********************************************/


static uint32_t turn_time_last = 0;
/******************************** CHASSIS REMOTE  HANDLER ***************************************/

void remote_ctrl_chassis_hook(void)
{
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == REMOTE_CTRL))
	{
		/* get chassis wheel ref speed */
		if(ramp_mode == RAMP_UP)
		{
			chassis.vx_offset = RC_CtrlData.rc.ch1 * 30.0f / 660;
			chassis.vy_offset = RC_CtrlData.rc.ch0 * 30.0f / 660;
		}
		else
		{
			chassis.vx_offset = RC_CtrlData.rc.ch1 * CHASSIS_REF_FACT;
			chassis.vy_offset = RC_CtrlData.rc.ch0 * CHASSIS_REF_FACT;
		}
	}
}

void keyboard_chassis_hook(void)
{
	static float forward_back_speed = 0;
	static float left_right_speed 	 = 0;
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == KEYBOR_CTRL))
	{
		/*********** speed mode: normal speed/high speed ***********/                   
		if(ramp_mode == RAMP_UP)
		{
			forward_back_speed = 30.0f;
			left_right_speed   = 30.0f;
		}
		else if(RC_CtrlData.key.v & SHIFT)																	
		{
			forward_back_speed = HIGH_FORWARD_BACK_SPEED;
			left_right_speed 	 = HIGH_LEFT_RIGHT_SPEED;
		}
		else if(gim.ctrl_mode == GIMBAL_WRITHE)
		{
			forward_back_speed = 40.0f;
			left_right_speed 	 = 30.0f;
		}
		else
		{
			forward_back_speed = NORMAL_FORWARD_BACK_SPEED;
			left_right_speed   = NORMAL_LEFT_RIGHT_SPEED;
		}
		
		/*********** get forward chassis wheel ref speed ***********/
		if(RC_CtrlData.key.v & W)
		{
			chassis.vx_offset =  forward_back_speed * ramp_calc(&FBSpeedRamp);
		}
		else if(RC_CtrlData.key.v & S)
		{
			chassis.vx_offset = -forward_back_speed * ramp_calc(&FBSpeedRamp);
		}
		else
		{
			chassis.vx_offset = 0;
			ramp_init(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
		}
		
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
		if((RC_CtrlData.key.v & CTRL) && (handler_run_time -turn_time_last>350))																
		{
			turn_time_last = handler_run_time;
			
			if(gim.ctrl_mode == GIMBAL_NORMAL)
			{	
				gim.ctrl_mode = GIMBAL_WRITHE;
			}
			else
			{
				gim.ctrl_mode = GIMBAL_NORMAL;
			}
		}
		
	}
	
}

/********************************* GIMBAL REMOTE  HANDLER ***************************************/
void remote_ctrl_gimbal_hook(void)
{
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == REMOTE_CTRL))
		{					
			/* get remote gimbal info */
		  gim.pid.yaw_angle_ref -= RC_CtrlData.rc.ch2 * GIMBAL_YAW_REF_FACT;
			gim.pid.pit_angle_ref += RC_CtrlData.rc.ch3 * GIMBAL_PIT_REF_FACT;
		}
}

void keyboard_gimbal_hook(void)
{	
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == KEYBOR_CTRL))
	{
		VAL_LIMIT(RC_CtrlData.mouse.x, -128, 128); 
		VAL_LIMIT(RC_CtrlData.mouse.y, -18, 32);   
		/* get remote gimbal info */
		gim.pid.yaw_angle_ref -= RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT  ;//+ shoot_buff_data.dynamic_yaw * 0.015f;
		gim.pid.pit_angle_ref += RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT ;//+ shoot_buff_data.dynamic_pit * 0.015f;
	}	
	else
	{
		RC_CtrlData.mouse.x = 0;
		RC_CtrlData.mouse.y = 0;
	}
	
}

/*********************************** SHOOT REMOTE  HANDLER **************************************/
/*********************************** SHOOT REMOTE  HANDLER **************************************/

