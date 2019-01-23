#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"

#define RUN_WRITHE_ANGLE_LIMIT					  25
#define STATIC_WRITHE_ANGLE_LIMIT					50
#define WRITHE_SPEED_LIMIT					      30.0f/1.3f

/* chassis speed ramp */
#define MOUSR_LR_RAMP_TICK_COUNT		100
#define MOUSR_FB_RAMP_TICK_COUNT		100

/* chassis control period time (ms) */
#define CHASSIS_PERIOD 4

/* gimbal control period time (ms) */
#define GIMBAL_PERIOD  1

typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // rotate
	float           vx_f;
	float           vy_f;
	float           vw_f;
	float        		vx_offset;
  float        		vy_offset; 
	float           vw_offset;
  float        		vx_f_offset;
  float        		vy_f_offset; 
	float           vw_f_offset;
	
	float         	follow_gyro;
	float					  writhe_speed_fac;
  
//  chassis_mode_e  ctrl_mode;
//  chassis_mode_e  last_ctrl_mode;

	float           wheel_pos_fdb[6];
  float           wheel_pos_ref[6];
	
  float           wheel_spd_fdb[6];
  float           wheel_spd_ref[6];
	
  int16_t         current[6];
	float         	power_surplus;
  
} chassis_t;

void chassis_task(void const * argument);
void chassis_param_init(void);

extern chassis_t chassis;
extern float Robot_angle_ref;

#endif
