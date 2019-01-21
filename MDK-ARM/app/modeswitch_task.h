#ifndef __MODE_SW_TASK_H__
#define __MODE_SW_TASK_H__

#include "main.h"

#define 	REMOTE_CTRL				1			
#define 	KEYBOR_CTRL				3

typedef enum
{
  RAMP_FLAT = 0,
	RAMP_UP   = 1,
	RAMP_DOWN = 2,
} ramp_mode_ee;

void mode_swtich_task(void const *arugment);
void get_main_ctrl_mode(void);
void led_flicker(void);

extern ramp_mode_ee ramp_mode;
extern uint8_t ctrl_mode;
extern uint32_t Reset;

#endif
