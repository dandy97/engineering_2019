#include "main.h"

/* stack usage monitor */
UBaseType_t can_send_surplus;

/* communicate task global parameter */
motor_current_t glb_cur;

void can_msg_send_task(void const *argument)
{
	osEvent event;
	for(;;)
	{
		event = osSignalWait(GIMBAL_MOTOR_MSG_SEND   | \
                         CHASSIS_MOTOR_MSG_SEND, osWaitForever);
		
		 if (event.status == osEventSignal)
    {
      if (event.value.signals & CHASSIS_MOTOR_MSG_SEND)
      {
        //send_chassis_motor_ctrl_message(glb_cur.chassis_cur);
				if(handler_run_time < 2000) {
					send_Gyro(0x30,1000); /*Ð£×¼ÍÓÂÝÒÇ*/
				}
      }
			else if (event.value.signals & GIMBAL_MOTOR_MSG_SEND)
      {
				//send_gimbal_motor_ctrl_message(glb_cur.gimbal_cur);
      }
		
    }
		can_send_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}
