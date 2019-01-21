#ifndef __SD_H__
#define __SD_H__

#include "main.h"

extern SD_HandleTypeDef SDcard_Handle;

void MY_SDcard_Init(void);
void show_sdcard_info(void);
void Read_From_Sd(uint8_t* buf,uint32_t sector,uint8_t cnt);
void Write_From_Sd(uint8_t* buf,uint32_t sector,uint8_t cnt);

#endif
