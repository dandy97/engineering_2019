#include "main.h"

SD_HandleTypeDef SDcard_Handle;
HAL_SD_CardInfoTypedef SD_Card_Info;

void MY_SDcard_Init(void)
{
	SDcard_Handle.Instance = SDIO;
  SDcard_Handle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  SDcard_Handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  SDcard_Handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  SDcard_Handle.Init.BusWide = SDIO_BUS_WIDE_1B;
  SDcard_Handle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  SDcard_Handle.Init.ClockDiv = 0;
  if (HAL_SD_Init(&SDcard_Handle, &SD_Card_Info) != SD_OK)
  {
    Error_Handler();
  }
  HAL_SD_WideBusOperation_Config(&SDcard_Handle, SDIO_BUS_WIDE_4B);
}

void HAL_SD_MspInit(SD_HandleTypeDef* sdHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(sdHandle->Instance==SDIO)
  {
    __HAL_RCC_SDIO_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();   
    __HAL_RCC_GPIOD_CLK_ENABLE();  
  
    /**SDIO GPIO Configuration    
    PC8     ------> SDIO_D0
    PC9     ------> SDIO_D1
    PC10     ------> SDIO_D2
    PC11     ------> SDIO_D3
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
}

//得到卡信息
//cardinfo:卡信息存储区
//返回值:错误状态
uint8_t SD_GetCardInfo(HAL_SD_CardInfoTypedef *cardinfo)
{
	uint8_t sta;
	sta = HAL_SD_Get_CardInfo(&SDcard_Handle,cardinfo);
	return sta;
}

//打印SD卡相关信息
void show_sdcard_info(void)
{
	switch(SD_Card_Info.CardType)
	{
		case STD_CAPACITY_SD_CARD_V1_1:printf("Card Type:SDSC V1.1\r\n");break;
		case STD_CAPACITY_SD_CARD_V2_0:printf("Card Type:SDSC V2.0\r\n");break;
		case HIGH_CAPACITY_SD_CARD:printf("Card Type:SDHC V2.0\r\n");break;
		case MULTIMEDIA_CARD:printf("Card Type:MMC Card\r\n");break;
	}	
  printf("Card ManufacturerID:%d\r\n",SD_Card_Info.SD_cid.ManufacturerID);	//制造商ID
 	printf("Card RCA:%d\r\n",SD_Card_Info.RCA);								//卡相对地址
	printf("Card Capacity:%d MB\r\n",(uint32_t)(SD_Card_Info.CardCapacity>>20));	//显示容量
 	printf("Card BlockSize:%d\r\n\r\n",SD_Card_Info.CardBlockSize);			//显示块大小
}

void Read_From_Sd(uint8_t* buf,uint32_t sector,uint8_t cnt)
{
	long long lsector=sector;
	HAL_SD_ReadBlocks(&SDcard_Handle,(uint32_t*)buf,lsector * 512,512,1);
}

void Write_From_Sd(uint8_t* buf,uint32_t sector,uint8_t cnt)
{
	long long lsector=sector;
	HAL_SD_WriteBlocks(&SDcard_Handle,(uint32_t*)buf,lsector * 512,512,1);
}
