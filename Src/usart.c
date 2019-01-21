#include "main.h"
#include "math.h"
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (uint8_t) ch;      
	return ch;
}
#endif 

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart6;

DMA_HandleTypeDef  UART1TxDMA_Handler;
DMA_HandleTypeDef  UART2RxDMA_Handler;
DMA_HandleTypeDef  UART3RxDMA_Handler;
DMA_HandleTypeDef  UART4RxDMA_Handler;
DMA_HandleTypeDef  UART6RxDMA_Handler;

RC_Ctl_t RC_CtrlData = {0};
pc_data_t pc_data = {0};

extern uint32_t pc_i;
uint8_t rc_RxBuffer[RECEIVELEN];
uint8_t Rx_data[14];
uint8_t optic_flow[RECEIVELEN];
optic_flow_data_t optic_flow_data;

// USART1 init function         			 串口调试
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}
// USART2 init function                 遥控接受串口，偶校验，波特率：100000
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
	huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;	
  HAL_UART_Init(&huart2);
	
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)rc_RxBuffer, RECEIVELEN);	
}
/* USART3 裁判系统 */
void MX_USART3_UART_Init(void)
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;	
  HAL_UART_Init(&huart3);
	
	SET_BIT(huart3.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)JudgeDataBuffer, 1024);
}

/* 无线 init function */
void MX_USART4_UART_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

/*工控*/
void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
	SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)Rx_data, RECEIVELEN);	
}

//开启串口时钟
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();  
    /* PA9     ------> USART1_TX
       PA10     ------> USART1_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		UART1TxDMA_Handler.Instance=DMA2_Stream7;                           
    UART1TxDMA_Handler.Init.Channel=DMA_CHANNEL_4;   									
    UART1TxDMA_Handler.Init.Direction = DMA_MEMORY_TO_PERIPH;
    UART1TxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    UART1TxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    UART1TxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    UART1TxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    UART1TxDMA_Handler.Init.Mode = DMA_CIRCULAR;
    UART1TxDMA_Handler.Init.Priority = DMA_PRIORITY_LOW;
    UART1TxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&UART1TxDMA_Handler);
		
	  __HAL_LINKDMA(uartHandle,hdmatx,UART1TxDMA_Handler);
		
		HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);				
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();   
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
   /* PD5     ------> USART2_TX
      PD6     ------> USART2_RX */ 
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
		//RX
		UART2RxDMA_Handler.Instance=DMA1_Stream5;                           
    UART2RxDMA_Handler.Init.Channel=DMA_CHANNEL_4;   									
    UART2RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
    UART2RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    UART2RxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    UART2RxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    UART2RxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    UART2RxDMA_Handler.Init.Mode = DMA_CIRCULAR;
    UART2RxDMA_Handler.Init.Priority = DMA_PRIORITY_LOW;
    UART2RxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&UART2RxDMA_Handler);           
		
		__HAL_LINKDMA(uartHandle,hdmarx,UART2RxDMA_Handler);    
		
		HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART2中断通道
  }
	//UART3 裁判系统
  else if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE(); 
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
   /* PD8     ------> USART3_TX
      PD9     ------> USART3_RX */  
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
		//RX
		UART3RxDMA_Handler.Instance=DMA1_Stream1;                           
    UART3RxDMA_Handler.Init.Channel=DMA_CHANNEL_4;   									
    UART3RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
    UART3RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    UART3RxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    UART3RxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    UART3RxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    UART3RxDMA_Handler.Init.Mode = DMA_CIRCULAR;
    UART3RxDMA_Handler.Init.Priority = DMA_PRIORITY_LOW;
    UART3RxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&UART3RxDMA_Handler);           
		
		__HAL_LINKDMA(uartHandle,hdmarx,UART3RxDMA_Handler);    
		
		HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
	 else if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_ENABLE(); 
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
   /* PA0     ------> UART4_TX
      PA1     ------> UART4_RX */  
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		//RX
		UART4RxDMA_Handler.Instance=DMA1_Stream2;                           
    UART4RxDMA_Handler.Init.Channel=DMA_CHANNEL_4;   									
    UART4RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
    UART4RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    UART4RxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    UART4RxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    UART4RxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    UART4RxDMA_Handler.Init.Mode = DMA_CIRCULAR;
    UART4RxDMA_Handler.Init.Priority = DMA_PRIORITY_LOW;
    UART4RxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&UART4RxDMA_Handler);           
		
		__HAL_LINKDMA(uartHandle,hdmarx,UART4RxDMA_Handler);    
		
		HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
  else if(uartHandle->Instance==USART6)
  {
    __HAL_RCC_USART6_CLK_ENABLE();   
		__HAL_RCC_DMA2_CLK_ENABLE();	
		__HAL_RCC_GPIOC_CLK_ENABLE();
    /* PC6     ------> USART6_TX
       PC7     ------> USART6_RX */  
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		UART6RxDMA_Handler.Instance=DMA2_Stream1;                           
    UART6RxDMA_Handler.Init.Channel=DMA_CHANNEL_5;   									
    UART6RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
    UART6RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    UART6RxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    UART6RxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    UART6RxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    UART6RxDMA_Handler.Init.Mode = DMA_CIRCULAR;
    UART6RxDMA_Handler.Init.Priority = DMA_PRIORITY_LOW;
    UART6RxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&UART6RxDMA_Handler);           
		
		__HAL_LINKDMA(uartHandle,hdmarx,UART6RxDMA_Handler);
		
		HAL_NVIC_EnableIRQ(USART6_IRQn);
		HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
		
  }
}

//关闭串口时钟
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_DISABLE();  
   /* PA0-WKUP     ------> UART4_TX
			PA1     ------> UART4_RX */  
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
  }
  else if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();     
   /* PA9     ------> USART1_TX
			 PA10     ------> USART1_RX */ 
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();     
   /* PD5     ------> USART2_TX
			PD6     ------> USART2_RX */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);
  }
  else if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();   
   /* PD8     ------> USART3_TX
			 PD9     ------> USART3_RX */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);
  }
  else if(uartHandle->Instance==USART6)
  {
    __HAL_RCC_USART6_CLK_DISABLE();     
   /* PC6     ------> USART6_TX
      PC7     ------> USART6_RX  */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);
  }
} 

uint16_t	temp;

//DMA1 STREAM5
void USART2_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) &&   /*读取USART_SR寄存器的IDLE位*/
		  __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE))
	{
		uint16_t tmp = huart2.Instance->SR;  //检测到空闲线路，硬件置1，软件清0 （读入USART_DR和USART_SR）
		tmp = huart2.Instance->DR;
		tmp--;
		CLEAR_BIT(huart2.Instance->SR, USART_SR_IDLE); //IDLE位清零
		__HAL_DMA_DISABLE(huart2.hdmarx);
		temp = huart2.hdmarx->Instance->NDTR;  //要传输的剩余数据项数
		if((RECEIVELEN - temp) == 18)//传输数组长度 - 传输的剩余数据项数
		{
			Get_Remote_info(&RC_CtrlData ,rc_RxBuffer);
		}
		HAL_UART_Receive_DMA(&huart2, (uint8_t *)rc_RxBuffer, RECEIVELEN);
		SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
		DMA1->HIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;//DMA中断清0
		__HAL_DMA_SET_COUNTER(huart2.hdmarx, RECEIVELEN); //重载NDTR位
		__HAL_DMA_ENABLE(huart2.hdmarx);
	} 
}

//DMA1 STREAM1
void USART3_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE))
    {
			
      uint16_t tmp = huart3.Instance->DR;
      tmp = huart3.Instance->SR;
      tmp--;
      CLEAR_BIT(huart2.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart3.hdmarx);
			
     	temp = huart3.hdmarx->Instance->NDTR;  
			if((1024 - temp) >=1 && (1024 - temp) <=222)
				Judge_Receive(JudgeDataBuffer);
			}
      DMA1->LIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;
      __HAL_DMA_SET_COUNTER(huart3.hdmarx, 1024);
      __HAL_DMA_ENABLE(huart3.hdmarx);
} 

//DMA1 STREAM2
void UART4_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_IDLE))
    {
			
      uint16_t tmp = huart4.Instance->DR;
      tmp = huart4.Instance->SR;
      tmp--;
      CLEAR_BIT(huart2.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart4.hdmarx);
			
     	temp = huart4.hdmarx->Instance->NDTR;  
			if((RECEIVELEN - temp) == 21){
			
			}
		}
    DMA1->LIFCR = DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TCIF2_6 | DMA_FLAG_TEIF2_6; 
		__HAL_DMA_SET_COUNTER(huart4.hdmarx, RECEIVELEN);
		__HAL_DMA_ENABLE(huart4.hdmarx);
}

//DMA2 STREAM1
void USART6_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))
    {
			
      uint16_t tmp = huart6.Instance->DR;
      tmp = huart6.Instance->SR;
      tmp--;
      CLEAR_BIT(huart6.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart6.hdmarx);
			
     	temp = huart6.hdmarx->Instance->NDTR;  
			if((RECEIVELEN - temp) == 7)
			{
					
			}
      DMA2->LIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, RECEIVELEN);
      __HAL_DMA_ENABLE(huart6.hdmarx);
		}
}

void Get_Remote_info(RC_Ctl_t *rc, uint8_t *pData)
{
	Reset ++;//遥控信号计数量
	rc->rc.ch0 = (pData[0] | pData[1] << 8) & 0x07FF;
  rc->rc.ch0 -= 1024;
  rc->rc.ch1 = (pData[1] >> 3 | pData[2] << 5) & 0x07FF;
  rc->rc.ch1 -= 1024;
  rc->rc.ch2 = (pData[2] >> 6 | pData[3] << 2 | pData[4] << 10) & 0x07FF;
  rc->rc.ch2 -= 1024;
  rc->rc.ch3 = (pData[4] >> 1 | pData[5] << 7) & 0x07FF;
  rc->rc.ch3 -= 1024;
 
	/* prevent remote control zero deviation */
  if(rc->rc.ch0 <= 10 && rc->rc.ch0 >= -10)
    rc->rc.ch0 = 0;
  if(rc->rc.ch1 <= 10 && rc->rc.ch1 >= -10)
    rc->rc.ch1 = 0;
  if(rc->rc.ch2 <= 10 && rc->rc.ch2 >= -10)
    rc->rc.ch2 = 0;
  if(rc->rc.ch3 <= 10 && rc->rc.ch3 >= -10)
    rc->rc.ch3 = 0;
	
 rc->rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
 rc->rc.s2 = ((pData[5] >> 4) & 0x0003);

 rc->mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
 rc->mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
 rc->mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

 rc->mouse.press_l = pData[12];
 rc->mouse.press_r = pData[13];

 rc->key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
}
