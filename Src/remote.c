#include "remote.h"
#include <stdlib.h>

CAN_RxHeaderTypeDef RxHeader;
Speed_System Speed_Motor;
UART_RX_BUFFER Uart1_Rx;
extern BASE base;

uint8_t Tx_Data[8];
uint8_t Rx_Data[8];
uint16_t Motor_Num;


RC_Ctl_t RC_CtrlData={
	{1024,1024,1024,1024,2,2},
	{0},
	{0},
	1024,
	0,
};

void RemoteDataProcess(uint8_t *pData) 
{     	
	if(pData == 0)     
	{         
		return;     
	}          
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;      
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;    
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;     
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;          
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;     
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003); 
 
  RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);     
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);     
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);     
 
  RC_CtrlData.mouse.press_l = pData[12];     
	RC_CtrlData.mouse.press_r = pData[13];       
	RC_CtrlData.key.vh = ((int16_t)pData[14]);
	RC_CtrlData.key.vl = ((int16_t)pData[15]);      
	RC_CtrlData.SW = (uint16_t)((pData[17]<<8)|pData[16]);
	
	RC_CtrlData.update = 1;
}
void Uart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,UART_RX_BUFFER* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData))
{
	uint8_t this_frame_len = 0;
	
	if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
		{   
			__HAL_DMA_DISABLE(hdma_usart_rx);
			__HAL_UART_CLEAR_IDLEFLAG(huart);  
			
			this_frame_len = Uart_Rx->Length_Max - __HAL_DMA_GET_COUNTER(hdma_usart_rx);
			if(Uart_Rx->Buffer_Num)
			{
				Uart_Rx->Buffer_Num = 0;
				HAL_UART_Receive_DMA(huart, Uart_Rx->Buffer[0], Uart_Rx->Length_Max);
				if(this_frame_len == Uart_Rx->Length_Max)
					if(DataProcessFunc) DataProcessFunc(Uart_Rx->Buffer[1]);
			}
			else
			{
				Uart_Rx->Buffer_Num = 1;
				HAL_UART_Receive_DMA(huart, Uart_Rx->Buffer[1], Uart_Rx->Length_Max);
				if(this_frame_len == Uart_Rx->Length_Max)
					if(DataProcessFunc) DataProcessFunc(Uart_Rx->Buffer[0]);
			}
		}
}

void Usart_All_Init(void)
{
	
	Uart1_Rx.Buffer[0]=(uint8_t*)malloc(sizeof(uint8_t)*USART1_RX_LEN_MAX);
	Uart1_Rx.Buffer[1]=(uint8_t*)malloc(sizeof(uint8_t)*USART1_RX_LEN_MAX);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	Uart1_Rx.Buffer_Num = 0;
	Uart1_Rx.Length_Max=USART1_RX_LEN_MAX;
	HAL_UART_Receive_DMA(&huart1, Uart1_Rx.Buffer[0], USART1_RX_LEN_MAX);
}


void Rx_Motor_All(void)
{
	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxHeader,Rx_Data);
	switch (Motor_Num)
	{
		case 0x201:Speed_Info_Analysis(&base.MotorLF.Info,Rx_Data);
		break;		
		case 0x202:Speed_Info_Analysis(&base.MotorRF.Info,Rx_Data);
		break;		
		case 0x203:Speed_Info_Analysis(&base.MotorLB.Info,Rx_Data);
		break;
		case 0x204:Speed_Info_Analysis(&base.MotorRB.Info,Rx_Data);
		break;
	}	
}

void Tx_Motor_All(void)
{
	base.MotorLF.Tar_Speed=Target1-Target2;
	PID_Speed_Cal(&base.MotorLF,Tx_Data);
	
	base.MotorRF.Tar_Speed=-Target1-Target2;
	PID_Speed_Cal(&base.MotorRF,Tx_Data);
	
	base.MotorLB.Tar_Speed=-Target1-Target2;
	PID_Speed_Cal(&base.MotorLB,Tx_Data);
	
	base.MotorRB.Tar_Speed=-Target1+Target2;
  PID_Speed_Cal(&base.MotorRB,Tx_Data);
	
	
  Send_To_Motor(&hcan1,Tx_Data);
}

