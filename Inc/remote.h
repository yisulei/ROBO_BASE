#ifndef __REMOTE_H__
#define __REMOTE_H__

#define PCKEY_NUM 16
#define BUFFER_LEN_MAX 100
#define USART1_RX_LEN_MAX 18


#include "robo_base.h"
#include "usart.h"

typedef  struct {     

	struct{         
				uint16_t ch0;         
				uint16_t ch1;         
				uint16_t ch2;         
				uint16_t ch3;         
				uint8_t  s1;         
				uint8_t  s2;     
				}rc; 
 
     struct{        
				int16_t x;         
				int16_t y;         
				int16_t z;         
				uint8_t press_l;         
				uint8_t press_r;    
				uint16_t time_press_l;
				uint16_t time_press_r;
				}mouse; 
 
     struct{         
				uint16_t vh;  
				uint16_t vl;  
				uint8_t pc_key[PCKEY_NUM];
				uint16_t time_pckey[PCKEY_NUM];
		   }key; 
	 
	uint16_t SW;				//down1684     up364
	 
	uint8_t update;
	 
}RC_Ctl_t;

#define RC_CtrlData_Initialize \
{\
	{1024,1024,1024,1024,2,2},\
	{0},\
	{0},\
	1024,\
	0,\
}

typedef struct UART_RX_BUFFER
{
	uint8_t* Buffer[2];
	uint8_t Buffer_Num;
	uint16_t Length_Max;
}UART_RX_BUFFER;


void Usart_All_Init(void);
void Remote_Init(void);
void Usart_Receive_IDLE(UART_HandleTypeDef *_huart);
void RemoteDataProcess(uint8_t *pData);
void Uart_DMA_Process(UART_HandleTypeDef *huart,DMA_HandleTypeDef* hdma_usart_rx,UART_RX_BUFFER* Uart_Rx,void(*DataProcessFunc)(uint8_t *pData));
void Speed_PID_Init_All(void);
void Rx_Motor_All(void);
void Tx_Motor_All(void);
#define Target1 (RC_CtrlData.rc.ch1-1024)*4.6
#define Target2 (RC_CtrlData.rc.ch0-1024)*4.6

#endif


