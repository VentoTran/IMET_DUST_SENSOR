#include "uart.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "main.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
//extern UART_HandleTypeDef huart5;
void UART2_putChar(unsigned char _data)
{
	uint8_t dat[1]= {_data};
  HAL_UART_Transmit(&huart2,dat,1,10);
}
void UART2_putString(char *strx)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)strx, strlen(strx),200);
}

void RS485_putString(char *strx)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)strx,strlen(strx),200);
}
void UART_putCharPC(unsigned char _data)
{
	uint8_t dat[1]= {_data};
  HAL_UART_Transmit(&huart3,dat,1,10);
}
void UART_putStringPC(char *strx)
{
  HAL_UART_Transmit(&huart3,(uint8_t *)strx,strlen(strx),200);
}


//--------------------------------------------END------------------------------------//


