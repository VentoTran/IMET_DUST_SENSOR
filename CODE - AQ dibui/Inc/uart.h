#ifndef UART_H_
#define UART_H_

#include "stm32f1xx_hal.h"
//void UART_init(UART_HandleTypeDef *uart);
void UART2_putChar(unsigned char _data);
void UART2_putString(char *strx);
void UART_putCharPC(unsigned char _data);
void UART_putStringPC(char *strx);
void RS485_putString(char *strx);

#endif /* UART_H_ */

