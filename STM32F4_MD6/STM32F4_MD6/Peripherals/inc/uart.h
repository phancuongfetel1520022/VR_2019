
/**********************************************************************
File    : uart.h
Purpose : 
**********************************************************************/
#include "stdio.h" 
#ifndef __UART_H__
#define __UART_H__
/****************************** Includes *****************************/
/****************************** Defines *******************************/
/***************************** Prototypes *****************************/
void USART_Config(void);
int fputc(int ch,FILE *f); // Primary UART for QUAT/ACCEL/GYRO data

#endif // __UART_H__


