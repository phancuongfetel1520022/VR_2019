
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "uart.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"


/********************************* Defines ************************************/
extern UART_HandleTypeDef huart2;


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
//	 Loop until the end of transmission 
//	 Checks whether the specified UART flag is set or not ?
	
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,10);
	while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC)==RESET)
	{

	}
	return ch;
}




