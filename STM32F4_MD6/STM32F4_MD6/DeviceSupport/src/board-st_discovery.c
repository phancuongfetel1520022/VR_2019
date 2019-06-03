#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_usart.h" 
#include "stm32f4xx_hal_rcc.h"
#include "spi.h"
#include "uart.h"
#include "main.h"
//#include "gpio.h"
#include "board-st_discovery.h"
#include "stm32f4xx_it.h"
#include "stdint.h"
#define TICK_FREQ (1000u)
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi3;


volatile uint32_t g_ul_ms_ticks=0;
extern uint32_t uwTick;

static volatile uint32_t TimingDelay=0;
unsigned long idle_time=0;
extern uint32_t SystemCoreClock; //160000000 (original value)


void board_init()
{
	SystemCoreClockUpdate();
	if (SysTick_Config (SystemCoreClock / TICK_FREQ))
	{
		while(1);
	}
	//Set_SPI_Retry(5);
	
	HAL_MspInit();
	HAL_SPI_MspInit(&hspi3);
	HAL_UART_MspInit(&huart2);
	
}

void mdelay(unsigned long nTime)
{

    HAL_Delay(nTime);
}

void get_tick_count(unsigned long *count)
{

	(*count)=HAL_GetTick();
}


	
