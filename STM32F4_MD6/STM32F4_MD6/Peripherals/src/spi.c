///*******************************************************************************
//File    : i2c.c
//Purpose : I2c 3 to communicate with the sensors
//Author  : 
//********************************** Includes ***********************************/
#include <stdio.h>
#include "spi.h"
#include "stm32f4xx_hal.h"


///********************************* Defines ************************************/

extern SPI_HandleTypeDef hspi3;
#define SPI_ENABLE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define SPI_DISABLE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)


/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int Sensors_SPI_ReadRegister_X(unsigned char slave_addr, unsigned char reg_addr, unsigned short len, unsigned char *data_ptr)
{    
		
		unsigned char address;    
		address = reg_addr | 0x80;

    SPI_ENABLE;
    // send register address to read
	
		HAL_SPI_Transmit(&hspi3, &address, 1, 100);
    
    // get len bytes 
		
    HAL_SPI_Receive(&hspi3, data_ptr, len, 100);
	
		SPI_DISABLE;
		
    return 0;
}

int Sensors_SPI_WriteRegister_X(unsigned char slave_addr, unsigned char reg_addr, unsigned short len, const unsigned char *data_ptr)
{   
	
    unsigned char address;    
		address = reg_addr & 0x7F;
    
	
		SPI_ENABLE;
    HAL_SPI_Transmit(&hspi3, &address, 1, 100);
    
    // get len bytes 
    HAL_SPI_Transmit(&hspi3, (unsigned char *)data_ptr, len, 100);
		
		SPI_DISABLE;
		
    return 0;
}
