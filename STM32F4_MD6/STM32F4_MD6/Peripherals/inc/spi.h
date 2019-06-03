/*********************************************************************
File    : spi.h
Purpose : 
**********************************************************************/

/****************************** Includes *****************************/
//#include "stdint.h"
/****************************** Defines *******************************/
#define SENSORS_I2C               I2C2

#define I2C_SPEEDX                 400000
#define I2C_OWN_ADDRESS           0x00


void I2cMaster_Init(void);
//void Set_SPI_Retry(unsigned short ml_sec);
////unsigned short Get_SPI_Retry(void);


int Sensors_SPI_ReadRegister_X(unsigned char slave_addr, unsigned char reg_addr, unsigned short len, unsigned char *data_ptr);
int Sensors_SPI_WriteRegister_X(unsigned char slave_addr, unsigned char reg_addr, unsigned short len, const unsigned char *data_ptr);




