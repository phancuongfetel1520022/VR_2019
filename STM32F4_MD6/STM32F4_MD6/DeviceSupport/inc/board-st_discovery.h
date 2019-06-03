#ifndef __ST_DISCOVERY_BOARD_H
#define __ST_DISCOVERY_BOARD_H

#include "stdint.h"
void board_init(void);
void get_tick_count(unsigned long *count);
void mdelay(unsigned long nTime);
void SysTick_Handler(void);

#endif	/* ST_DISCOVERY_BOARD_H */


