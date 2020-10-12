#ifndef INITIAL_H
#define INITIAL_H

#include "DataBase.h"
#include "uart.h"

#define WWDG_ENABLE 0
#define IWDG_ENABLE 1

void delay_us(uint16_t t);
void delay_ms(uint16_t t);
void chip_system_init(void);
void feed_dog(uint8_t counter);
void WWDG_SWReset(void);


#endif
