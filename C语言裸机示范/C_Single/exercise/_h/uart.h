#ifndef UART_H
#define UART_H


#include "DataBase.h"



void uart1_tx_byte(uint8_t data);
void uart1_tx(uint8_t* pdata, uint8_t quantity);
void uart1_init(void);
void uart1_tx_str(uint8_t* str);

#endif