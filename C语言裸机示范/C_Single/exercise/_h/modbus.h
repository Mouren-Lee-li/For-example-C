#ifndef __MODBUS_H
#define __MODBUS_H
#include "DataBase.h"
#include "uart.h"

uint8_t FrameGet(FIFO_DATA* pFIFO_dat);
void FrameProcess(MODBUS_PARA* pmod_data, T_RH_PARA* pT_RH_dat);
void GetRS485Addr(uint8_t* addr);

#endif