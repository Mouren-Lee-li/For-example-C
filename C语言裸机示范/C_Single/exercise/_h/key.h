#ifndef KEY_H
#define KEY_H

#include "DataBase.h"
#include "user_api.h"
#include "eeprom.h"
#include "Display.h"

typedef enum{
  KEY1 = 0x0001, 
  LNG_PRS_1SEC = 0x0100,
  LNG_PRS_2SEC = 0x0200,
  LNG_PRS_3SEC = 0x0300,
  LNG_PRS_4SEC = 0x0400,
  LNG_PRS_5SEC = 0x0500,
  LNG_PRS = 0xff00
}KEY_INFO;



KEY_INFO KeyScan(void);
KEY_INFO KeyFuncAnls(uint16_t key_value);
void KeyFuncHandle(uint16_t key_info);

#endif