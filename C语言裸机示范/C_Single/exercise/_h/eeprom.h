#ifndef EEPROM_H
#define EEPROM_H

#include "DataBase.h"


#define EEPMASS1                       0xAE      //password1
#define EEPMASS2                       0x56      //password2
#define EEPROM_BASE_ADDRESS            0x004000  //bass address
#define EEPROM_TEMP_ADDRESS            0x004001  //temperature: user set last time
#define EEPROM_TIME_ADDRESS            0x004002  //time: user set last time
#define EEPROM_TEMP_FORM_ADDRESS       0x004003  //F degree or C degree: user set last time
#define EEPROM_MAX_ADDRESS             0x0043ff  //maxium address

void EEpromSet (uint16_t address, uint8_t* pdata, uint8_t length);
void EEpromCopy (uint8_t* pdata, uint16_t address, uint8_t length);
uint8_t EEpromReadByte(uint16_t address);
void EEpromWriteByte (uint16_t address, uint8_t Data);


#endif
