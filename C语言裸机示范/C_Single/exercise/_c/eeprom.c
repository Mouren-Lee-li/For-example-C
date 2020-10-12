#include "eeprom.h"



//-----------从相应的地址中写数据---------//
/*程序说明
           FLASH_DUKR = 0xAE;
           FLASH_DUKR = 0x56;
           FLASH_CR2 = 0x00;
           FLASH_NCR2 = 0xFF;
   前俩个是借程序存储器的密钥
   后俩个是指令
                               */
void EEpromSet(uint16_t address, uint8_t* pdata, uint8_t length) { //write data to eeprom
  FLASH_DUKR = 0xAE;
  FLASH_DUKR = 0x56;
  FLASH_CR2 = 0x00;
  FLASH_NCR2 = 0xFF;
  while (!(FLASH_IAPSR & 0x08));

  for (uint8_t i = 0; i < length; i++) {
    *((uint8_t*)address + i) = *(pdata + i);
  }
}

void EEpromCopy(uint8_t* pdata, uint16_t address, uint8_t length) { //copy from eeprom
  for (uint8_t i = 0; i < length; i++) {
    *(pdata + i) = *((uint8_t*)address + i);
  }
}

uint8_t EEpromReadByte(uint16_t address) {
  uint8_t data;

  data = *((uint8_t*)address);

  return data;
}

void EEpromWriteByte(uint16_t address, uint8_t Data) {
  FLASH_DUKR = 0xAE;
  FLASH_DUKR = 0x56;
  FLASH_CR2 = 0x00;
  FLASH_NCR2 = 0xFF;
  while (!(FLASH_IAPSR & 0x08)); //检测是否准备好

  *((uint8_t*)address) = Data; //朝EEROM写入数据
}
