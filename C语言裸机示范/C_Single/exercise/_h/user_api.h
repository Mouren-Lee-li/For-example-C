#ifndef USER_API_H
#define USER_API_H

#include "DataBase.h"
#include "display.h"
#include "sht20.h"
#include "modbus.h"


/*user parameter define*/

#define     MAX_TIME           99
#define     MIN_TIME           1

#define     TRADE_TIME         600 //600 seconds----10 minutes
#define     SVR_TIMEOUT        120 //seconds----2 minutes
#define     ICCID_INDEX        "898602B" //SIM卡的ICCID前7位索引

void board_system_init(void);
void sys_para_init(void);
uint8_t stop_system(void);

#endif
