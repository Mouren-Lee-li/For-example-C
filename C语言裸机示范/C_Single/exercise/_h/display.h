#ifndef __DISPLAY_H
#define __DISPLAY_H
#include "DataBase.h"
#include "initial.h"

#define SYS_DIS     0x00
#define SYS_EN      0x01
#define LCD_OFF     0x02
#define LCD_ON      0x03
#define XTAL_32K    0x14
#define RC_256K     0x18
#define BIAS2
#define BIAS3_COM4  0x29
#define TOPT        0xe0
#define TNORMAL     0xe3

void Display(T_RH_PARA* pT_RH_dat, STATUS sta);
void DisplayAll(void);
void TM1621_Init(void);


#endif
