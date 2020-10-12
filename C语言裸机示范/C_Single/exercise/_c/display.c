#include "display.h"

#define NOTHING   0x00
//#define POINT     0x
const uint8_t CODE_TABLE1[] = {
  0x7D, 0x60, 0x3E, 0x7A, 0x63, 0x5B, 0x5F, 0x70, 0x7F, 0x7B
};

const uint8_t CODE_TABLE2[] = {
  0xD7, 0x06, 0xE3, 0xA7, 0X36, 0xB5, 0xF5, 0x07, 0xF7, 0xB7
};

void TM1621_delay(uint16_t i) { //us延时
  for (; i > 0; i--) {
    asm("nop"); //一个asm("nop")函数经过示波器测试代表100ns
    asm("nop");
    asm("nop");
    asm("nop");
  }
}

void TM1621_SendByte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x01)
      TM1621_DATA = 1;
    else
      TM1621_DATA = 0;

    TM1621_WR = 0;
    TM1621_delay(1);
    TM1621_WR = 1;
    TM1621_delay(1);
    data = data >> 1;
  }
}

//向TM1621发送命令--------------------------------------
void TM1621_SendCmd(uint8_t cmd) {
  TM1621_CS = 1;
  TM1621_delay(10);
  TM1621_CS = 0;

  /*输入命令头--100*/
  TM1621_DATA = 1;
  TM1621_WR = 0;
  TM1621_delay(1);
  TM1621_WR = 1;
  TM1621_delay(1);

  TM1621_DATA = 0;
  TM1621_WR = 0;
  TM1621_delay(1);
  TM1621_WR = 1;
  TM1621_delay(1);

  TM1621_DATA = 0;
  TM1621_WR = 0;
  TM1621_delay(1);
  TM1621_WR = 1;
  TM1621_delay(1);

  for (uint8_t i = 0; i < 8; i++) {
    if (cmd & 0x80)
      TM1621_DATA = 1;
    else
      TM1621_DATA = 0;

    TM1621_WR = 0;
    TM1621_delay(1);
    TM1621_WR = 1;
    TM1621_delay(1);
    cmd = cmd << 1;
  }

  TM1621_DATA = 0; //此位可以0或1
  TM1621_WR = 0;
  TM1621_delay(1);
  TM1621_WR = 1;
  TM1621_delay(1);
}

//写地址
void TM1621_Addr(uint8_t addr) {//101命令头+6位地址码
  TM1621_CS = 1;
  TM1621_delay(10);
  TM1621_CS = 0;

  addr= addr<<2;
  /*输入命令头--101*/
  TM1621_DATA = 1;
  TM1621_WR = 0;
  TM1621_delay(1);
  TM1621_WR = 1;
  TM1621_delay(1);

  TM1621_DATA = 0;
  TM1621_WR = 0;
  TM1621_delay(1);
  TM1621_WR = 1;
  TM1621_delay(1);

  TM1621_DATA = 1;
  TM1621_WR = 0;
  TM1621_delay(1);
  TM1621_WR = 1;
  TM1621_delay(1);

  for (uint8_t i = 0; i < 6; i++) {
    if (addr & 0x80)
      TM1621_DATA = 1;
    else
      TM1621_DATA = 0;

    TM1621_WR = 0;
    TM1621_delay(1);
    TM1621_WR = 1;
    TM1621_delay(1);
    addr = addr << 1;
  }
}
 
void TM1621_Init(void) {
  TM1621_SendCmd(0xE3);    //普通模式
  TM1621_SendCmd(0x01);    //打开系统振荡器
  TM1621_SendCmd(0x18);    //内部RC振荡器
  TM1621_SendCmd(0x03);    //打开LCD偏压发生器
  TM1621_SendCmd(0x29);    //1/3偏压，4个COM
}

/*显示所有,此项目SEG19往后都没用*/
void DisplayAll(void) {
  TM1621_Addr(0x00); //SEG0-SEG5
  TM1621_SendByte(0xff);
  TM1621_SendByte(0xff);
  TM1621_SendByte(0xff);
  TM1621_Addr(0x0D); //SEG13-SEG18
  TM1621_SendByte(0xff);
  TM1621_SendByte(0xff);
  TM1621_SendByte(0xff);
//TM1621_SendByte(0x0f); //SEG19
//TM1621_Addr(0x1A); //SEG26-SEG30
//TM1621_SendByte(0xff);
//TM1621_SendByte(0xff);
//TM1621_SendByte(0x0f);
}

void Display(T_RH_PARA* pT_RH_dat, STATUS sta) {
  uint16_t tmp = 0;

  switch (sta) {
    case RS485_ADDR_STATUS:
      tmp = mod_data.addr;
      TM1621_Addr(0x00); //SEG0-SEG5
      TM1621_SendByte(0x00); //地址范围为0--15,不需要显示百位
      TM1621_SendByte(CODE_TABLE1[tmp%100/10]); //点亮小数点
      TM1621_SendByte(CODE_TABLE1[tmp%10]);

      tmp = (uint16_t)(10 * pT_RH_dat->T);
      TM1621_Addr(0x0D); //SEG13-SEG18
      TM1621_SendByte(0xe6); //显示"d"
      TM1621_SendByte(0xe6); //显示"d"
      TM1621_SendByte(0x77); //显示"A"
      break;
    case NORMAL_RUN_STATUS:
      tmp = (uint16_t)(10 * pT_RH_dat->RH);
      if (tmp > 999)
        tmp = 999;
      TM1621_Addr(0x00); //SEG0-SEG5
      TM1621_SendByte(CODE_TABLE1[tmp/100]);
      TM1621_SendByte(CODE_TABLE1[tmp%100/10] | 0x80); //点亮小数点
      TM1621_SendByte(CODE_TABLE1[tmp%10]);

      if (pT_RH_dat->T >= 0) {
        tmp = (uint16_t)(10 * pT_RH_dat->T);
        if (tmp > 999)
          tmp = 999;
        TM1621_Addr(0x0D); //SEG13-SEG18
        TM1621_SendByte(CODE_TABLE2[tmp%10] | 0x08);
        TM1621_SendByte(CODE_TABLE2[tmp%100/10] | 0x08);
        TM1621_SendByte(CODE_TABLE2[tmp/100] | 0x08);
      } else {
        if (pT_RH_dat->T < -9.9) { //显示极限值-9.9
          tmp = 99;
        } else  {
          tmp = (uint16_t)(10 * fabs(pT_RH_dat->T));
        }
        TM1621_Addr(0x0D); //SEG13-SEG18
        TM1621_SendByte(CODE_TABLE2[tmp%10] | 0x08);
        TM1621_SendByte(CODE_TABLE2[tmp%100/10] | 0x08);
        TM1621_SendByte(0x20 | 0x08); //显示‘-’号
      }
      break;
    default:
      break;
  }
}
