#include "user_api.h"

CTRL_PARA ctrl_data;
TIME_PARA time_data;
T_RH_PARA T_RH_data;
DISPLAY_PARA disp_data;
KEY_PARA key_data;
FIFO_DATA recv_buf;
MODBUS_PARA mod_data;

void sys_para_init(void) {
  /*data initialize*/
  ctrl_data.status = RS485_ADDR_STATUS; //开机先显示地址
}


void board_system_init(void) {
  sys_para_init();
  SHT2X_Init();
  TM1621_Init();
  DisplayAll(); //全亮用来检测液晶是否存在断码显示异常
  GetRS485Addr(&mod_data.addr);
  RS485_DIR = 0; //485芯片处于接收模式
}

uint8_t stop_system(void) {

  return 0;
}

