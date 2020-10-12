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
  ctrl_data.status = RS485_ADDR_STATUS; //��������ʾ��ַ
}


void board_system_init(void) {
  sys_para_init();
  SHT2X_Init();
  TM1621_Init();
  DisplayAll(); //ȫ���������Һ���Ƿ���ڶ�����ʾ�쳣
  GetRS485Addr(&mod_data.addr);
  RS485_DIR = 0; //485оƬ���ڽ���ģʽ
}

uint8_t stop_system(void) {

  return 0;
}

