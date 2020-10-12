#include "initial.h"
#include "eeprom.h"
#include "user_api.h"
#include "key.h"
#include "modbus.h"
#include "Display.h"
#include "sht20.h"

void main(void) {
  delay_ms(1000);
  chip_system_init();
  board_system_init();

  /*test Display*/
#if 0
  while (1) {
    Display();
  }
#endif

  /*test key scan*/
#if 0
  while (1) {
    KeyScan();
  }
#endif

  /*test uart*/
#if 0
  while (1) {
    delay_ms(5000);
    uart1_tx_str("-------温湿度采集器---------\r\n");
  }
#endif

  /*test SHT20(温湿度传感器)*/
#if 0
  while (1) {
    delay_ms(5000);
    SHT2X_Init();

    SHT2x_MeasureHM(TEMP, &T_RH_data);
    SHT2x_MeasureHM(HUMIDITY, &T_RH_data);
    PrintData(&T_RH_data);
  }
#endif

  //print start infomation
//uart1_tx_str("-------温湿度采集器---------\r\n");

  if (0x55 == EEpromReadByte(EEPROM_BASE_ADDRESS)) {
    EEpromWriteByte(EEPROM_BASE_ADDRESS, 0x55);
  }

  __enable_interrupt();
  while (1) {
    if (true == T_RH_data.get_value) {
      T_RH_data.get_value = false;
      if (SHT2x_MeasureHM(TEMP, &T_RH_data) != 0) {
        T_RH_data.T = 0;
      }
      if (SHT2x_MeasureHM(HUMIDITY, &T_RH_data) != 0) {
        T_RH_data.RH = 0;
      }
      Display(&T_RH_data, ctrl_data.status);
    }

    if (true == key_data.scan_permit) {
      uint16_t key_info = 0;

      key_data.scan_permit = false;
      key_info = KeyFuncAnls(KeyScan());
      KeyFuncHandle(key_info);
    }

    if (true == recv_buf.req) {
      if (!FrameGet(&recv_buf)) {
        RS485_DIR = 1;	//使485芯片处于发送
        FrameProcess(&mod_data, &T_RH_data);
        delay_us(200); //延时约1ms,确保发送完成,否则会造成最后一字节丢失
        RS485_DIR = 0;	//使485芯片处于接收状态
      }
      recv_buf.req = false; //最后再将req置为false,防止定时器1在过程中recv_buf.num变为
                            //0之前再次将req置为true,起到过临界区的作用
    }
    
  }
}

#pragma vector = TIM1_OVR_UIF_vector
__interrupt __root void TIM1_UIF_HANDLER(void) {  //1ms
  if (recv_buf.timeout >= 5) { //串口5ms内没收到数据则认为此帧数据接收完毕
    recv_buf.timeout = 5;
    if (recv_buf.num != 0)
      recv_buf.req = true;
    else {
      if (recv_buf.head != recv_buf.tail) //纠错,防止任何情况下出现此现象
        recv_buf.head = recv_buf.tail = 0;
    }
  } else {
    recv_buf.timeout++;
  }

  TIM1_SR1_UIF = 0;
}

#pragma vector = TIM2_OVR_UIF_vector
__interrupt __root void TIM2_UIF_HANDLER(void) {  //10ms	
  static uint16_t cnt_get = 0;

  cnt_get++;
  if (cnt_get > 100) {
    cnt_get = 0;
    T_RH_data.get_value = true;
  }

  if (RS485_ADDR_STATUS == ctrl_data.status) {
    static uint16_t cnt_dis_addr = 0;
    static uint8_t old_addr = 0;

    if (old_addr != mod_data.addr) //拨码开关动作,地址改变
      cnt_dis_addr = 0;
    else
      cnt_dis_addr++;

    old_addr = mod_data.addr;
    if (cnt_dis_addr > 300) {
      cnt_dis_addr = 0;
      ctrl_data.status = NORMAL_RUN_STATUS;
    }
  }
  
  /* for key scanning*/
  key_data.cnt_scan++;
  if (key_data.cnt_scan >= 1) {
    key_data.cnt_scan = 0;
    key_data.scan_permit = true;
  }

  if (true == key_data.act_prs) {
    if (key_data.threshold != -1) { //short press
      key_data.threshold++;
    } else { //long press
      static uint16_t count_10ms = 0;

      if (key_data.trig_period != -1) {
        key_data.trig_period++;
      }

      count_10ms++;
      if (count_10ms >= 100) { //1 sec
        count_10ms = 0;
        if (key_data.prs_second != -1) {
          key_data.prs_second++;
        }
      }
    }
  } else {
    key_data.threshold = 0;
    key_data.trig_period = 0;
    key_data.prs_second = 0;
  }

  TIM2_SR1_UIF = 0;
}

#pragma vector = 0x07 //通过查STM8S003XX手册可知中断向量表0x07位置对应PORTC
__interrupt __root void EXTI_PORTC_IRQHANDLER(void) { //实时刷新设备modbus通讯地址
  GetRS485Addr(&mod_data.addr);
  //display current RS485 address
  ctrl_data.status = RS485_ADDR_STATUS;
}

