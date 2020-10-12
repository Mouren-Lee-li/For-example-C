#include "key.h"
#include "initial.h"


uint16_t KeyScan(void) {
  if (KEY_IN == 0)
    return KEY1;

  return 0;
}

uint16_t KeyFuncAnls(KEY_INFO key_para) {
  uint16_t key_value = 0, cur_key = 0;
  static uint16_t old_key = 0;

  cur_key = key_para;
  if (cur_key != 0 && cur_key == old_key) { //is pressing the key
    key_data.act_prs = true; //start to threshold++ in timer interrupt function

    if (key_data.threshold > 50) { //长按0.5s后激活长按并触发一次
      if (0) { //需要周期性触发的按键
        key_data.lng_prs_mode = 1; //开始周期性触发事件计时
        key_value = cur_key; //返回当前键值
      } else if (KEY1 == cur_key) { //需要n秒后执行单次触发事件的按键,"0"为屏蔽此功能,即没有需要n秒后执行单次触发的按键
        key_data.lng_prs_mode = 2; //开始单次触发事件计时
        key_data.trig_period = -1; //不需要周期性触发
      }

      key_data.threshold = -1; //到达阈值,停止计数
    } else if (-1 == key_data.threshold) { //已经到达长按阈值
      if (1 == key_data.lng_prs_mode) { //continuous trig
        if (key_data.trig_period >= 10) {
          key_data.trig_period = 0;
          key_value = cur_key | LNG_PRS; //连续返回当前键值
//	  key_value = cur_key;
        }
      } else if (2 == key_data.lng_prs_mode) { //one-shot trig
        if (key_data.prs_second >= 2) {  //2+0.5(阈值) sec.
          key_data.prs_second = -1; //单次触发事件,不允许再次进入
          key_value = cur_key | LNG_PRS_3SEC;
        }
      }
    }
  } else if (old_key != 0 && 0 == cur_key && true == key_data.act_prs) { //has released the key
    if (0 == key_data.lng_prs_mode) { //除了长按单次触发事件,其他事件松开按键后都执行一次
      key_value = old_key;
    } else
      key_value = 0;

    key_data.act_prs = false;
    key_data.lng_prs_mode = 0;
  } else if (cur_key != 0 && old_key != 0 && cur_key != old_key) { //未松开按键的情况下又去按下其他按键
    key_value = 0;
    key_data.threshold = 0;
    key_data.trig_period = 0;
    key_data.prs_second = 0;
  } else { //无按键按下
    key_value = 0;
    key_data.lng_prs_mode = 0;
    key_data.threshold = 0;
    key_data.trig_period = 0;
    key_data.prs_second = 0;
  }

  old_key = cur_key;

  return key_value;
}

void KeyFuncHandle(uint16_t key_info) {
  switch (key_info) {
    case KEY1 | LNG_PRS_3SEC:
      //display current RS485 address
      ctrl_data.status = RS485_ADDR_STATUS;
      break;
    default:
      key_info = 0;
      key_data.active = false;
      break;
  }
}
