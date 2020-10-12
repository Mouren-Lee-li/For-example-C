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

    if (key_data.threshold > 50) { //����0.5s�󼤻��������һ��
      if (0) { //��Ҫ�����Դ����İ���
        key_data.lng_prs_mode = 1; //��ʼ�����Դ����¼���ʱ
        key_value = cur_key; //���ص�ǰ��ֵ
      } else if (KEY1 == cur_key) { //��Ҫn���ִ�е��δ����¼��İ���,"0"Ϊ���δ˹���,��û����Ҫn���ִ�е��δ����İ���
        key_data.lng_prs_mode = 2; //��ʼ���δ����¼���ʱ
        key_data.trig_period = -1; //����Ҫ�����Դ���
      }

      key_data.threshold = -1; //������ֵ,ֹͣ����
    } else if (-1 == key_data.threshold) { //�Ѿ����ﳤ����ֵ
      if (1 == key_data.lng_prs_mode) { //continuous trig
        if (key_data.trig_period >= 10) {
          key_data.trig_period = 0;
          key_value = cur_key | LNG_PRS; //�������ص�ǰ��ֵ
//	  key_value = cur_key;
        }
      } else if (2 == key_data.lng_prs_mode) { //one-shot trig
        if (key_data.prs_second >= 2) {  //2+0.5(��ֵ) sec.
          key_data.prs_second = -1; //���δ����¼�,�������ٴν���
          key_value = cur_key | LNG_PRS_3SEC;
        }
      }
    }
  } else if (old_key != 0 && 0 == cur_key && true == key_data.act_prs) { //has released the key
    if (0 == key_data.lng_prs_mode) { //���˳������δ����¼�,�����¼��ɿ�������ִ��һ��
      key_value = old_key;
    } else
      key_value = 0;

    key_data.act_prs = false;
    key_data.lng_prs_mode = 0;
  } else if (cur_key != 0 && old_key != 0 && cur_key != old_key) { //δ�ɿ��������������ȥ������������
    key_value = 0;
    key_data.threshold = 0;
    key_data.trig_period = 0;
    key_data.prs_second = 0;
  } else { //�ް�������
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
