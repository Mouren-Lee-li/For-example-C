#include "modbus.h"

uint16_t CalcCRCModBus(uint8_t* pbuf, uint8_t len) {
  uint8_t check = 0;
  uint16_t crc = 0xffff;

  for (uint8_t i = 0; i < len; i++) {
    crc = crc ^ *pbuf;

    for (uint8_t i = 0; i < 8; i++) {
      check = crc & 1;
      crc = crc >> 1;
      crc = crc & 0x7fff;

      if (check == 1) {
        crc = crc ^ 0xa001;
      }
      crc = crc & 0xffff;
    }

    pbuf++;
  }

  return crc;
}

/** 
 * ��FIFO�л�ȡһ֡���� 
 * @author Administrator (2017-04-14)
 * 
 * @param pFIFO_dat FIFO����
 * 
 * @return uint8_t ����1Ϊ��Ч֡,����0Ϊ��Ч֡
 */
  uint16_t crc = 0;
uint8_t FrameGet(FIFO_DATA* pFIFO_dat) {

  memset(mod_data.frame, 0, FRAME_LEN); //������ݻ�����
  if (pFIFO_dat->data[pFIFO_dat->tail] == mod_data.addr) { //������ַΪ0x01
    for (uint8_t len = 0; len < FRAME_LEN; len++) {
      mod_data.frame[len] = pFIFO_dat->data[pFIFO_dat->tail];
      pFIFO_dat->data[pFIFO_dat->tail] = 0;
      if (0 == pFIFO_dat->num) //�Ѿ�ȡ��
        return 1;
      else
        pFIFO_dat->num--; //FIFO������ȡ��һ��
      pFIFO_dat->tail = (pFIFO_dat->tail + 1) % FIFO_DEEP;
    }
  } else {
    pFIFO_dat->data[pFIFO_dat->tail] = 0;
    pFIFO_dat->tail = (pFIFO_dat->tail + 1) % FIFO_DEEP;
    if (pFIFO_dat->num != 0) //�Ѿ�ȡ��
      pFIFO_dat->num--; //FIFO������ȡ��һ����������
    return 1;
  }

  crc = CalcCRCModBus(mod_data.frame, FRAME_LEN - 2);
  if ((crc / 256 != mod_data.frame[MOD_CRCH]) || (crc % 256 != mod_data.frame[MOD_CRCL]))
    return 1;
  
  return 0;
}

const uint8_t FRAME_1[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b}; //����ʪ��
const uint8_t FRAME_2[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xd5, 0xca}; //�����¶�

void FrameProcess(MODBUS_PARA* pmod_data, T_RH_PARA* pT_RH_dat) {
  uint16_t temp, humidity;
  uint8_t tx_buf[10];
  uint8_t len = 0; //������CRC�����ֽ�
  uint16_t crc = 0;

  humidity = (uint16_t)(10*pT_RH_dat->RH);
  if (pT_RH_dat->T < 0) {
    temp = 0;
  } else {
    temp = (uint16_t)(10 * pT_RH_dat->T);
  }
  if (0x00 == pmod_data->frame[MOD_REGL]) { //�����0x00�Ĵ�����ʼ��ȡ
    tx_buf[0] = pmod_data->addr;
    tx_buf[1] = 0x03;
    tx_buf[2] = 0x04;
    tx_buf[3] = (uint8_t)(humidity/256);
    tx_buf[4] = (uint8_t)(humidity%256);
    tx_buf[5] = (uint8_t)(temp/256);
    tx_buf[6] = (uint8_t)(temp%256);
    len = 7;
    crc = CalcCRCModBus(tx_buf, len);
    tx_buf[7] = crc % 256;
    tx_buf[8] = crc / 256;
    uart1_tx(tx_buf, len+2);
  } else if (0x01 == pmod_data->frame[MOD_REGL]) { //�����0x01�Ĵ�����ʼ��ȡ
    tx_buf[0] = pmod_data->addr;
    tx_buf[1] = 0x03;
    tx_buf[2] = 0x02;
    tx_buf[3] = (uint8_t)(temp/256);
    tx_buf[4] = (uint8_t)(temp%256);
    len = 5;
    crc = CalcCRCModBus(tx_buf, len);
    tx_buf[5] = crc % 256;
    tx_buf[6] = crc / 256;
    uart1_tx(tx_buf, len+2);
  }
}

void GetRS485Addr(uint8_t* addr) {
  *addr = (0x01 & ~ADDR4);
  *addr = (mod_data.addr << 1) | (0x01 & ~ADDR3);
  *addr = (mod_data.addr << 1) | (0x01 & ~ADDR2);
  *addr = (mod_data.addr << 1) | (0x01 & ~ADDR1);
}
