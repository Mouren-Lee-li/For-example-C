#include "uart.h"


void uart1_tx_byte(uint8_t data) {
  UART1_DR = data;
  while ((UART1_SR & 0x80) == 0x00);    // �ȴ����ݵĴ���
}


uint8_t UART1_recieve_byte(void) {

  return (uint8_t)UART1_DR;
}

void uart1_tx(uint8_t* pdata, uint8_t quantity) {
  for (uint8_t i = 0; i < quantity; i++) {
    uart1_tx_byte(pdata[i]);
  }
}

uint8_t uart_rx(void) {

  return UART1_recieve_byte();
}

void uart1_tx_str(uint8_t* str) {
  while (0 != *str) {
    uart1_tx_byte(*str);
    str++;
  }
}

void uart1_init(void) {
  UART1_BRR2 = 0x02;
  UART1_BRR1 = 0x68;                 //16MHZʱ������9600��Ӧ��ֵ
  UART1_CR2  = 0x2c;              //�������� ���ռ������ж�
}

void PrintData (T_RH_PARA* pdat ) {
  uart1_tx_str("�¶�: ");
  if ((uint8_t)(pdat->T)/100 != 0) {
    uart1_tx_byte((uint8_t)(pdat->T)/100 + 0x30);
  }
  uart1_tx_byte((uint8_t)(pdat->T)%100/10 + 0x30);
  uart1_tx_byte((uint8_t)(pdat->T)%10 + 0x30);
  uart1_tx_str("��   ");

  uart1_tx_str("ʪ��:");
  if ((uint8_t)(pdat->RH)/100 != 0) {
    uart1_tx_byte((uint8_t)(pdat->RH)/100 + 0x30);
  }
  uart1_tx_byte((uint8_t)(pdat->RH)%100/10 + 0x30);
  uart1_tx_byte((uint8_t)(pdat->RH)%10 + 0x30);
  uart1_tx_str("%RH");
  uart1_tx_str("\r\n");
}

uint8_t uart_status;
#pragma vector=UART1_R_RXNE_vector
__interrupt __root void UART1_Recv_IRQHandler(void) {
  uart_status = UART1_SR; //clean status,include error status
  if (((recv_buf.head + 1) % FIFO_DEEP) == recv_buf.tail) { //FIFOװ���������,���������ݼ���
    uart_rx(); //�����������buffer�е����ݷ����п��ܻ�������봮���ж�
  } else {
    recv_buf.data[recv_buf.head] = uart_rx();
    recv_buf.head = (recv_buf.head + 1) % FIFO_DEEP;
    recv_buf.num++;
  }
  recv_buf.timeout = 0;
}
