#include "initial.h"


void delay_us(uint16_t t) { //1us
  while (t--)
    for (uint16_t i = 0; i < 16; i++);
}

void delay_ms(uint16_t t) { //1ms
  while (t--)
    for (uint16_t i = 0; i < 600; i++);
}

void clk_config(void) {
  CLK_CKDIVR &= (uint8_t)(~0x18); /*ʹ���ڲ�ʱ��*/
  CLK_CKDIVR |= (uint8_t)0x00; /*����ʱ��Ϊ�ڲ�16M����ʱ��*/
}

void IWDG_INIT(void) {
  IWDG_KR = 0xCC; //����IWDG
  IWDG_KR = 0x55;        //��� PR �� RLR ��д����
  IWDG_RLR = 0xFF; //���Ź���������װ����ֵ,1.02s
  IWDG_PR = 0x06; //��Ƶϵ��Ϊ256,���ʱʱ��Ϊ1.02S
  IWDG_KR = 0xAA; //ˢ��IDDG?����������Ź���λ?ͬʱ�ָ� PR �� RLR ��д����״̬
}

/**
  * @brief  Initializes the WWDG peripheral.
  *         This function set Window Register = WindowValue, Counter Register
  *         according to Counter and \b ENABLE \b WWDG
  * @param  counter : WWDG counter value
  * @param  window_value : specifies the WWDG Window Register,range is 0x00 to 0x7F.
  * @retval None
  */
void WWDG_init(uint8_t counter, uint8_t window_value) {
  WWDG_WR = 0x7F;  //if not write this register, the system will reset when started
  WWDG_CR = BIT7 | BIT6 | counter;
  WWDG_WR = (~BIT7) & (BIT6 | window_value);
}

/**
  * @brief  Refreshes the WWDG or IWDGperipheral.
  * @param  Counter :  WWDG Counter Value or 0xAA
  *         This parameter must be a number between 0x40 and 0x7F.
  * @retval None
  */
void feed_dog(uint8_t counter) {
#if WWDG_ENABLE
  /* Write to T[6:0] bits to configure the counter value, no need to do
    a read-modify-write; writing a 0 to WDGA bit does nothing */
  WWDG_CR = (uint8_t)(counter & (uint8_t)0x7F);
#endif
#if IWDG_ENABLE
  /* Write 0xAA to IWDG_KR will refresh IWDG */
  IWDG_KR = 0xAA;
#endif
}

/**
  * @brief  Generates immediate WWDG RESET.
  * @param  None
  * @retval None
  */
void WWDG_SWReset(void) {
  WWDG_CR = 0x80; /* Activate WWDG, with clearing T6 */
}

/*T = TIM_ARR * (1 / (16M / (TIM_PSC + 1))) */
void timer1_init(void) {
  TIM1_PSCRH = 0x00;
  TIM1_PSCRL = 0x0f;         //f = 16M/(0x0f + 1)=1M
  TIM1_ARRH  = 0x03;            //0x03e8 * (1 / f) = 1s
  TIM1_ARRL  = 0xe8;
  TIM1_IER_UIE = 1;
  TIM1_CR1_CEN = 1;   //����������
}

/*T = TIM_ARR * (1 / (16M / (2^TIM2_PSCR))) */
void timer2_init(void) {
  TIM2_PSCR = 4;
  TIM2_ARRH = 0x27;         //10ms
  TIM2_ARRL = 0x10;
  TIM2_IER_UIE = 1;         //�����ж�ʹ��
  TIM2_CR1_ARPE = 1;            //Ԥװ��ʹ��
  TIM2_CR1_CEN = 1;           //����������
}

void timer4_init(void) {        //10us����
  TIM4_CR1=0x01;//ʹ�ܼ�����
  TIM4_IER=0x01;//�����ж�ʹ��
  TIM4_EGR=0x01;
//TIM4_CNTR=250;//������ֵ
  TIM4_ARR=0x05;//�Զ���װ��ֵ 0.002ms*5=10us
  TIM4_PSCR=0x05;//��Ƶֵ        Ĭ���ϵ�ʱ�� 16MHz/32=500000Hz  ����һ��0.002ms
}

void GPIO_init(void) {
  PA_DDR |= BIT3; 
  PA_CR1 |= BIT3;
  PB_DDR |= BIT4 | BIT5;
  PB_CR1 |= BIT4 | BIT5;
  PC_DDR |= 0;
  PC_CR1 |= BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
  PC_CR2 |= BIT4 | BIT5 | BIT6 | BIT7; //���뿪�ص�ַ��ȡ�����ⲿ�ж�ģʽ
  EXTI_CR1 |= 0x30; //����PORTCΪ�½��ش���
                    //0bxx00xxxx:�½��غ͵͵�ƽ����
                    //0bxx01xxxx:�������ش���
                    //0bxx10xxxx:���½��ش���
                    //0bxx11xxxx:�����غ��½��ش���
  PD_DDR |= BIT2 | BIT3 | BIT4;
  PD_CR1 |= BIT2 | BIT3 | BIT4;
}

//����ж����ȼ����ú���
//vector:�ж�������(0~24)
//prio:���ȼ�(1~3),��ֹ����Ϊ0
//STM8�����ȼ���Ϊ������ȼ���Ӳ�����ȼ�,������ȼ�������Ӳ�����ȼ�.
//Ӳ�����ȼ���������ȷ��,������ԽС,���ȼ�Խ��.
//������ȼ�����ͨ������������.
//STM8������ȼ����ÿ��Է�Ϊ4���ȼ�(0~3),ʵ���Ͽ����õľ������ȼ�:1~3
//���ȼ�˳��:0<1<2<3,3�����ȼ����,�����ȼ����жϿ��Դ�ϵ����ȼ����ж�
//����ж�ͬʱ����:��������ȼ���ͬ�������,��Ӳ�����ȼ�����˭����Ӧ.
void ITC_Set(uint8_t vector, uint8_t priority) {
  if (priority == 0) return;                  //��������Ϊ���ȼ�0
  if (priority == 2) priority = 0;              //���ȼ�2:00B
  if (vector < 4) {
    ITC_SPR1 &= ~(3 << vector * 2);         //���ԭ��������
    ITC_SPR1 |= priority << vector * 2;     //�������ȼ�
  } else if (vector < 8) {
    ITC_SPR2 &= ~(3 << (vector - 4) * 2);     //���ԭ��������
    ITC_SPR2 |= priority << (vector - 4) * 2; //�������ȼ�
  } else if (vector < 12) {
    ITC_SPR3 &= ~(3 << (vector - 8) * 2);     //���ԭ��������
    ITC_SPR3 |= priority << (vector - 8) * 2; //�������ȼ�
  } else if (vector < 16) {
    ITC_SPR4 &= ~(3 << (vector - 12) * 2);    //���ԭ��������
    ITC_SPR4 |= priority << (vector - 12) * 2; //�������ȼ�
  } else if (vector < 20) {
    ITC_SPR5 &= ~(3 << (vector - 16) * 2);    //���ԭ��������
    ITC_SPR5 |= priority << (vector - 16) * 2; //�������ȼ�
  } else if (vector < 24) {
    ITC_SPR6 &= ~(3 << (vector - 20) * 2);    //���ԭ��������
    ITC_SPR6 |= priority << (vector - 20) * 2; //�������ȼ�
  } else if (vector < 28) {
    ITC_SPR7 &= ~(3 << (vector - 24) * 2);    //���ԭ��������
    ITC_SPR7 |= priority << (vector - 24) * 2; //�������ȼ�
  } else if (vector < 32) {
    ITC_SPR8 &= ~(3 << (vector - 28) * 2);    //���ԭ��������
    ITC_SPR8 |= priority << (vector - 28) * 2; //�������ȼ�
  }
}

void chip_system_init(void) {
  clk_config();
  ITC_Set(UART1_R_RXNE_vector, 2);
  ITC_Set(TIM1_OVR_UIF_vector, 1);
  ITC_Set(TIM2_OVR_UIF_vector, 1);
  GPIO_init();
  timer1_init();
  timer2_init();
//timer4_init();
  uart1_init();
}


