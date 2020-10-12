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
  CLK_CKDIVR &= (uint8_t)(~0x18); /*使能内部时钟*/
  CLK_CKDIVR |= (uint8_t)0x00; /*设置时钟为内部16M高速时钟*/
}

void IWDG_INIT(void) {
  IWDG_KR = 0xCC; //启动IWDG
  IWDG_KR = 0x55;        //解除 PR 及 RLR 的写保护
  IWDG_RLR = 0xFF; //看门狗计数器重装载数值,1.02s
  IWDG_PR = 0x06; //分频系数为256,最长超时时间为1.02S
  IWDG_KR = 0xAA; //刷新IDDG?避免产生看门狗复位?同时恢复 PR 及 RLR 的写保护状态
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
  TIM1_CR1_CEN = 1;   //启动计数器
}

/*T = TIM_ARR * (1 / (16M / (2^TIM2_PSCR))) */
void timer2_init(void) {
  TIM2_PSCR = 4;
  TIM2_ARRH = 0x27;         //10ms
  TIM2_ARRL = 0x10;
  TIM2_IER_UIE = 1;         //更新中断使能
  TIM2_CR1_ARPE = 1;            //预装载使能
  TIM2_CR1_CEN = 1;           //启动计数器
}

void timer4_init(void) {        //10us周期
  TIM4_CR1=0x01;//使能计数器
  TIM4_IER=0x01;//更新中断使能
  TIM4_EGR=0x01;
//TIM4_CNTR=250;//计数器值
  TIM4_ARR=0x05;//自动重装的值 0.002ms*5=10us
  TIM4_PSCR=0x05;//分频值        默认上电时是 16MHz/32=500000Hz  计数一下0.002ms
}

void GPIO_init(void) {
  PA_DDR |= BIT3; 
  PA_CR1 |= BIT3;
  PB_DDR |= BIT4 | BIT5;
  PB_CR1 |= BIT4 | BIT5;
  PC_DDR |= 0;
  PC_CR1 |= BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
  PC_CR2 |= BIT4 | BIT5 | BIT6 | BIT7; //薄码开关地址读取采用外部中断模式
  EXTI_CR1 |= 0x30; //配置PORTC为下降沿触发
                    //0bxx00xxxx:下降沿和低电平触发
                    //0bxx01xxxx:仅上升沿触发
                    //0bxx10xxxx:仅下降沿触发
                    //0bxx11xxxx:上升沿和下降沿触发
  PD_DDR |= BIT2 | BIT3 | BIT4;
  PD_CR1 |= BIT2 | BIT3 | BIT4;
}

//软件中断优先级设置函数
//vector:中断向量号(0~24)
//prio:优先级(1~3),禁止设置为0
//STM8的优先级分为软件优先级和硬件优先级,软件优先级优先于硬件优先级.
//硬件优先级由向量号确定,向量号越小,优先级越高.
//软件优先级可以通过本函数设置.
//STM8软件优先级设置可以分为4个等级(0~3),实际上可设置的就三个等级:1~3
//优先级顺序:0<1<2<3,3的优先级最高,高优先级的中断可以打断低优先级的中断
//多个中断同时发生:在软件优先级相同的情况下,由硬件优先级决定谁先响应.
void ITC_Set(uint8_t vector, uint8_t priority) {
  if (priority == 0) return;                  //不能设置为优先级0
  if (priority == 2) priority = 0;              //优先级2:00B
  if (vector < 4) {
    ITC_SPR1 &= ~(3 << vector * 2);         //清除原来的设置
    ITC_SPR1 |= priority << vector * 2;     //设置优先级
  } else if (vector < 8) {
    ITC_SPR2 &= ~(3 << (vector - 4) * 2);     //清除原来的设置
    ITC_SPR2 |= priority << (vector - 4) * 2; //设置优先级
  } else if (vector < 12) {
    ITC_SPR3 &= ~(3 << (vector - 8) * 2);     //清除原来的设置
    ITC_SPR3 |= priority << (vector - 8) * 2; //设置优先级
  } else if (vector < 16) {
    ITC_SPR4 &= ~(3 << (vector - 12) * 2);    //清除原来的设置
    ITC_SPR4 |= priority << (vector - 12) * 2; //设置优先级
  } else if (vector < 20) {
    ITC_SPR5 &= ~(3 << (vector - 16) * 2);    //清除原来的设置
    ITC_SPR5 |= priority << (vector - 16) * 2; //设置优先级
  } else if (vector < 24) {
    ITC_SPR6 &= ~(3 << (vector - 20) * 2);    //清除原来的设置
    ITC_SPR6 |= priority << (vector - 20) * 2; //设置优先级
  } else if (vector < 28) {
    ITC_SPR7 &= ~(3 << (vector - 24) * 2);    //清除原来的设置
    ITC_SPR7 |= priority << (vector - 24) * 2; //设置优先级
  } else if (vector < 32) {
    ITC_SPR8 &= ~(3 << (vector - 28) * 2);    //清除原来的设置
    ITC_SPR8 |= priority << (vector - 28) * 2; //设置优先级
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


