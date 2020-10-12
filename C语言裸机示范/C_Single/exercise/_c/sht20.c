#include "sht20.h"

//初始化IIC
void IIC_init(void) {
  //已在GPIO_Init中配置
}
//产生IIC起始信号
void IIC_Start(void) {
  SDA_OUT(); //sda线输出
  IIC_SDA = 1;
  IIC_SCL = 1;
  delay_us(4);
  IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
  delay_us(4);
  IIC_SCL = 0; //钳住I2C总线?准备发送或接收数据
}
//产生IIC停止信号
void IIC_Stop(void) {
  SDA_OUT(); //sda线输出
  IIC_SCL = 0;
  IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
  delay_us(4);
  IIC_SCL = 1;
  IIC_SDA = 1; //发送I2C总线结束信号
  delay_us(4);
}
//等待应答信号到来
//返回值?1?接收应答失败
//        0?接收应答成功
uint8_t IIC_Wait_Ack(void) {
  uint8_t ucErrTime = 0;
  SDA_IN();      //SDA设置为输入
  IIC_SDA = 1;
  delay_us(1);
  IIC_SCL = 1;
  delay_us(1);
  while (READ_SDA) {
    ucErrTime++;
    if (ucErrTime > 250) {
      IIC_Stop();
      return 1;
    }
  }
  IIC_SCL = 0; //时钟输出0
  return 0;
}
//产生ACK应答
void IIC_Ack(void) {
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 0;
  delay_us(2);
  IIC_SCL = 1;
  delay_us(2);
  IIC_SCL = 0;
}
//不产生ACK应答
void IIC_NAck(void) {
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 1;
  delay_us(2);
  IIC_SCL = 1;
  delay_us(2);
  IIC_SCL = 0;
}
//IIC发送一个字节
//返回从机有无应答
//1?有应答
//0?无应答
void IIC_Send_Byte(uint8_t txd) {
  uint8_t t;
  SDA_OUT();
  IIC_SCL = 0; //拉低时钟开始数据传输
  for (t = 0; t < 8; t++) {
    IIC_SDA = (txd & 0x80) >> 7;
    txd <<= 1;
    delay_us(2);   //对TEA5767这三个延时都是必须的
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
    delay_us(2);
  }
}
//读1个字节?ack=1时?发送ACK?ack=0?发送nACK
uint8_t IIC_Read_Byte(unsigned char ack) {
  unsigned char i, receive = 0;
  SDA_IN(); //SDA设置为输入
  for (i = 0; i < 8; i++) {
    IIC_SCL = 0;
    delay_us(2);
    IIC_SCL = 1;
    receive <<= 1;
    if (READ_SDA) receive++;
    delay_us(1);
  }
  if (!ack) IIC_NAck();    //发送nACK
  else
    IIC_Ack();            //发送ACK
  return receive;
}

void RDA5807FP_write(uint8_t reg_addr, uint16_t data) {

  IIC_Start();                              //产生起始位
  IIC_Send_Byte(0x22);   //地址为0010001x?读时x为1?写时x为0
  IIC_Wait_Ack();


  IIC_Send_Byte(reg_addr);                     //芯片寄存器地址
  IIC_Wait_Ack();

  IIC_Send_Byte(data >> 8);
  IIC_Wait_Ack();
  IIC_Send_Byte(data & 0xff);
  IIC_Wait_Ack();

  IIC_Stop();
}

uint16_t RDA5807FP_read(uint8_t reg) {
  uint16_t data;

  IIC_Start();                              //产生起始位
  IIC_Send_Byte(0x22);   //地址为0010001x?读时x为1?写时x为0

  IIC_Wait_Ack();

  IIC_Send_Byte(reg);                     //芯片寄存器地址
  IIC_Wait_Ack();

  IIC_Start();                              //产生起始位

  IIC_Send_Byte(0x23);   //地址为0010001x?读时x为1?写时x为0
  IIC_Wait_Ack();

  data = IIC_Read_Byte(1);

  data = (data << 8) | IIC_Read_Byte(0);

  IIC_Stop();               //产生停止位


  return data;                      //返回数据
}

// CRC
const uint16_t POLYNOMIAL = 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001

// calculates checksum for n bytes of data and compares it with expected
// checksum
// input: data[] checksum is built based on this data
// nbrOfBytes checksum is built for n bytes of data
// checksum expected checksum
// return: error: CHECKSUM_ERROR = checksum does not match
// 0 = checksum matches
uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum) {
  uint8_t crc = 0;
  uint8_t byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr) { 
    crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit) { 
      if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else
        crc = (crc << 1);
    }
  }
  if (crc != checksum) 
    return 1;
  else
    return 0;
}

uint8_t SHT2x_ReadUserRegister(uint8_t* pRegisterValue) {
  uint8_t checksum; //variable for checksum byte
  uint8_t error = 0; //variable for error code

  IIC_Start();
  IIC_Send_Byte(I2C_ADR_W);
  error = IIC_Wait_Ack();
  IIC_Send_Byte(USER_REG_R);
  error = IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_R);
  error = IIC_Wait_Ack();
  *pRegisterValue = IIC_Read_Byte(1);
  checksum = IIC_Read_Byte(0);
  SHT2x_CheckCrc(pRegisterValue, 1, checksum);
  error = IIC_Wait_Ack();
  IIC_Stop();

  return error;
}

uint8_t SHT2x_WriteUserRegister(uint8_t* pRegisterValue) {
  uint8_t error = 0; //variable for error code

  IIC_Start();
  IIC_Send_Byte(I2C_ADR_W);
  error = IIC_Wait_Ack();
  IIC_Send_Byte(USER_REG_W);
  error = IIC_Wait_Ack();
  IIC_Send_Byte(*pRegisterValue);
  error = IIC_Wait_Ack();
  IIC_Stop();

  return error;
}

uint8_t SHT2x_SoftReset(void) {
  uint8_t error = 0; //error variable

  IIC_Start();
  IIC_Send_Byte(I2C_ADR_W); // I2C Adr
  error = IIC_Wait_Ack();
  IIC_Send_Byte(SOFT_RESET); // Command
  error = IIC_Wait_Ack();
  IIC_Stop();

  return error;
}

static float SHT2x_CalcRH(uint16_t u16sRH) {
  float humidityRH; // variable for result
  u16sRH &= ~0x0003; // clear bits [1..0] (status bits)
                     //-- calculate relative humidity [%RH] --
  humidityRH = -6.0 + 125.0 / 65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}

static float SHT2x_CalcTemperatureC(uint16_t u16sT) {
  float temperatureC; // variable for result
  u16sT &= ~0x0003; // clear bits [1..0] (status bits)
                    //-- calculate temperature [°C] --
  temperatureC = -46.85 + 175.72 * (float)u16sT / 65536; //T= -46.85 + 175.72 * ST/2^16
  return temperatureC;
}

uint8_t SHT2x_MeasureHM(MEASURE_TYPE eSHT2xMeasureType, T_RH_PARA* pT_RH_PARA) {
  uint8_t checksum; //checksum
  uint8_t data[2]; //data array for checksum verification
  uint8_t error = 0; //error variable
  uint16_t i; //counting variable
  uint16_t val = 0;

  //-- write I2C sensor address and command --
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_W); // I2C Adr
  error = IIC_Wait_Ack();
  switch (eSHT2xMeasureType) {
    case HUMIDITY:
      IIC_Send_Byte(TRIG_RH_MEASUREMENT_HM);
      error = IIC_Wait_Ack();
      break;
    case TEMP :
      IIC_Send_Byte(TRIG_T_MEASUREMENT_HM);
      error = IIC_Wait_Ack();
      break;
    default:
      assert(0);
  }
  delay_us(10);
  //-- wait until hold master is released --
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_R);
  error = IIC_Wait_Ack();
  SDA_IN();
//delay_us(5000);
//SCL_IN(); //采用Hold Master Mode:传感器采集温湿度过程中将SCL强行拉低,采集完成立即再拉高
  delay_us(10);
  for (i = 0; i < 1000; i++) { 
    delay_us(10); // a timeout (~1s) is reached // wait until master hold is released or
    if (0 == READ_SDA)
      break;
  }
  //-- check for timeout --
  if (0 == READ_SDA)
    error = 1;
  SDA_OUT();
//SCL_OUT(); //恢复SCL方向,以进行下一步读取
  //-- read two data bytes and one checksum byte --
  val = data[0] = IIC_Read_Byte(1);
  delay_us(10);
  data[1] = IIC_Read_Byte(1);
  delay_us(10);
  val = (val << 8) | data[1];
  checksum = IIC_Read_Byte(0);
  //-- verify checksum --
  error = SHT2x_CheckCrc(data, 2, checksum);
  IIC_Stop();

  switch (eSHT2xMeasureType) {
    case HUMIDITY:
      pT_RH_PARA->RH = SHT2x_CalcRH(val);
      break;
    case TEMP :
      pT_RH_PARA->T = SHT2x_CalcTemperatureC(val);
      break;
    default:
      assert(0);
  }

  return error;
}

uint8_t SHT2x_MeasurePoll(MEASURE_TYPE eSHT2xMeasureType, T_RH_PARA* pT_RH_PARA) {
  uint8_t checksum; //checksum
  uint8_t data[2]; //data array for checksum verification
  uint8_t error = 0; //error variable
  uint16_t i = 0; //counting variable
  uint16_t val = 0;

  //-- write I2C sensor address and command --
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_W); // I2C Adr
  error = IIC_Wait_Ack();
  switch (eSHT2xMeasureType) {
    case HUMIDITY:
      IIC_Send_Byte(TRIG_RH_MEASUREMENT_POLL);
      error = IIC_Wait_Ack();
      break;
    case TEMP :
      IIC_Send_Byte(TRIG_T_MEASUREMENT_POLL);
      error = IIC_Wait_Ack();
      break;
    default:
      assert(0);
  }
  //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
  do {
    IIC_Start();
    delay_us(100); //delay 10ms
    if (i++ >= 20) 
      break;
    IIC_Send_Byte(I2C_ADR_R);
  } while (IIC_Wait_Ack() == 1);
  if (i >= 20) 
    error = 1;
  //-- read two data bytes and one checksum byte --
  val = data[0] = IIC_Read_Byte(1);
  data[1] = IIC_Read_Byte(1);
  val = (val << 8) | data[1];
  checksum = IIC_Read_Byte(0);
  //-- verify checksum --
  error |= SHT2x_CheckCrc(data, 2, checksum);
  IIC_Stop();

  switch (eSHT2xMeasureType) {
    case HUMIDITY:
      pT_RH_PARA->RH = SHT2x_CalcRH(val);
      break;
    case TEMP :
      pT_RH_PARA->T = SHT2x_CalcTemperatureC(val);
      break;
    default:
      assert(0);
  }

  return error;
}

uint8_t SHT2x_GetSerialNumber(uint8_t u8SerialNumber[]) {
  uint8_t error = 0; //error variable

  //Read from memory location 1
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_W); //I2C address
  error = IIC_Wait_Ack();
  IIC_Send_Byte(0xFA); //Command for readout on-chip memory
  error = IIC_Wait_Ack();
  IIC_Send_Byte(0x0F); //on-chip memory address
  error = IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_R); //I2C address
  error = IIC_Wait_Ack();
  u8SerialNumber[5] = IIC_Read_Byte(1); //Read SNB_3
  IIC_Read_Byte(1); //Read CRC SNB_3 (CRC is not analyzed)
  u8SerialNumber[4] = IIC_Read_Byte(1); //Read SNB_2
  IIC_Read_Byte(1); //Read CRC SNB_2 (CRC is not analyzed)
  u8SerialNumber[3] = IIC_Read_Byte(1); //Read SNB_1
  IIC_Read_Byte(1); //Read CRC SNB_1 (CRC is not analyzed)
  u8SerialNumber[2] = IIC_Read_Byte(1); //Read SNB_0
  IIC_Read_Byte(0); //Read CRC SNB_0 (CRC is not analyzed)
  IIC_Stop();

  //Read from memory location 2
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_W); //I2C address
  error = IIC_Wait_Ack();
  IIC_Send_Byte(0xFC); //Command for readout on-chip memory
  error = IIC_Wait_Ack();
  IIC_Send_Byte(0xC9); //on-chip memory address
  error = IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(I2C_ADR_R); //I2C address
  error = IIC_Wait_Ack();
  u8SerialNumber[1] = IIC_Read_Byte(1); //Read SNC_1
  u8SerialNumber[0] = IIC_Read_Byte(1); //Read SNC_0
  IIC_Read_Byte(1); //Read CRC SNC0/1 (CRC is not analyzed)
  u8SerialNumber[7] = IIC_Read_Byte(1); //Read SNA_1
  u8SerialNumber[6] = IIC_Read_Byte(1); //Read SNA_0
  IIC_Read_Byte(0); //Read CRC SNA0/1 (CRC is not analyzed)
  IIC_Stop();

  return error;
}

uint8_t userRegister;
uint8_t SerialNumber_SHT2x[8];
void SHT2X_Init(void) {
  // --- Reset sensor by command ---
  SHT2x_SoftReset();
  delay_us(1500); // wait till sensor has restarted
  // --- Read the sensors serial number (64bit) ---
  SHT2x_GetSerialNumber(SerialNumber_SHT2x);
  // --- Set Resolution e.g. RH 10bit, Temp 13bit ---
  SHT2x_ReadUserRegister(&userRegister); //get actual user reg
  userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
  SHT2x_WriteUserRegister(&userRegister); //write changed user reg
}

////==============================================================================
//uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
////==============================================================================
//{
//  uint8_t crc = 0;
//  uint8_t byteCtr;
//  //calculates 8-Bit checksum with given polynomial
//  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr) { crc ^= (data[byteCtr]);
//    for (uint8_t bit = 8; bit > 0; --bit) { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
//      else
//        crc = (crc << 1);
//    }
//  }
//  if (crc != checksum) return CHECKSUM_ERROR;
//  else
//    return 0;
//}
////===========================================================================
//uint8_t SHT2x_ReadUserRegister(uint8_t* pRegisterValue)
////===========================================================================
//{
//  uint8_t checksum; //variable for checksum byte
//  uint8_t error = 0; //variable for error code
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_W);
//  error |= IIC_Send_Byte(USER_REG_R);
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_R);
//  *pRegisterValue = IIC_Read_Byte(1);
//  checksum = IIC_Read_Byte(0);
//  error |= SHT2x_CheckCrc(pRegisterValue, 1, checksum);
//  IIC_Stop();
//  return error;
//}
////===========================================================================
//uint8_t SHT2x_WriteUserRegister(uint8_t* pRegisterValue)
////===========================================================================
//{
//  uint8_t error = 0; //variable for error code
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_W);
//  error |= IIC_Send_Byte(USER_REG_W);
//  error |= IIC_Send_Byte(*pRegisterValue);
//  IIC_Stop();
//  return error;
//}
////===========================================================================
//uint8_t SHT2x_MeasureHM(MEASURE_TYPE eSHT2xMeasureType, nt16* pMeasurand)
////===========================================================================
//{
//  uint8_t checksum; //checksum
//  uint8_t data[2]; //data array for checksum verification
//  uint8_t error = 0; //error variable
//  u16t i; //counting variable
//          //-- write I2C sensor address and command --
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_W); // I2C Adr
//    switch (eSHT2xMeasureType) {case HUMIDITY:
//      error |= IIC_Send_Byte(TRIG_RH_MEASUREMENT_HM);
//      break;
//    case TEMP :
//      error |= IIC_Send_Byte(TRIG_T_MEASUREMENT_HM);
//      break;
//    default:
//      assert(0);
//  }
//  //-- wait until hold master is released --
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_R);
//  SCL = HIGH; // set SCL I/O port as input
//  for (i = 0; i < 1000; i++){ DelayMicroSeconds(1000); // a timeout (~1s) is reached // wait until master hold is released or
//    if (SCL_CONF == 1) break;
//  }
//  //-- check for timeout --
//  if (SCL_CONF == 0) error |= TIME_OUT_ERROR;
//  //-- read two data bytes and one checksum byte --
//  pMeasurand->s16.u8H = data[0] = IIC_Read_Byte(1);
//  pMeasurand->s16.u8L = data[1] = IIC_Read_Byte(1);
//  checksum = IIC_Read_Byte(0);
//  //-- verify checksum --
//  error |= SHT2x_CheckCrc(data, 2, checksum);
//  IIC_Stop();
//  return error;
//}
////===========================================================================
//uint8_t SHT2x_MeasurePoll(MEASURE_TYPE eSHT2xMeasureType, nt16* pMeasurand)
////===========================================================================
//{
//  uint8_t checksum; //checksum
//  uint8_t data[2]; //data array for checksum verification
//  uint8_t error = 0; //error variable
//  u16t i = 0; //counting variable
//              //-- write I2C sensor address and command --
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_W); // I2C Adr
//    switch (eSHT2xMeasureType) {case HUMIDITY:
//      error |= IIC_Send_Byte(TRIG_RH_MEASUREMENT_POLL);
//      break;
//    case TEMP :
//      error |= IIC_Send_Byte(TRIG_T_MEASUREMENT_POLL);
//      break;
//    default:
//      assert(0);
//  }
//  //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
//  do {IIC_Start();
//    DelayMicroSeconds(10000); //delay 10ms
//    if (i++ >= 20) break;
//  } while (IIC_Send_Byte(I2C_ADR_R) == ACK_ERROR);
//  if (i >= 20) error |= TIME_OUT_ERROR;
//  //-- read two data bytes and one checksum byte --
//  pMeasurand->s16.u8H = data[0] = IIC_Read_Byte(1);
//  pMeasurand->s16.u8L = data[1] = IIC_Read_Byte(1);
//  checksum = IIC_Read_Byte(0);
//  //-- verify checksum --
//  error |= SHT2x_CheckCrc(data, 2, checksum);
//  IIC_Stop();
//  return error;
//}
////===========================================================================
//uint8_t SHT2x_SoftReset()
////===========================================================================
//{
//  uint8_t error = 0; //error variable
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_W); // I2C Adr
//  error |= IIC_Send_Byte(SOFT_RESET); // Command
//  IIC_Stop();
//  DelayMicroSeconds(15000); // wait till sensor has restarted
//  return error;
//}
////==============================================================================
//float SHT2x_CalcRH(u16t u16sRH)
////==============================================================================
//{
//  ft humidityRH; // variable for result
//  u16sRH &= ~0x0003; // clear bits [1..0] (status bits)
//                     //-- calculate relative humidity [%RH] --
//  humidityRH = -6.0 + 125.0 / 65536 * (ft)u16sRH; // RH= -6 + 125 * SRH/2^16
//  return humidityRH;
//}
////==============================================================================
//float SHT2x_CalcTemperatureC(u16t u16sT)
////==============================================================================
//{
//  ft temperatureC; // variable for result
//  u16sT &= ~0x0003; // clear bits [1..0] (status bits)
//                    //-- calculate temperature [°C] --
//  temperatureC = -46.85 + 175.72 / 65536 * (ft)u16sT; //T= -46.85 + 175.72 * ST/2^16
//  return temperatureC;
//}
////==============================================================================
//uint8_t SHT2x_GetSerialNumber(uint8_t u8SerialNumber[])
////==============================================================================
//{
//  uint8_t error = 0; //error variable
//                 //Read from memory location 1
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_W); //I2C address
//  error |= IIC_Send_Byte(0xFA); //Command for readout on-chip memory
//  error |= IIC_Send_Byte(0x0F); //on-chip memory address
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_R); //I2C address
//  u8SerialNumber[5] = IIC_Read_Byte(1); //Read SNB_3
//  IIC_Read_Byte(1); //Read CRC SNB_3 (CRC is not analyzed)
//  u8SerialNumber[4] = IIC_Read_Byte(1); //Read SNB_2
//  IIC_Read_Byte(1); //Read CRC SNB_2 (CRC is not analyzed)
//  u8SerialNumber[3] = IIC_Read_Byte(1); //Read SNB_1
//  IIC_Read_Byte(1); //Read CRC SNB_1 (CRC is not analyzed)
//  u8SerialNumber[2] = IIC_Read_Byte(1); //Read SNB_0
//  IIC_Read_Byte(0); //Read CRC SNB_0 (CRC is not analyzed)
//  IIC_Stop();
//  //Read from memory location 2
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_W); //I2C address
//  error |= IIC_Send_Byte(0xFC); //Command for readout on-chip memory
//  error |= IIC_Send_Byte(0xC9); //on-chip memory address
//  IIC_Start();
//  error |= IIC_Send_Byte(I2C_ADR_R); //I2C address
//  u8SerialNumber[1] = IIC_Read_Byte(1); //Read SNC_1
//  u8SerialNumber[0] = IIC_Read_Byte(1); //Read SNC_0
//  IIC_Read_Byte(1); //Read CRC SNC0/1 (CRC is not analyzed)
//  u8SerialNumber[7] = IIC_Read_Byte(1); //Read SNA_1
//  u8SerialNumber[6] = IIC_Read_Byte(1); //Read SNA_0
//  IIC_Read_Byte(0); //Read CRC SNA0/1 (CRC is not analyzed)
//  IIC_Stop();
//  return error;
//}
