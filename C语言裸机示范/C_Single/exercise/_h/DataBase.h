#ifndef DATABASE_H
#define DATABASE_H

#include "iostm8s003f3.h"
#include "string.h" /* memcpy memset*/
#include "intrinsics.h"/*总中断头文件*/
#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
#include "assert.h"
#include "float.h"
#include "math.h"

#define BIT0          0x01
#define BIT1          0x02
#define BIT2          0x04
#define BIT3          0x08
#define BIT4          0x10
#define BIT5          0x20
#define BIT6          0x40
#define BIT7          0x80


//GPIO define
#define     TM1621_CS                 PA_ODR_bit.ODR3 
#define     IIC_SCL                   PB_ODR_bit.ODR4 
#define     READ_SCL                  PB_IDR_bit.IDR4 
#define     IIC_SDA                   PB_ODR_bit.ODR5
#define     READ_SDA                  PB_IDR_bit.IDR5
#define     KEY_IN                    PC_IDR_bit.IDR3   
#define     ADDR1                     PC_IDR_bit.IDR4
#define     ADDR2                     PC_IDR_bit.IDR5    
#define     ADDR3                     PC_IDR_bit.IDR6    
#define     ADDR4                     PC_IDR_bit.IDR7    
#define     TM1621_DATA               PD_ODR_bit.ODR2    
#define     TM1621_WR                 PD_ODR_bit.ODR3      
#define     RS485_DIR                 PD_ODR_bit.ODR4    
//IO direct
#define     SDA_IN()                  {PB_DDR &= ~BIT5;}
#define     SDA_OUT()                 {PB_DDR |= BIT5;}
#define     SCL_IN()                  {PB_DDR &= ~BIT4;}
#define     SCL_OUT()                 {PB_DDR |= BIT4;}

typedef enum {
  NORMAL_RUN_STATUS,
  RS485_ADDR_STATUS,
  INVALID_STATUS,
}STATUS;

typedef struct { 
  bool system_run; //0:not run. 1:running. 2:is stopping and will be 0(not run) next moment
  STATUS status; //system status
}CTRL_PARA;

typedef struct {
  uint8_t cursor; //cursor position when setting. 0:not in setting mode now. 
                  //1:temperature positon. 2:time position. 3:extra temperature position
                  //4.both of 2 channel temperature positon blink at the same time 
  uint8_t blink_bit; //1:lit.
}DISPLAY_PARA;

typedef struct {
  uint8_t eeprom_temp; //for record targ_temp setted by user last time
  uint8_t eeprom_min; //for record targ_time setted by user last time
}EEPROM_PARA;

typedef struct {
  uint8_t targ_min;
  uint8_t targ_sec;
  uint8_t time_out; //The countdown to the end.0:counting down 1:to the end 2:NA.
  uint8_t no_limit; //0: normal run 1:all day run
}TIME_PARA;

typedef struct {
  bool get_value;
  float T;
  float RH; //relative humidity
}T_RH_PARA;


typedef struct {
  uint16_t cnt_scan; //count key scan
  uint8_t lng_prs_mode; //0: not in long press mode. 1:to precess keys which need to trig continuous. 2:to precess keys which only need to trig one-shot
  int16_t threshold; //-1:in long press mode and no need to count. 100:reach the threshold time of active long press mode
  int16_t trig_period; //tirg period of long press. -1:lng_prs_mode==2, no need to count.
//  uint16_t prs_hld_cnt; //start to count when some key is pressed
  int16_t prs_second; //the whole time(second) from long press to release. -1:NA. no need to count
  bool act_prs; //true:some key being pressed now
  bool active; //true:some key be short pressed and be raised
  bool scan_permit; //true:scan permit
  bool beep;
}KEY_PARA;

#define FIFO_DEEP 60

typedef struct{
  uint8_t data[FIFO_DEEP];
  uint8_t head; //head++ when put one byte in FIFO
  uint8_t tail; //tail++ when get one byte from FIFO
  uint8_t num; //number of data in fifo
  uint16_t timeout; //use it to judge if a frame is recieved over
                    //set timeout=0 when recive a byte(every 1.042ms:10(bits)*1/9600(bps)=1.042ms)
                    //when timeout>5ms,start to get modbus frame from FIFO 
  bool req; //true: when timeout >= 5ms
}FIFO_DATA;

#define MOD_ADDR    0
#define MOD_FUNC    1
#define MOD_REGH    2
#define MOD_REGL    3
#define MOD_NUMH    4
#define MOD_NUML    5
#define MOD_CRCL    6           
#define MOD_CRCH    7
#define FRAME_LEN   8
typedef struct{
  uint8_t frame[FRAME_LEN]; //frame data buffer
  uint8_t addr;
}MODBUS_PARA;

extern CTRL_PARA ctrl_data;
extern TIME_PARA time_data;
extern T_RH_PARA T_RH_data;
extern DISPLAY_PARA disp_data;
extern KEY_PARA key_data;
extern FIFO_DATA recv_buf;
extern MODBUS_PARA mod_data;

#endif

