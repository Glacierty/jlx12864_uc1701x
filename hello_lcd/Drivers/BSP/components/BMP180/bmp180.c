#include "stm32f1xx_hal.h"
#include "bmp180.h"
#include "stdio.h"
#include "debug_cfg.h"

#define  MODULE                                   "@BMP180_module@"
#define  BMP180_SlaveAddress                      0xee //定义器件在IIC总线中的从地址 
#define  OSS                                      1    //标准模式
#define  CONVERSION_TIME                          10   //in ms
#define  DEFAULT_MAX_WRITE_CNT_IN_SEQUENCE        1    //1byte

void BSP_BMP180_IO_INIT();
void BSP_BMP_DELAY(uint8_t ms);
void BSP_BMP180_IO_WRITE_MULTIPLE_DATA(uint8_t dev_addr,uint8_t* ptr_param,uint8_t param_cnt); 
void BSP_BMP180_IO_READ_MULTIPLE_DATA(uint8_t dev_addr,uint8_t* ptr_param,uint8_t param_cnt);



int16_t  ac1,ac2,ac3,b1,b2,mb,mc,md;
uint16_t ac4,ac5,ac6;


static void bmp180_write(uint8_t data_addr,uint8_t* ptr_param,uint8_t param_cnt)
{
 uint8_t param[DEFAULT_MAX_WRITE_CNT_IN_SEQUENCE+1];
 uint8_t cnt=1;
 
 if((param_cnt>DEFAULT_MAX_WRITE_CNT_IN_SEQUENCE) || (param_cnt==0))
 return ;
 
 param[0]=data_addr;
 while(cnt<=param_cnt)
 {
 param[cnt++]=*ptr_param++;
 }
 
 BSP_BMP180_IO_WRITE_MULTIPLE_DATA(BMP180_SlaveAddress,param,param_cnt+1);
  
}

static void bmp180_read(uint8_t data_addr,uint8_t* ptr_param,uint8_t param_cnt)
{
 BSP_BMP180_IO_WRITE_MULTIPLE_DATA(BMP180_SlaveAddress,&data_addr,1);
 BSP_BMP180_IO_READ_MULTIPLE_DATA(BMP180_SlaveAddress,ptr_param,param_cnt); 
}


static uint8_t bmp180_read_id()
{
  uint8_t param;
  bmp180_read(0xD0,&param,1);
  
  return param;
}
static uint8_t bmp180_is_conversion_complete()
{
  uint8_t addr;
  uint8_t result;
  addr=0xF4;
  bmp180_read(addr,&result,1);
  if(result&(1<<5))
    return 0;
      
  return 1;
}
void bmp180_init()
{
  uint8_t id;
  uint8_t param[2];
  BSP_BMP180_IO_INIT();
  
  id=bmp180_read_id();

  DEBUG_INFO("bmp180 id :%d\r\n",id);

  bmp180_read(0xAA,param,2);
  ac1=(uint16_t)param[0]<<8|param[1];
    
  bmp180_read(0xAC,param,2);
  ac2=(uint16_t)param[0]<<8|param[1];
  
  bmp180_read(0xAE,param,2);
  ac3=(uint16_t)param[0]<<8|param[1];
    
  bmp180_read(0xB0,param,2);
  ac4=(uint16_t)param[0]<<8|param[1];  
  
  bmp180_read(0xB2,param,2);
  ac5=(uint16_t)param[0]<<8|param[1];
    
  bmp180_read(0xB4,param,2);
  ac6=(uint16_t)param[0]<<8|param[1];
  
  bmp180_read(0xB6,param,2);
  b1=(uint16_t)param[0]<<8|param[1];
    
  bmp180_read(0xB8,param,2);
  b2=(uint16_t)param[0]<<8|param[1];  

  bmp180_read(0xBA,param,2);
  mb=(uint16_t)param[0]<<8|param[1];
  
  bmp180_read(0xBC,param,2);
  mc=(uint16_t)param[0]<<8|param[1];
    
  bmp180_read(0xBE,param,2);
  md=(uint16_t)param[0]<<8|param[1]; 
}


static int32_t bmp180_read_uncompensated_temperature()
{
  uint8_t addr, param;
  uint8_t result[2];
  addr=0xF4;
  param=0x2E;
  bmp180_write(addr,&param,1);
  while(!bmp180_is_conversion_complete())
  {
  BSP_BMP_DELAY(CONVERSION_TIME); 
  };
  
  addr=0xF6;//..0xf6.0xf7
  bmp180_read(addr,result,2);

  return   (uint16_t)result[0]<<8|(uint16_t)result[1];
}

static int32_t bmp180_read_uncompensated_pressure()
{
  uint8_t addr, param;
  uint8_t result[3];
  addr=0xF4;
  param=0x34+(OSS<<6);
  bmp180_write(addr,&param,1);
  while(!bmp180_is_conversion_complete())
  {
  BSP_BMP_DELAY(CONVERSION_TIME); 
  };
  addr=0xF6;//0xf6.0xf7.0xf8
  bmp180_read(addr,result,3); 
  
  return ((uint32_t)result[0]<<16|(uint32_t)result[1]<<8|(uint32_t)result[2])>>(8-OSS);
}

uint32_t bmp180_read_true_temperature_and_pressure(int32_t* ptr_t,int32_t* ptr_p)
{
    int32_t ut, up;
    int32_t x1, x2, b5, b6, x3, b3, p;
    uint32_t b4, b7;
    int32_t  temperature,pressure;
     
    ut = bmp180_read_uncompensated_temperature(); // 读取未校正温度
    up = bmp180_read_uncompensated_pressure();    // 读取未校正压强
	
    x1 = (ut - ac6) * ac5 >> 15;
    x2 = ( mc << 11) / (x1 + md);
    b5 = x1 + x2;
    temperature = (b5 + 8) >> 4;
    
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((ac1 * 4 + x3)<< OSS) + 2)/4;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * ( uint32_t) (x3 + 32768)) >> 15;
    b7 = (( uint32_t) up - b3) * (50000 >> OSS);
    if( b7 < 0x80000000)
      p = (b7 * 2) / b4 ;
    else  
      p = (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = p + ((x1 + x2 + 3791) >> 4); 
    
    *ptr_t=temperature;
    *ptr_p=pressure;
    
    return 0;
}






