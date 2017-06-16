#include "stm32f1xx_hal.h"
#include "stm32zet6_milestone.h"
#include "bmp180.h"
#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;


#define    DEFAULT_I2C_TIMEOUT     0xff //in ms

static void lcd_rs_high()
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
}
static void lcd_rs_low()
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
}

static void lcd_rst_high()
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
}
static void lcd_rst_low()
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
}

static void lcd_cs_high()
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
}
static void lcd_cs_low()
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
}

void BSP_BMP180_IO_INIT()
{
  
}
void BSP_BMP_DELAY(uint8_t delay_ms)
{
  osDelay(delay_ms);
}
void BSP_BMP180_IO_WRITE_MULTIPLE_DATA(uint8_t dev_addr,uint8_t* ptr_param,uint8_t param_cnt)
{
   HAL_I2C_Master_Transmit(&hi2c1, dev_addr, ptr_param,  param_cnt,DEFAULT_I2C_TIMEOUT); 
}

void BSP_BMP180_IO_READ_MULTIPLE_DATA(uint8_t dev_addr,uint8_t* ptr_param,uint8_t param_cnt)
{
  HAL_I2C_Master_Receive(&hi2c1, dev_addr, ptr_param, param_cnt,DEFAULT_I2C_TIMEOUT); 
}


void lcd_io_init(void )
{ 
  lcd_cs_high();
  lcd_cs_low();
  lcd_rst_low();
  lcd_rst_high();

  
}
void lcd_io_write_cmd(uint8_t cmd)
{
  lcd_rs_low();
  HAL_SPI_Transmit(&hspi1,&cmd,1,0xffff);
}
void lcd_io_write_multiple_data(uint8_t* ptr_dis_data,uint16_t size)
{
  lcd_rs_high();
  HAL_SPI_Transmit(&hspi1,ptr_dis_data,size,0xffff);
}

void BSP_BMP180_init()
{
  bmp180_init();
}

uint32_t BSP_BMP180_read_true_temperature_and_pressure(int32_t* ptr_t,int32_t* ptr_p)
{
 return  bmp180_read_true_temperature_and_pressure(ptr_t,ptr_p); 
}


