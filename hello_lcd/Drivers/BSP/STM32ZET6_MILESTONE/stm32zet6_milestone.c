#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32zet6_milestone.h"

extern SPI_HandleTypeDef hspi1;
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





