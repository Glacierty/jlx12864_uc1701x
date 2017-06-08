#include "stm32f1xx_hal.h"
#include "lcd.h"
#include "uc1701x.h"
//2017



typedef enum 
{
 CURSOR_V_INCREASE_H_INCREASE =0,
 CURSOR_V_DECREASE_H_DECREASE ,
 CURSOR_V_INCREASE_H_DECREASE ,
 CURSOR_V_DECREASE_H_INCREASE ,
 CURSOR_V_HOLD_ON_H_INCREASE  ,
 CURSOR_H_HOLD_ON_V_INCREASE
}cursor_direction_t;



typedef struct
{
  uint16_t x_pos;
  uint16_t y_pos;
  cursor_direction_t direction;
}cursor_t;
typedef struct 
{
  uint16_t x_pos;
  uint16_t width;
  uint16_t y_pos;
  uint16_t height;
}window_t;

window_t window={0,128,0,64};
cursor_t cursor={0,0,CURSOR_V_INCREASE_H_INCREASE};

uint8_t    array_RGB[128];
uint8_t    display_ram[8][128];//虚拟显存


static void uc1701x_init(void);
static void uc1701x_virtual_set_cursor(uint16_t x,uint16_t y);
static uint16_t uc1701x_read_id(void);
static void uc1701x_display_on(void);
static void uc1701x_display_off(void);
static void uc1701x_virtual_write_pixel(uint16_t x_pos,uint16_t y_pos,uint16_t RGB_code);
static uint16_t uc1701x_virtual_read_pixel(uint16_t x_pos,uint16_t y_pos);
static void uc1701x_virtual_set_display_window(uint16_t x_pos,uint16_t y_pos,uint16_t width,uint16_t height);
static void uc1701x_virtual_draw_h_line(uint16_t RGB_code,uint16_t x_pos,uint16_t y_pos,uint16_t length);
static void uc1701x_virtual_draw_v_line(uint16_t RGB_code,uint16_t x_pos,uint16_t y_pos,uint16_t length);
static uint16_t uc1701x_get_lcd_pixel_width(void);
static uint16_t uc1701x_get_lcd_pixel_height(void);
static void  uc1701x_virtual_draw_bit_map(uint16_t x_pos,uint16_t y_pos, uint8_t *pbmp);
static void  uc1701x_virtual_draw_rgb_image(uint16_t x_pos, uint16_t y_pos, uint16_t x_size, uint16_t y_size, uint8_t* pdata);



LCD_DrvTypeDef uc1701x_drv={
  uc1701x_init,// void     (*Init)(void);
  uc1701x_read_id,//uint16_t (*ReadID)(void);
  uc1701x_display_on,//void     (*DisplayOn)(void);
  uc1701x_display_off,//void     (*DisplayOff)(void);
  uc1701x_virtual_set_cursor,//void     (*SetCursor)(uint16_t, uint16_t);
  uc1701x_virtual_write_pixel,//void     (*WritePixel)(uint16_t, uint16_t, uint16_t);
  uc1701x_virtual_read_pixel,//uint16_t (*ReadPixel)(uint16_t, uint16_t);
  
   /* Optimized operation */
  uc1701x_virtual_set_display_window,//void     (*SetDisplayWindow)(uint16_t, uint16_t, uint16_t, uint16_t);
  uc1701x_virtual_draw_h_line,//void     (*DrawHLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  uc1701x_virtual_draw_v_line,//void     (*DrawVLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  
  uc1701x_get_lcd_pixel_width,//uint16_t (*GetLcdPixelWidth)(void);
  uc1701x_get_lcd_pixel_height,//uint16_t (*GetLcdPixelHeight)(void);
  uc1701x_virtual_draw_bit_map,//void     (*DrawBitmap)(uint16_t, uint16_t, uint8_t*);
  uc1701x_virtual_draw_rgb_image,//void     (*DrawRGBImage)(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t*); 
};



static void uc1701x_write_cmd(uint8_t cmd)
{
  lcd_io_write_cmd(cmd);
}

static void  uc1701x_write_multiple_data(uint8_t* ptr_dis_data,uint16_t size)
{
  lcd_io_write_multiple_data(ptr_dis_data,size);
}

static void  uc1701x_virtual_cursor_update()
{
  switch(cursor.direction)
  {
  case CURSOR_V_INCREASE_H_INCREASE:
       cursor.x_pos++;
       if(cursor.x_pos>=window.x_pos+window.width)
       {
        cursor.x_pos=window.x_pos;
        (cursor.y_pos+1)>=window.y_pos+window.height?cursor.y_pos=window.y_pos:cursor.y_pos++;
       }
    break;
    case CURSOR_V_DECREASE_H_DECREASE:
       
       if(cursor.x_pos<=window.x_pos)
       {
        cursor.x_pos=window.x_pos+window.width-1;
        cursor.y_pos<=window.y_pos?cursor.y_pos=window.y_pos+window.height-1:cursor.y_pos--;
       }
       else
       cursor.x_pos--;
    break; 
    case CURSOR_V_DECREASE_H_INCREASE:     
       cursor.x_pos++;
       if(cursor.x_pos>=window.x_pos+window.width)
       {
        cursor.x_pos=window.x_pos;
        cursor.y_pos<=window.y_pos?cursor.y_pos=window.y_pos+window.height-1:cursor.y_pos--;
       }
    break; 
    case CURSOR_V_INCREASE_H_DECREASE:     
      if(cursor.x_pos<=window.x_pos)
       {
        cursor.x_pos=window.x_pos+window.width-1;
        (cursor.y_pos+1)>=window.y_pos+window.height?cursor.y_pos=window.y_pos:cursor.y_pos++;;
       }
      else
      cursor.x_pos--;
    break; 
    case CURSOR_V_HOLD_ON_H_INCREASE:     
      (cursor.x_pos+1)>=window.x_pos+window.width?cursor.x_pos=window.x_pos:cursor.x_pos++;;
    break;
    case CURSOR_H_HOLD_ON_V_INCREASE:     
      (cursor.y_pos+1)>=window.y_pos+window.height?cursor.y_pos=window.y_pos:cursor.y_pos++;;
    break;   
  }  
}

static uint8_t  is_cursor_in_display_window()
{
   if(cursor.x_pos<window.x_pos||
      cursor.x_pos>=window.x_pos+window.width|| 
      cursor.y_pos<window.y_pos||
      cursor.y_pos>=window.y_pos+window.height )
   { return 0;}
  
   return 1;
}
static void  uc1701x_virtual_write_multiple_data(uint8_t* ptr_dis_data,uint16_t size)
{
  uint8_t row_idx,column_idx,bit_idx;
  uint16_t idx;
  for(idx=0;idx<size;idx++)
  {
  if(is_cursor_in_display_window())
  {
  row_idx=cursor.y_pos>>3;
  column_idx=cursor.x_pos;
  bit_idx=cursor.y_pos&0x07; 
  display_ram[row_idx][column_idx]&=~(1<<bit_idx);
  display_ram[row_idx][column_idx]|=(*ptr_dis_data)<<bit_idx;
  ptr_dis_data++;
  } 
  uc1701x_virtual_cursor_update();
  }
}

static void uc1701x_virtual_set_cursor(uint16_t x,uint16_t y)
{
  cursor.x_pos=x;
  cursor.y_pos=y;
}
static void uc1701x_virtual_set_cursor_direction(cursor_direction_t direction)
{
  cursor.direction=direction;
}
void uc1701x_refresh_screen()
{
  uint16_t page_addr;
  
  for(page_addr=0;page_addr<8;page_addr++)
  {
    uc1701x_write_cmd(CMD_LCD_ADDR_PAGE(page_addr));
    uc1701x_write_cmd(CMD_LCD_ADDR_COLUMN_HI(0));
    uc1701x_write_cmd(CMD_LCD_ADDR_COLUMN_LO(0));
  
    uc1701x_write_multiple_data(&display_ram[page_addr][0],128);
   }
}

static void uc1701x_init(void)
{
 lcd_io_init(); 
 uc1701x_write_cmd(CMD_LCD_RESET);                                                  //lcd复位
 uc1701x_write_cmd(CMD_LCD_PWR_CONTROL_OPEN_ALL);                                   //内部电压全开
 uc1701x_write_cmd(CMD_LCD_CONTRAST_RESISTER(DEFAULT_CONTRAST_RESISTER));            //粗调对比度
 uc1701x_write_cmd(CMD_LCD_CONTRAST_VOLTAGE_MODE);                                  //细调对比度1
 uc1701x_write_cmd(CMD_LCD_CONTRAST_VOLTAGE_VALUE(DEFAULT_CONTRAST_VOLTAGE_VALUE));  //细调对比度2
 uc1701x_write_cmd(CMD_LCD_BIAS_1_9);                                               //设置偏压比1:9
 uc1701x_write_cmd(CMD_LCD_ADC_LEFT_RIGHT);                                         //从左到右扫描
 uc1701x_write_cmd(CMD_LCD_OUTPUT_MODE_DOWN_UP);                                    //从下到上扫描
 uc1701x_write_cmd(CMD_LCD_START_ADDR_LINE(DEFAULT_START_LINE));                     //默认第一行显示
 uc1701x_write_cmd(CMD_LCD_ON);                                                     //打开lcd显示
}

static uint16_t uc1701x_read_id(void)
{
  return UC1701X_ID;
}

static void uc1701x_display_on(void)
{
  uc1701x_write_cmd(CMD_LCD_ON); 
}

static void uc1701x_display_off(void)
{
  uc1701x_write_cmd(CMD_LCD_OFF); 
}



static void uc1701x_virtual_write_pixel(uint16_t x_pos,uint16_t y_pos,uint16_t RGB_code)
{
  uc1701x_virtual_set_cursor(x_pos,y_pos);
  uc1701x_virtual_write_multiple_data((uint8_t*)&RGB_code,1);
}
static uint16_t uc1701x_virtual_read_pixel(uint16_t x_pos,uint16_t y_pos)
{
  uint8_t row_idx,column_idx,bit_idx;
  uc1701x_virtual_set_cursor(x_pos,y_pos);
  row_idx=cursor.y_pos>>3;
  column_idx=cursor.x_pos;
  bit_idx=cursor.y_pos&0x07;
  
  return (display_ram[row_idx][column_idx]&=1<<bit_idx);
}
static void uc1701x_virtual_set_display_window(uint16_t x_pos,uint16_t y_pos,uint16_t width,uint16_t height)
{
  window.x_pos=x_pos;
  window.y_pos=y_pos;
  window.width=width;
  window.height=height;
  
}
static void uc1701x_virtual_draw_h_line(uint16_t RGB_code,uint16_t x_pos,uint16_t y_pos,uint16_t length)
{
  uint16_t idx;
  uc1701x_virtual_set_cursor(x_pos,y_pos);
  uc1701x_virtual_set_cursor_direction(CURSOR_V_HOLD_ON_H_INCREASE);
 
  for(idx=0;idx<length;idx++)
  array_RGB[idx]=RGB_code;
 
  uc1701x_virtual_write_multiple_data(&array_RGB[0],length);  
  uc1701x_virtual_set_cursor_direction(CURSOR_V_INCREASE_H_INCREASE);
}

static void uc1701x_virtual_draw_v_line(uint16_t RGB_code,uint16_t x_pos,uint16_t y_pos,uint16_t length)
{
  uint16_t idx;
  uc1701x_virtual_set_cursor(x_pos,y_pos);
  uc1701x_virtual_set_cursor_direction(CURSOR_H_HOLD_ON_V_INCREASE);
  
  for(idx=0;idx<length;idx++)
  array_RGB[idx]=RGB_code;
  

  uc1701x_virtual_write_multiple_data(&array_RGB[0],length);  
  uc1701x_virtual_set_cursor_direction(CURSOR_V_INCREASE_H_INCREASE);
}


static uint16_t uc1701x_get_lcd_pixel_width(void)
{
 return 128; 
}
uint16_t uc1701x_get_lcd_pixel_height(void)
{
 return 64;
}
static void  uc1701x_virtual_draw_bit_map(uint16_t x_pos,uint16_t y_pos, uint8_t *pbmp)
{
 uint32_t index = 0, size = 0;
  /* Read bitmap size */
  size = *(volatile uint16_t *) (pbmp + 2);
  size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(volatile uint16_t *) (pbmp + 10);
  index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
  //size = (size - index)/2;
  size = size - index;
  pbmp += index;
 
  /* Set Cursor */
  uc1701x_virtual_set_cursor(x_pos, y_pos);  
  uc1701x_virtual_set_cursor_direction(CURSOR_V_DECREASE_H_INCREASE);
  /* Prepare to write GRAM */
  uc1701x_virtual_write_multiple_data((uint8_t*)pbmp, window.width);
  
  uc1701x_virtual_write_multiple_data((uint8_t*)pbmp, size);
  uc1701x_virtual_set_cursor_direction(CURSOR_V_INCREASE_H_INCREASE);
}
static void  uc1701x_virtual_draw_rgb_image(uint16_t x_pos, uint16_t y_pos, uint16_t x_size, uint16_t y_size, uint8_t* pdata)
{
  uint32_t size = 0;

  size = (x_size * y_size);

  /* Set Cursor */
  uc1701x_virtual_set_cursor(x_pos, y_pos);  
  //uc1701x_virtual_set_cursor_direction(CURSOR_DIRECTION_INCREASE,CURSOR_DIRECTION_INCREASE);
  /* Prepare to write GRAM */
  uc1701x_virtual_write_multiple_data((uint8_t*)pdata, size);
}












