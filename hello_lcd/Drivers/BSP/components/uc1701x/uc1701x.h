#ifndef __UC1701X_H
#define __UC1701X_H

#define RGB_CODE_BLACK                    0x0001
#define RGB_CODE_BLANK                    0x0000



#define UC1701X_ID                        0x1701


#define DEFAULT_CONTRAST_RESISTER         3 //0-7  粗调对比度 数值越大 对比度越高
#define DEFAULT_CONTRAST_VOLTAGE_VALUE    40//0-63 细调对比度 数值越大 对比度越高
#define DEFAULT_START_LINE                0 //0-63 初始显示行

#define CMD_LCD_ON                          0xAF
#define CMD_LCD_OFF                         0xAE
#define CMD_LCD_START_ADDR_LINE(X)         (1<<6)|(0x3F&(X))          
#define CMD_LCD_ADDR_PAGE(X)               (1<<7|1<<5|1<<4)|(0x07&(X))  
#define CMD_LCD_ADDR_COLUMN_HI(X)          (1<<4)|(((X)>>4)&0x0F)              
#define CMD_LCD_ADDR_COLUMN_LO(X)          (0x0F&(X)) 
#define CMD_LCD_ADC_LEFT_RIGHT             (1<<7|1<<5)       
#define CMD_LCD_ADC_RIGHT_LEFT             (1<<7|1<<5|1<<0) 
#define CMD_LCD_NORMAL                     (1<<7|1<<5|1<<2|1<<1)
#define CMD_LCD_REVERSE                    (1<<7|1<<5|1<<2|1<<1|1<<0)
#define CMD_LCD_ALL_POINTS_OFF             (1<<7|1<<5|1<<2)
#define CMD_LCD_ALL_POINTS_ON              (1<<7|1<<5|1<<2|1<<0)
#define CMD_LCD_BIAS_1_9                   (1<<7|1<<5|1<<1)
#define CMD_LCD_BIAS_1_7                   (1<<7|1<<5|1<<1|1<<0)
#define CMD_LCD_RESET                      (1<<7|1<<6|1<<5|1<<1)
#define CMD_LCD_OUTPUT_MODE_UP_DOWN        (1<<7|1<<6)
#define CMD_LCD_OUTPUT_MODE_DOWN_UP        (1<<7|1<<6|1<<3)
#define CMD_LCD_PWR_CONTROL_OPEN_ALL       (1<<5|1<<3|1<<2|1<<1|1<<0)
#define CMD_LCD_CONTRAST_RESISTER(X)       (1<<5)|(0x07&(X))
#define CMD_LCD_CONTRAST_VOLTAGE_MODE      (1<<7|1<<0)
#define CMD_LCD_CONTRAST_VOLTAGE_VALUE(X)  (0x3F&(X))

void uc1701x_refresh_screen(void);


void lcd_io_init();
void lcd_io_write_cmd(uint8_t cmd);
void lcd_io_write_multiple_data(uint8_t* ptr_dis_data,uint16_t size);


/* LCD driver structure */
extern LCD_DrvTypeDef   uc1701x_drv;

#endif