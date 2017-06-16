#ifndef __BMP_H__
#define __BMP_H__

void bmp180_init();
uint32_t bmp180_read_true_temperature_and_pressure(int32_t* ptr_t,int32_t* ptr_p);





#endif