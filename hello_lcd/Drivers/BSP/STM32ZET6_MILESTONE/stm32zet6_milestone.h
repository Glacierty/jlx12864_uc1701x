#ifndef __STM32ZET6_MILESTONE_H
#define __STM32ZET6_MILESTONE_H


void BSP_BMP180_init();
uint32_t BSP_BMP180_read_true_temperature_and_pressure(int32_t* ptr_t,int32_t* ptr_p);













#endif