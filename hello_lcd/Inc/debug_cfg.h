#ifndef __DEBUG_CFG_H__
#define __DEBUG_CFG_H__



#if     DEBUG_OUTPUT > 0
 
#define DEBUG_INFO(format,arg...)   printf(MODULE"fun:%s line:%d\r\n"format,__func__,__LINE__,##arg);
#else
#define DEBUG_INFO(format,arg...)  
#endif











#endif








