#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_


#include "board.h"

//循迹
extern bool gCheckADC1;        //ADC1采集成功标志位
extern bool gCheckADC2;        //ADC2采集成功标志位

void adc_start(void);
void adc_getValue(uint16_t * adcbuf);
    

#endif


