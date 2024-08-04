
#include "bsp_adc.h"
#include "bsp_Filter.h"

bool gCheckADC1 = false ;
bool gCheckADC2 = false ;


void adc_start(void)
{
    //软件触发ADC开始转换
    DL_ADC12_startConversion(adc_INST);

}

//读取ADC的数据
void adc_getValue(uint16_t * adcbuf)
{
    //获取数据
    adcbuf[0] = DL_ADC12_getMemResult(adc_INST, adc_ADCMEM_1);
    adcbuf[1] = DL_ADC12_getMemResult(adc_INST, adc_ADCMEM_2);

    //卡尔曼滤波
    adcbuf[0] = kalman_update(&kf_adc1,(float)adcbuf[0]);
    adcbuf[1] = kalman_update(&kf_adc2,(float)adcbuf[1]);
    
    //清除标志位
    gCheckADC1 = false;
    gCheckADC2 = false;

}

float adc_getvoltage(uint16_t adc_value)
{
    return (adc_value/4095.0f*3.3f);
}

//ADC中断服务函数
void adc_INST_IRQHandler(void)
{
        //查询并清除ADC中断
        switch (DL_ADC12_getPendingInterrupt(adc_INST)) 
        {
                        //检查是否完成数据采集
                        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
                                        gCheckADC1 = true;//将标志位置1
                                        break;
                        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
                                        gCheckADC2 = true;//将标志位置1
                                        break;
                        default:
                                        break;
        }
}


