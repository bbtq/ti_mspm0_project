/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-27     LCKFB-LP    first version
 */
#ifndef _BSP_ULTRASONIC_H_
#define _BSP_ULTRASONIC_H_

#include "board.h"

extern bool SR04_LEN_GET_FLAG;

#define SR04_TRIG(x)  ( x ? DL_GPIO_setPins(SR04_PORT,SR04_TRIG_PIN) : DL_GPIO_clearPins(SR04_PORT,SR04_TRIG_PIN) )
#define SR04_ECHO()   ( ( ( DL_GPIO_readPins(SR04_PORT,SR04_ECHO_PIN) & SR04_ECHO_PIN ) > 0 ) ? 1 : 0 )

void Ultrasonic_Init(void);//超声波初始化 
void Hcsr04Start(void);//开启超声波测距
float Hcsr04GetLength(void);//获取超声波测量距离

#endif

