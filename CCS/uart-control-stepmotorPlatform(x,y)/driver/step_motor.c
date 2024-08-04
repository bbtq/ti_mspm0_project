/*
 * step_motor.c
 *
 *  Created on: 2024年6月7日
 *      Author: Admin
 */

#include "step_motor.h"

#include "delay.h"

#include "algorithm.h"

/**
 * @brief :Stepper motor uniform motion (5-wire 4-item stepper motor)
 * @param port
 * @param an    :Peripheral Pin Select（4 pin）
 * @param speed :The smaller the speed value, the faster （2~200）
 * @tips: 使用m0时发现a1口翻转速度和a0、2、3不一致，原因不详，本人改用a2、3、4、5口。
 */

void step_motor_evenly(GPIO_Regs* port, uint32_t * an, uint8_t speed , bool wise)
{
    speed = constrain_uint8_t(speed, 2, 200);
    if (wise){
        for(uint8_t i = 0 ; i<4 ; i++)
            {
                uint8_t j = i+1;
                DL_GPIO_setPins(port, an[4-((j==4 ? 0 : j))]);
                delay_ms(speed);
                DL_GPIO_clearPins(port, an[4-i]);
                delay_ms(speed);
            }
    }
    else{
        for(uint8_t i = 0 ; i<4 ; i++)
            {
                uint8_t j = i+1;
                DL_GPIO_setPins(port, an[(j==4 ? 0 : j)]);
                delay_ms(speed);
                DL_GPIO_clearPins(port, an[i]);
                delay_ms(speed);
            }
    }
        //    DL_GPIO_setPins(port, an[1]);
//    delay_ms(speed);
//    DL_GPIO_clearPins(port, an[0]);
//    delay_ms(speed);
//    DL_GPIO_setPins(port, an[2]);
//    delay_ms(speed);
//    DL_GPIO_clearPins(port, an[1]);
//    delay_ms(speed);
//    DL_GPIO_setPins(port, an[3]);
//    delay_ms(speed);
//    DL_GPIO_clearPins(port, an[2]);
//    delay_ms(speed);
//    DL_GPIO_setPins(port, an[0]);
//    delay_ms(speed);
//    DL_GPIO_clearPins(port, an[3]);
//    delay_ms(speed);
}


void step_once(GPIO_Regs* port, uint32_t * an, bool wise,uint8_t * step_step){

    switch (*step_step) {
        //step_1
        case 1 :
            DL_GPIO_setPins(port, an[0]);
            DL_GPIO_clearPins(port, an[1]);
            DL_GPIO_clearPins(port, an[2]);
            DL_GPIO_clearPins(port, an[3]);
            break;
        //step_2
        case 2 :
            DL_GPIO_setPins(port, an[0]);
            DL_GPIO_setPins(port, an[1]);
            DL_GPIO_clearPins(port, an[2]);
            DL_GPIO_clearPins(port, an[3]);
            break;
        //step_3
        case 4 :
            DL_GPIO_clearPins(port, an[0]);
            DL_GPIO_setPins(port, an[1]);
            DL_GPIO_clearPins(port, an[2]);
            DL_GPIO_clearPins(port, an[3]);
            break;
        //step_4
        case 8 :
            DL_GPIO_clearPins(port, an[0]);
            DL_GPIO_setPins(port, an[1]);
            DL_GPIO_setPins(port, an[2]);
            DL_GPIO_clearPins(port, an[3]);
            break;
        //step_5
        case 16 :
            DL_GPIO_clearPins(port, an[0]);
            DL_GPIO_clearPins(port, an[1]);
            DL_GPIO_setPins(port, an[2]);
            DL_GPIO_clearPins(port, an[3]);
            break;
        //step_6
        case 32 :
            DL_GPIO_clearPins(port, an[0]);
            DL_GPIO_clearPins(port, an[1]);
            DL_GPIO_setPins(port, an[2]);
            DL_GPIO_setPins(port, an[3]);
            break;
        //step_7
        case 64 :
            DL_GPIO_clearPins(port, an[0]);
            DL_GPIO_clearPins(port, an[1]);
            DL_GPIO_clearPins(port, an[2]);
            DL_GPIO_setPins(port, an[3]);
            break;
        //step_8
        case 128 :
            DL_GPIO_setPins(port, an[0]);
            DL_GPIO_clearPins(port, an[1]);
            DL_GPIO_clearPins(port, an[2]);
            DL_GPIO_setPins(port, an[3]);
            break;
        default :
            break;
    }


    if(wise) *step_step = rotate_left_with_bit7_to_bit0(*step_step);
    else *step_step = rotate_right_with_bit0_to_bit7(*step_step);
}




