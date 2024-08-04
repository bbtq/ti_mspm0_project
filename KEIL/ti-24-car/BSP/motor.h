
#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "board.h"
#include "stdint.h"



typedef struct 
{
    uint32_t in1;
    uint32_t in2;
    DL_TIMER_CC_INDEX pwmIndex;
    uint32_t pwm_Period_Count;
    bool reversed;//反向？
    float power_multiple;//动力倍率，（建议：0.0~1.0）
    uint16_t deadzone_up;//0~100
    uint16_t deadzone_down;
    uint16_t Duty_limitMax;//0~1000
    uint16_t Duty_limitMin;//0~1000
    uint32_t count;

}per_param;

typedef struct 
{
    per_param left;
    per_param right;
}motor_wheel;

extern motor_wheel defaultwheel;


void motor_control(int Lduty, int Rduty);

#endif
