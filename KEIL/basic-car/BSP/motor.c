#include "motor.h"
#include "stdlib.h"

motor_wheel defaultwheel = {
    .left = {
        .in1 = motor_in_L1_PIN,
        .in2 = motor_in_L2_PIN,
        .pwmIndex = DL_TIMER_CC_0_INDEX,
        .pwm_Period_Count = 4000,
        .power_multiple = 0.5,  //动力倍率
        .reversed = 0,
        .deadzone_down = 2,
        .deadzone_up = 2,
        .Duty_limitMax = 3900,
        .Duty_limitMin = 0,
    },
    .right = {
        .in1 = motor_in_R1_PIN,
        .in2 = motor_in_R2_PIN,
        .pwmIndex = DL_TIMER_CC_1_INDEX,
        .pwm_Period_Count = 4000,
        .reversed = 1,
        .power_multiple = 0.5,  //动力倍率
        .deadzone_down = 2,
        .deadzone_up = 2,
        .Duty_limitMax = 3900,
        .Duty_limitMin = 0,
    }
};



void per_motor_control(per_param *wheel,int duty)
{
    if(wheel->reversed)duty = -duty;
    
    if(duty>0){
        DL_GPIO_setPins(GPIOA, wheel->in1);
        DL_GPIO_clearPins(GPIOA, wheel->in2);
    }
    else{
        duty = -duty;
        DL_GPIO_setPins(GPIOA, wheel->in2);
        DL_GPIO_clearPins(GPIOA, wheel->in1);
    }
    duty = (int)(duty*wheel->power_multiple);
    
    duty = limit(duty,wheel->Duty_limitMin,wheel->Duty_limitMax);

    duty = (int)(wheel->power_multiple * duty);
    
    DL_TimerG_setCaptureCompareValue(motor_pwm_INST,wheel->pwm_Period_Count-1-duty,wheel->pwmIndex);
}

//duty必须比period小1
void motor_control(int Lduty, int Rduty)
{
    per_motor_control(&defaultwheel.left,Lduty);
    
    per_motor_control(&defaultwheel.right,Rduty);
}



