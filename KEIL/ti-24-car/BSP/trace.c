
#include "trace.h"

int trace_check(uint8_t digital_data){
    int feedback = 0;
    digital_data = ~digital_data;
    switch (digital_data)
    {
        /*没有黑线*/
        case 0x00:
            feedback = 0;
            break;
        /*中心*/
        case 0x18:
            feedback = 0;
            break;
        /*右侧（5~8）黑线，向右*/
        case 0x10:
            feedback = 7;
            break;
        case 0x30:
            feedback = 10;
            break;
        case 0x20:
            feedback = 20;
            break;
        case 0x60:
            feedback = 25;
            break;
        case 0x40:
            feedback = 26;
            break;
        case 0xc0:
            feedback = 28;
            break;
        case 0x80:
            feedback = 30;
            break;
        /*左侧（1~4）黑线，向左*/
        case 0x01:
            feedback = -30;
            break;
        case 0x03:
            feedback = -28;
            break;
        case 0x02:
            feedback = -26;
            break;
        case 0x06:
            feedback = -25;
            break;
        case 0x04:
            feedback = -20;
            break;
        case 0x0c:
            feedback = -10;
            break;
        case 0x08:
            feedback = -7; //10    -5-
            break;
    
        default:    //不做处理防止pid参数变化异常
            break;
    }
    return feedback;
} 
