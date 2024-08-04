#include <stdio.h>
#include "math.h"
#include "bsp_jy901.h"

#define JY901_PACKET_LENGTH 11

union short_div {
    short data;
    uint8_t div[2];
};

static union short_div angle_yaw;

float jy901_yaw;

void copeJY901_data(uint8_t data)
{
    static uint8_t rxBuffer[JY901_PACKET_LENGTH + 1] = {0}; // 数据包
    static uint8_t rxCount = 0;        // 接收计数

    rxBuffer[rxCount++] = data; // 将收到的数据存入缓冲区中

    if (rxBuffer[0] != 0x55)
    {
        // 数据头不对，则重新开始寻找0x55数据头
        rxCount = 0; // 清空缓存区
        return;
    }
    if (rxCount < JY901_PACKET_LENGTH)
        return; // 数据不满11个，则返回

    /*********** 只有接收满11个字节数据 才会进入以下程序 ************/

    if (0x53 == rxBuffer[1]) // 判断数据包校验是否正确
    {
        angle_yaw.div[0] = rxBuffer[6];
        angle_yaw.div[1] = rxBuffer[7];
        jy901_yaw = angle_yaw.data  / 32768.0f * 180.0f;
        // printf("jy901_yaw : %d . %d\r\n",(int)jy901_yaw,(int)(jy901_yaw*100)%100);
    }
    rxCount = 0;
}
