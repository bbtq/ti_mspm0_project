/*
 * algorithm.c
 *
 *  Created on: 2024年6月7日
 *      Author: Admin
 */

#include "ti_msp_dl_config.h"

/**
 * @brief 限幅函数
*         if(amt<low)   amt = low;
*               else         amt = amt;
*         if(amt>high)   amt = high;
*               else         amt = amt;
*/

uint8_t constrain_uint8_t(uint8_t amt, uint8_t low, uint8_t high)//限幅
{
  return ((amt)<(low)?(low):((amt) >(high)?(high):(amt)));
}

uint8_t rotate_left_with_bit7_to_bit0(uint8_t value) {
    // 取得 bit7 的值
    uint8_t bit7 = (value & 0x80) >> 7;
    // 左移一位并将 bit7 放到 bit0
    return (value << 1) | bit7;
}

uint8_t rotate_right_with_bit0_to_bit7(uint8_t value) {
    // 取得 bit0 的值
    uint8_t bit0 = (value & 0x01);
    // 右移一位并将 bit0 放到 bit7
    return (value >> 1) | (bit0 << 7);
}

