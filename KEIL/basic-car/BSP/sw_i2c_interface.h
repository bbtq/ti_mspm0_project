#ifndef _SW_I2C_INTERFACE_H_
#define _SW_I2C_INTERFACE_H_

#include "board.h"

void sw_sda_out(uint8_t bit, void *user_data);
uint8_t sw_sda_in(void *user_data);
void sw_scl_out(uint8_t bit, void *user_data);


#endif
