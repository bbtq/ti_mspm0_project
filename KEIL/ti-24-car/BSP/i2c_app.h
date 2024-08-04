/*
 * i2c_app.h
 *
 *  Created on: 2024Äê7ÔÂ29ÈÕ
 *      Author: 25016
 */

#ifndef I2C_APP_H
#define I2C_APP_H

#include "ti_msp_dl_config.h"

uint8_t i2c_app_read        (I2C_Regs *i2c, uint8_t addr, uint8_t *dat, uint8_t len);
uint8_t i2c_app_write       (I2C_Regs *i2c, uint8_t addr, uint8_t *dat, uint8_t len);

#endif
