/*
 * step_motor.h
 *
 *  Created on: 2024Äê6ÔÂ7ÈÕ
 *      Author: Admin
 */

#ifndef DRIVER_STEP_MOTOR_H_
#define DRIVER_STEP_MOTOR_H_

#include "ti_msp_dl_config.h"

void step_motor_evenly(GPIO_Regs* port, uint32_t * an, uint8_t speed , bool wise);
void step_once(GPIO_Regs* port, uint32_t * an, bool wise, uint8_t * step_step);

#endif /* DRIVER_STEP_MOTOR_H_ */
