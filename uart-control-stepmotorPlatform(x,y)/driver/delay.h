/*
 * delay.h
 *
 *  Created on: 2024Äê6ÔÂ6ÈÕ
 *      Author: Admin
 */

#ifndef DRIVER_DELAY_H_
#define DRIVER_DELAY_H_

#include "ti_msp_dl_config.h"

#define delay_us(us) delay_cycles(CPUCLK_FREQ / 1000000 * (us))
#define delay_ms(ms) delay_cycles(CPUCLK_FREQ / 1000 * (ms))


#endif /* DRIVER_DELAY_H_ */
