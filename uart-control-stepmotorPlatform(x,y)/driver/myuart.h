/*
 * myuart.h
 *
 *  Created on: 2024Äê7ÔÂ11ÈÕ
 *      Author: Admin
 */

#ifndef DRIVER_MYUART_H_
#define DRIVER_MYUART_H_

extern char rxbuf[100];
extern uint8_t rxflag;

void uart0_pack_transmit(uint8_t rxdata);
uint8_t get_rxpack(uint8_t *rxbuf);

#endif /* DRIVER_MYUART_H_ */
