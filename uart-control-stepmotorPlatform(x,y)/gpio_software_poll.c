/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include "./driver/delay.h"

#include "./driver/step_motor.h"

#include "./driver/myuart.h"



    uint8_t rxData = 0;
    uint8_t txData = 0;
//    uint8_t rxbuf[100] = {};
//    uint8_t rxflag = 0;
    uint8_t i,j;

    uint32_t step_x[4] = {step_an1_PIN,step_an2_PIN,step_an3_PIN,step_an4_PIN};
    uint32_t step_y[4] = {step_y_an1_y_PIN,step_y_an2_y_PIN,step_y_an3_y_PIN,step_y_an4_y_PIN};
//    bool x , y;
//    uint8_t j,i,speed=20;
    uint8_t speed_x = 3, speed_y = 4;
    bool    wise_x  = 0, wise_y  = 1;
    uint8_t step_step_x = 1, step_step_y = 1;
    uint8_t stop = 0;

int main(void)
{
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    delay_ms(5);

    /* LED on by default */
//    DL_GPIO_setPins(test_PORT, step_an1_PIN);
//    delay_ms(10);

    DL_TimerG_startCounter(TIMER_0_INST);

    while (1) {




    }
}

void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_0_INST) == DL_UART_MAIN_IIDX_RX) {
            rxData= DL_UART_Main_receiveDataBlocking(UART_0_INST);
            DL_UART_Main_transmitData(UART_0_INST, rxData); //将接收到的数据返还回去
            uart0_pack_transmit(rxData);
//            rxflag = get_rxpack(rxbuf);
    }
}

/**
 * STANDBY0 Clock, runs all the time at the same frequency ----- 1ms
 */
void TIMER_0_INST_IRQHandler(void)
{
    static uint8_t count = 0;
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERG_IIDX_ZERO:
            count ++;

            if(rxflag== 1){
                rxflag = 0;
                switch (rxbuf[0])
                  {
                  case 0x01 : wise_y = 0;  speed_y = rxbuf[1]; break;   //下
                  case 0x02 : wise_y = 1;  speed_y = rxbuf[1]; break;   //上
                  case 0x03 : wise_x = 0;  speed_x = rxbuf[1]; break;   //左
                  case 0x04 : wise_x = 1;  speed_x = rxbuf[1]; break;   //右
                  case 0xff : stop = rxbuf[1];                          //停止/启动 位 ***** 0：all ***** 1：y ***** 2：x *****
                  }
                DL_UART_Main_transmitData(UART_0_INST, speed_y);
            }
            if(stop==1){
                if (count%speed_x == 0) step_once(step_PORT,step_x,wise_x,&step_step_x);
            }
            else if (stop==2){
                if (count%speed_y == 0) step_once(step_PORT,step_y,wise_y,&step_step_y);
            }
            else if (stop==0) ;
            else {
                if (count%speed_x == 0) step_once(step_PORT,step_x,wise_x,&step_step_x);
                if (count%speed_y == 0) step_once(step_PORT,step_y,wise_y,&step_step_y);
            }
            count = count == (speed_x > speed_y ? speed_x : speed_y) ? 0 : count;
            break;
        default:
            break;
    }
}


