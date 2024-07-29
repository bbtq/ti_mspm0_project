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

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0L130X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_motor_pwm_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_adc_init();
    SYSCFG_DL_SYSTICK_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_TimerG_reset(motor_pwm_INST);
    DL_TimerG_reset(TIMER_0_INST);
    DL_UART_Main_reset(UART_0_INST);
    DL_ADC12_reset(adc_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_TimerG_enablePower(motor_pwm_INST);
    DL_TimerG_enablePower(TIMER_0_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_ADC12_enablePower(adc_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_motor_pwm_C0_IOMUX,GPIO_motor_pwm_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_motor_pwm_C0_PORT, GPIO_motor_pwm_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_motor_pwm_C1_IOMUX,GPIO_motor_pwm_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_motor_pwm_C1_PORT, GPIO_motor_pwm_C1_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(LED1_PIN_14_IOMUX);

    DL_GPIO_initDigitalInput(KEY_PIN_18_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_SCL_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_SDA_IOMUX);

    DL_GPIO_initDigitalOutput(SR04_TRIG_IOMUX);

    DL_GPIO_initDigitalInput(SR04_ECHO_IOMUX);

    DL_GPIO_initDigitalOutput(motor_in_L1_IOMUX);

    DL_GPIO_initDigitalOutput(motor_in_L2_IOMUX);

    DL_GPIO_initDigitalOutput(motor_in_R1_IOMUX);

    DL_GPIO_initDigitalOutput(motor_in_R2_IOMUX);

    DL_GPIO_clearPins(GPIOA, LED1_PIN_14_PIN |
		motor_in_L1_PIN |
		motor_in_L2_PIN |
		motor_in_R1_PIN |
		motor_in_R2_PIN);
    DL_GPIO_setPins(GPIOA, GPIO_SCL_PIN |
		GPIO_SDA_PIN |
		SR04_TRIG_PIN);
    DL_GPIO_enableOutput(GPIOA, LED1_PIN_14_PIN |
		GPIO_SCL_PIN |
		GPIO_SDA_PIN |
		SR04_TRIG_PIN |
		motor_in_L1_PIN |
		motor_in_L2_PIN |
		motor_in_R1_PIN |
		motor_in_R2_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_4_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOA, SR04_ECHO_PIN);
    DL_GPIO_enableInterrupt(GPIOA, SR04_ECHO_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_enableMFCLK();

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gmotor_pwmClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gmotor_pwmConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 4000,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_motor_pwm_init(void) {

    DL_TimerG_setClockConfig(
        motor_pwm_INST, (DL_TimerG_ClockConfig *) &gmotor_pwmClockConfig);

    DL_TimerG_initPWMMode(
        motor_pwm_INST, (DL_TimerG_PWMConfig *) &gmotor_pwmConfig);

    DL_TimerG_setCaptureCompareValue(motor_pwm_INST, 4000, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(motor_pwm_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(motor_pwm_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptureCompareValue(motor_pwm_INST, 4000, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(motor_pwm_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(motor_pwm_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_enableClock(motor_pwm_INST);


    
    DL_TimerG_setCCPDirection(motor_pwm_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );

}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (8000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 8000000 Hz / (4 * (7 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_4,
    .prescale    = 7U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (100us * 1000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

    DL_TimerG_setClockConfig(TIMER_0_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_0ClockConfig);

    DL_TimerG_initTimerMode(TIMER_0_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMER_0_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
	NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 0);
    DL_TimerG_enableClock(TIMER_0_INST);




}



static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9598.08
     */
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_4_MHZ_9600_BAUD, UART_0_FBRD_4_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART_0_INST_INT_IRQN, 3);


    DL_UART_Main_enable(UART_0_INST);
}

/* adc Initialization */
static const DL_ADC12_ClockConfig gadcClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_adc_init(void)
{
    DL_ADC12_setClockConfig(adc_INST, (DL_ADC12_ClockConfig *) &gadcClockConfig);

    DL_ADC12_initSeqSample(adc_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_01, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(adc_INST, adc_ADCMEM_1,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(adc_INST, adc_ADCMEM_2,
        DL_ADC12_INPUT_CHAN_3, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(adc_INST,184);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(adc_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED
		 | DL_ADC12_INTERRUPT_MEM1_RESULT_LOADED));
    DL_ADC12_enableInterrupt(adc_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED
		 | DL_ADC12_INTERRUPT_MEM1_RESULT_LOADED));
    DL_ADC12_enableConversions(adc_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
    /* Initialize the period to 1.00 μs */
    DL_SYSTICK_init(32);
    /* Enable the SysTick and start counting */
    DL_SYSTICK_enable();
}

