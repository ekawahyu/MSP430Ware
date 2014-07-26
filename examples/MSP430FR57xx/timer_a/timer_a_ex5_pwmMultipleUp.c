/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//*******************************************************************************
//!  This program generates two PWM outputs on P1.2,P1.3 using
//!  Timer1_A configured for up mode. The value in CCR0, 512-1, defines the PWM
//!  period and the values in CCR1 and CCR2 the PWM duty cycles. Using ~1.045MHz
//!  SMCLK as TACLK, the timer period is ~500us with a 75% duty cycle on P2.2
//!  and 25% on P2.3.
//!  ACLK = n/a, SMCLK = MCLK = TACLK = default DCO ~1.045MHz.
//!
//!  Tested On: MSP430FR5749
//!            -------------------
//!        /|\|                   |
//!         | |                   |
//!         --|RST                |
//!           |                   |
//!           |         P1.2/TA1.1|--> CCR1 - 75% PWM
//!           |         P1.3/TA1.2|--> CCR2 - 25% PWM
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - Timer peripheral
//! - GPIO peripheral
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - NONE
//!
//*****************************************************************************
#include "driverlib.h"

#define TIMER_PERIOD 511
#define DUTY_CYCLE1  384
#define DUTY_CYCLE2  128

void main(void)
{
        //Stop WDT
        WDT_A_hold(WDT_A_BASE);

        //P1.2 and P1.3 output
        //P1.2 and P1.3 options select
        GPIO_setAsPeripheralModuleFunctionOutputPin(
                GPIO_PORT_P1,
                GPIO_PIN2 + GPIO_PIN3,
                GPIO_PRIMARY_MODULE_FUNCTION
                );

        //Start timer
        TIMER_A_configureUpMode(   TIMER_A1_BASE,
                                   TIMER_A_CLOCKSOURCE_SMCLK,
                                   TIMER_A_CLOCKSOURCE_DIVIDER_1,
                                   TIMER_PERIOD,
                                   TIMER_A_TAIE_INTERRUPT_DISABLE,
                                   TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
                                   TIMER_A_DO_CLEAR
                                   );

        TIMER_A_startCounter(   TIMER_A1_BASE,
                                TIMER_A_UP_MODE
                                );

        //Initialize compare mode to generate PWM1
        TIMER_A_initCompare(TIMER_A1_BASE,
                            TIMER_A_CAPTURECOMPARE_REGISTER_1,
                            TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_A_OUTPUTMODE_RESET_SET,
                            DUTY_CYCLE1
                            );

        //Initialize compare mode to generate PWM2
        TIMER_A_initCompare(TIMER_A1_BASE,
                            TIMER_A_CAPTURECOMPARE_REGISTER_2,
                            TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_A_OUTPUTMODE_RESET_SET,
                            DUTY_CYCLE2
                            );

        //Enter LPM0
        __bis_SR_register(LPM0_bits);

        //For debugger
        __no_operation();
}

