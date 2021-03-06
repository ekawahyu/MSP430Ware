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
//******************************************************************************
//!Toggle P1.7 using hardware TA1.0 output. Timer1_A is configured
//!for up/down mode with CCR0 defining period, TA1.0 also output on P1.7. In
//!this example, CCR0 is loaded with 250 and TA1.0 will toggle P1.7 at
//!TACLK/2*250. Thus the output frequency on P1.7 will be the TACLK/1000.
//!No CPU or software resources required.
//!As coded with TACLK = SMCLK, P1.7 output frequency is ~1.045M/1000.
//!SMCLK = MCLK = TACLK = default DCO ~1.045MHz
//!
//!Tested On: MSP430F5529
//!		    -------------------
//!		/|\|                   |
//!		 | |                   |
//!		 --|RST                |
//!		   |                   |
//!		   |             P1.7/TA1.0|--> SMCLK/1000
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - Timer peripheral
//! - GPIO Port peripheral
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - NONE
//!
//
//*****************************************************************************

#include "driverlib.h"

#define TIMER_A_PERIOD 250
#define DUTY_CYCLE 250

void main(void)
{
        //Stop WDT
        WDT_A_hold(WDT_A_BASE);

        //P1.7 output
        //P1.7 option select
        GPIO_setAsPeripheralModuleFunctionOutputPin(
                GPIO_PORT_P1,
                GPIO_PIN7
                );

        //Start timer in up down mode
        TIMER_A_configureUpDownMode(
                TIMER_A1_BASE,
                TIMER_A_CLOCKSOURCE_SMCLK,
                TIMER_A_CLOCKSOURCE_DIVIDER_1,
                TIMER_A_PERIOD,
                TIMER_A_TAIE_INTERRUPT_DISABLE,
                TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
                TIMER_A_DO_CLEAR
                );

        //Init compare mode
        TIMER_A_initCompare(TIMER_A1_BASE,
                            TIMER_A_CAPTURECOMPARE_REGISTER_0,
                            TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_A_OUTPUTMODE_TOGGLE,
                            DUTY_CYCLE
                            );

        TIMER_A_startCounter(
                TIMER_A1_BASE,
                TIMER_A_UPDOWN_MODE
                );

        //Enter LPM0
        __bis_SR_register(LPM0_bits);

        //For debugger
        __no_operation();
}

