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
//  MSP430F51x2 Demo - TimerD0/1-TEC, Synchronizing timers, Normal timer mode
//
//  Description: This code example shows how to configure the TEC module for
//  synchronizing two timer instances (TD0 and TD1). In this example code, TD0 is
//  the master timer and TD1 is the slave instance. The clock configuration of
//  the master is applied to the slave. The counter length, high res enable
//  and timer mode settings of the master are applied to the slave as well.
//
//  Note: TDxCCR0 registers of both master and slave timer instance (TD0 and
//      TD1 in this case) should be the same value
//
//  ACLK = REFO = 32kHz; SMCLK = MCLK = DCO Clock = 12MHz; TD0 = TD1 = 10kHz
//
//                 MSP430F51x2
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |       P1.6/TD0.0|--> CCR0 - 50% dutycycle
//            |       P2.1/TD1.0|--> CCR0 - 50% dutycycle
//            |                 |
//            |       P1.7/TD0.1|--> CCR1 - 20% dutycycle
//            |       P2.2/TD1.1|--> CCR1 - 60% dutycycle
//            |                 |
//            |       P2.0/TD0.2|--> CCR2 - 40% dutycycle
//            |       P2.3/TD1.2|--> CCR2 - 80S% dutycycle
//
//******************************************************************************
#include "driverlib.h"

//*****************************************************************************
//
//Target frequency for MCLK in kHz
//
//*****************************************************************************
#define UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ   24000

//*****************************************************************************
//
//MCLK/FLLRef Ratio
//
//*****************************************************************************
#define UCS_MCLK_FLLREF_RATIO   374

void main(void)
{

        //Stop WDT
        WDT_A_hold(WDT_A_BASE);

        // Increase Vcore setting to level1 to support fsystem=12MHz
        // NOTE: Change core voltage one level at a time..
        PMM_setVCore(PMMCOREV_1);

        // Set DCO FLL reference = REFO
        UCS_clockSignalInit(UCS_FLLREF,
                            UCS_REFOCLK_SELECT,
                            UCS_CLOCK_DIVIDER_1
                            );

        // Set ACLK = REFO
        UCS_clockSignalInit(UCS_ACLK,
                            UCS_REFOCLK_SELECT,
                            UCS_CLOCK_DIVIDER_1
                            );

        // (N + 1) * FLLRef = Fdco
        // (374 + 1) * 32768 = 12MHz
        // Set FLL Div = fDCOCLK/2
        // Enable the FLL control loop
        UCS_initFLLSettle(UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ,
                          UCS_MCLK_FLLREF_RATIO
                          );

        // P1.6/TD0.0, P1.7,TD0.1, options select
        // Output direction
        GPIO_setAsPeripheralModuleFunctionOutputPin(
                GPIO_PORT_P1,
                GPIO_PIN6 | GPIO_PIN7
                );

        // P2.0/TD0.2, P2.1/TD1.0, P2.2/TD1.1, P2.3/TD1.2, options select
        // Output direction
        GPIO_setAsPeripheralModuleFunctionOutputPin(
                GPIO_PORT_P2,
                GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3
                );


        // Configure Master Timer Instance - TimerD0
        // Freq = 100kHz, TD0.1/TD0.2 PWM Period = 10us
        TIMER_D_configureUpMode(TIMER_D0_BASE,
                                TIMER_D_CLOCKSOURCE_SMCLK,
                                TIMER_D_CLOCKSOURCE_DIVIDER_1,
                                TIMER_D_CLOCKINGMODE_EXTERNAL_CLOCK,
                                120,
                                TIMER_D_TDIE_INTERRUPT_DISABLE,
                                TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                                TIMER_D_DO_CLEAR
                                );


        TIMER_D_initCompare(TIMER_D0_BASE,
                            TIMER_D_CAPTURECOMPARE_REGISTER_0,
                            TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_D_OUTPUTMODE_TOGGLE,
                            120
                            );

        TIMER_D_initCompare(TIMER_D0_BASE,
                            TIMER_D_CAPTURECOMPARE_REGISTER_1,
                            TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_D_OUTPUTMODE_RESET_SET,
                            24
                            );

        TIMER_D_initCompare(TIMER_D0_BASE,
                            TIMER_D_CAPTURECOMPARE_REGISTER_2,
                            TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_D_OUTPUTMODE_RESET_SET,
                            48
                            );


        // Configure Slave Timer Instance - TimerD1 PWM outputs
        TIMER_D_configureUpMode(TIMER_D1_BASE,
                                TIMER_D_CLOCKSOURCE_EXTERNAL_TDCLK,
                                TIMER_D_CLOCKSOURCE_DIVIDER_1,
                                TIMER_D_CLOCKINGMODE_AUXILIARY_CLK,
                                120,
                                TIMER_D_TDIE_INTERRUPT_DISABLE,
                                TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                                TIMER_D_DO_CLEAR
                                );

        TIMER_D_initCompare(TIMER_D1_BASE,
                            TIMER_D_CAPTURECOMPARE_REGISTER_0,
                            TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_D_OUTPUTMODE_TOGGLE,
                            120
                            );

        TIMER_D_initCompare(TIMER_D1_BASE,
                            TIMER_D_CAPTURECOMPARE_REGISTER_1,
                            TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_D_OUTPUTMODE_RESET_SET,
                            72
                            );

        TIMER_D_initCompare(TIMER_D1_BASE,
                            TIMER_D_CAPTURECOMPARE_REGISTER_2,
                            TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_D_OUTPUTMODE_RESET_SET,
                            96
                            );

        // Syncronizing master (TD0) and slave (TD1) timer instances
        // Enable synchronized clear by enabling Aux clear of slave timer
        TEC_enableAuxiliaryClearSignal(TEC1_BASE);

        // Clear timer counter, Up mode, Start timers
        TIMER_D_startCounter(TIMER_D0_BASE,
                             TIMER_D_UP_MODE);

        __bis_SR_register(LPM0_bits);           // Enter LPM0
        __no_operation();                       // For debugger

}
