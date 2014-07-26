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
//  MSP430F51x2 Demo - Timer0_D3, Combining two CCRx register to control one
//                     PWM channel; Normal Timer mode
//
//  Description: This code example shows how to combine two CCRx registers to
//  control both the rising and falling edges of one PWM channel (TD0.2, TD1.2).
//  In up-mode, TDxCCR1 controls the rising edge and TDxCCR2 controls the
//  falling edge. In this example, since CCR1 registers of TD0/1 have the same
//  count, the rising edges at TD0.2 and TD1.2 happen almost at the same
//  time instance. With TD0CCR2=40 and TD1CCR2=96, the dutycycles of the TD0.2
//  and TD1.2 are 30% and 43.75%.
//
//  ACLK = REF0; SMCLK = MCLK = default DCO ~1.045MHz.
//
//                 MSP430F51x2
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |       P2.0/TD0.2|--> CCR2 - 30% duty cycle;
//            |                 |    ON period=(TD0CCR2-TD0CCR1)/32767 ~ 1.22ms
//            |       P2.3/TD1.2|--> CCR2 - 43.75% duty cycle;
//            |                 |    ON period=(TD1CCR2-TD1CCR1)/32767 ~ 1.7ms
//
//******************************************************************************
#include "driverlib.h"

void main(void)
{
        WDT_A_hold(WDT_A_BASE);

        // Set port pins
        GPIO_setAsPeripheralModuleFunctionOutputPin(
                GPIO_PORT_P2,
                GPIO_PIN0 + GPIO_PIN3
                );


        // Configure TimerD0 to combine CCR0/1 block to control TD0CCR2 PWM output
        TIMER_D_combineTDCCRToGeneratePWM(TIMER_D0_BASE,
                                          TIMER_D_CLOCKSOURCE_SMCLK,
                                          TIMER_D_CLOCKSOURCE_DIVIDER_1,
                                          TIMER_D_CLOCKINGMODE_EXTERNAL_CLOCK,
                                          128,
                                          TIMER_D_COMBINE_CCR1_CCR2,
                                          TIMER_D_OUTPUTMODE_RESET_SET,
                                          40,
                                          80
                                          );

        // Configure TimerD1 to combine CCR0/1 block to control TD1CCR2 PWM output
        TIMER_D_combineTDCCRToGeneratePWM(TIMER_D1_BASE,
                                          TIMER_D_CLOCKSOURCE_SMCLK,
                                          TIMER_D_CLOCKSOURCE_DIVIDER_1,
                                          TIMER_D_CLOCKINGMODE_EXTERNAL_CLOCK,
                                          128,
                                          TIMER_D_COMBINE_CCR1_CCR2,
                                          TIMER_D_OUTPUTMODE_RESET_SET,
                                          40,
                                          96
                                          );


        TIMER_D_configureUpMode(TIMER_D0_BASE,
                                TIMER_D_CLOCKSOURCE_ACLK,
                                TIMER_D_CLOCKSOURCE_DIVIDER_1,
                                TIMER_D_CLOCKINGMODE_EXTERNAL_CLOCK,
                                128 - 1,
                                TIMER_D_TDIE_INTERRUPT_DISABLE,
                                TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                                TIMER_D_DO_CLEAR
                                );

        TIMER_D_configureUpMode(TIMER_D1_BASE,
                                TIMER_D_CLOCKSOURCE_ACLK,
                                TIMER_D_CLOCKSOURCE_DIVIDER_1,
                                TIMER_D_CLOCKINGMODE_EXTERNAL_CLOCK,
                                128 - 1,
                                TIMER_D_TDIE_INTERRUPT_DISABLE,
                                TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE,
                                TIMER_D_DO_CLEAR
                                );
        TIMER_D_startCounter(TIMER_D0_BASE,
                             TIMER_D_UP_MODE
                             );
        TIMER_D_startCounter(TIMER_D1_BASE,
                             TIMER_D_UP_MODE
                             );

        __bis_SR_register(LPM0_bits);           // Enter LPM0
        __no_operation();                       // For debugger

}
