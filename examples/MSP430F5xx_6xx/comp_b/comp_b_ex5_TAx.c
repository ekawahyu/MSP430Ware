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
//! COMPB - TA0.1, TA1.1, Varying duty cycle of input voltage
//!
//!  Use CompB to determine if input, Vcompare has a duty cycle greater than
//!  50%. When Vcompare exceeds Vcc*3/4 then TimerA0 captures the time
//!  (TA0CCR1). When Vcompare is less than Vcc*1/4 then TimerA1 captures the
//!  time (TA1CCR1) and resets the timers for TIMERA0 and TIMERA1. If TA0CCR1
//!  is greater than TA1CCR1/2, then turn on the LED. If TA0CCR1 is less
//!  than TA1CCR1/2, then turn off the LED.
//!  Clocks: ACLK = REFO; MCLK = SMCLK = 12MHz
//!
//!             MSP430x552x
//!         ------------------
//!     /|\|                  |
//!      | |                  |
//!      --|RST       P6.0/CB0|<- Vcompare (200Hz<f<1Mhz) (vary dutycycle)
//!        |              P1.0|-> LED ('ON' if dutycycle > 50%;
//!        |                  |        'OFF' if dutycycle < 50%)
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - COMPB peripheral
//! - GPIO Port peripheral
//! - CB0
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - TIMER1_A1_VECTOR
//!
//******************************************************************************

#include "driverlib.h"

#define TIMER_A_PERIOD 0xFFFF
#define UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ 12000
#define UCS_MCLK_FLLREF_RATIO 374

void main(void)
{
        //Stop WDT
        WDT_A_hold(WDT_A_BASE);

        //Set DCO FLL reference = REFO
        UCS_clockSignalInit(
                UCS_FLLREF,
                UCS_REFOCLK_SELECT,
                UCS_CLOCK_DIVIDER_1
                );

        //Set ACLK = REFO
        UCS_clockSignalInit(
                UCS_ACLK,
                UCS_REFOCLK_SELECT,
                UCS_CLOCK_DIVIDER_1
                );

        //Set Ratio and Desired MCLK Frequency  and initialize DCO
        UCS_initFLLSettle(
                UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ,
                UCS_MCLK_FLLREF_RATIO
                );


        //Initialize the Comparator B module
        /*
         * Base Address of Comparator B,
         * Pin CB0 to Positive(+) Terminal,
         * Reference Voltage to Negative(-) Terminal,
         * Normal Power Mode,
         * Output Filter On with minimal delay,
         * Non-Inverted Output Polarity
         */
        COMP_B_init(COMP_B_BASE,
                    COMP_B_INPUT0,
                    COMP_B_VREF,
                    COMP_B_POWERMODE_NORMALMODE,
                    COMP_B_FILTEROUTPUT_DLYLVL1,
                    COMP_B_NORMALOUTPUTPOLARITY
                    );

        //Set the reference voltage that is being supplied to the (-) terminal
        /*
         * Base Address of Comparator B,
         * Reference Voltage of 2.0 V,
         * Lower Limit of Vcc*(8/32) = (1/4)*Vcc,
         * Upper Limit of Vcc*(24/32) = (3/4)*Vcc
         * Static Mode Accuracy
         */
        COMP_B_setReferenceVoltage(COMP_B_BASE,
                                   COMP_B_VREFBASE_VCC,
                                   8,
                                   24,
                                   COMP_B_ACCURACY_STATIC
                                   );

        //Allow power to Comparator module
        COMP_B_enable(COMP_B_BASE);

        //delay for the reference to settle
        __delay_cycles(75);

        //Start timer TIMER_A0 in up mode
        TIMER_A_startUpMode(   TIMER_A0_BASE,
                               TIMER_A_CLOCKSOURCE_SMCLK,
                               TIMER_A_CLOCKSOURCE_DIVIDER_1,
                               TIMER_A_PERIOD,
                               TIMER_A_TAIE_INTERRUPT_DISABLE,
                               TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
                               TIMER_A_DO_CLEAR
                               );

        //Start timer TIMER_A1 in up mode
        TIMER_A_startUpMode(   TIMER_A1_BASE,
                               TIMER_A_CLOCKSOURCE_SMCLK,
                               TIMER_A_CLOCKSOURCE_DIVIDER_1,
                               TIMER_A_PERIOD,
                               TIMER_A_TAIE_INTERRUPT_DISABLE,
                               TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
                               TIMER_A_DO_CLEAR
                               );

        //Capture Falling Edge
        TIMER_A_initCapture(TIMER_A0_BASE,
                            TIMER_A_CAPTURECOMPARE_REGISTER_1,
                            TIMER_A_CAPTUREMODE_FALLING_EDGE,
                            TIMER_A_CAPTURE_INPUTSELECT_CCIxB,
                            TIMER_A_CAPTURE_SYNCHRONOUS,
                            TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
                            TIMER_A_OUTPUTMODE_OUTBITVALUE
                            );

        //Capture Rising Edge, Enable Interrupt
        TIMER_A_clearCaptureCompareInterruptFlag(TIMER_A1_BASE,
                                                 TIMER_A_CAPTURECOMPARE_REGISTER_1);
        TIMER_A_initCapture(TIMER_A1_BASE,
                            TIMER_A_CAPTURECOMPARE_REGISTER_1,
                            TIMER_A_CAPTUREMODE_RISING_EDGE,
                            TIMER_A_CAPTURE_INPUTSELECT_CCIxB,
                            TIMER_A_CAPTURE_SYNCHRONOUS,
                            TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
                            TIMER_A_OUTPUTMODE_OUTBITVALUE
                            );

        //Set P1.0 to output direction
        GPIO_setAsOutputPin(
                GPIO_PORT_P1,
                GPIO_PIN0
                );

        //Enter LPM0 with global interrupts enabled
        __bis_SR_register(LPM3_bits + GIE);

        //For Debug
        __no_operation();
}

//******************************************************************************
//
//This is the TIMER1_A1_VECTOR interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A1_VECTOR)))
#endif
void TIMER_A(void)
{
        uint16_t temp;

        switch ( TA1IV) {
        case  2:

                //Re-Start timer TIMER_A0
                TIMER_A_startUpMode(   TIMER_A0_BASE,
                                       TIMER_A_CLOCKSOURCE_SMCLK,
                                       TIMER_A_CLOCKSOURCE_DIVIDER_1,
                                       TIMER_A_PERIOD,
                                       TIMER_A_TAIE_INTERRUPT_DISABLE,
                                       TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
                                       TIMER_A_DO_CLEAR
                                       );

                //Re-Start timer TIMER_A1
                TIMER_A_startUpMode(   TIMER_A1_BASE,
                                       TIMER_A_CLOCKSOURCE_SMCLK,
                                       TIMER_A_CLOCKSOURCE_DIVIDER_1,
                                       TIMER_A_PERIOD,
                                       TIMER_A_TAIE_INTERRUPT_DISABLE,
                                       TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
                                       TIMER_A_DO_CLEAR
                                       );

                //Compare On and off time of input signal
                temp = (TIMER_A_getCaptureCompareCount(TIMER_A1_BASE,
                                                       TIMER_A_CAPTURECOMPARE_REGISTER_1
                                                       )) >> 1;

                if (TIMER_A_getCaptureCompareCount(TIMER_A0_BASE,
                                                   TIMER_A_CAPTURECOMPARE_REGISTER_1
                                                   ) > temp) {
                        //set P1.0
                        GPIO_setOutputHighOnPin(
                                GPIO_PORT_P1,
                                GPIO_PIN0
                                );
                } else {
                        //Clear P1.0 LED off
                        GPIO_setOutputLowOnPin(
                                GPIO_PORT_P1,
                                GPIO_PIN0
                                );
                }
                break;
        case  4: break;         //CCR2 not used
        case  6: break;         //CCR3 not used
        case  8: break;         //CCR4 not used
        case 10: break;         //CCR5 not used
        case 12: break;         //Reserved not used
        case 14:                //Overflow
                __no_operation();

                //If input frequency < 200Hz, trap here
                while (1) ;
        default: break;
        }
}
