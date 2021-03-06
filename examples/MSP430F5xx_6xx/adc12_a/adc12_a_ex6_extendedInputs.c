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
#include "driverlib.h"

//******************************************************************************
//!
//! ADC12_A - Sample A8 & A9 as Inputs, AVcc Ref
//!
//!  This example shows how to perform A/D conversion on up to 10 external
//!  channels by showing that channels A8 and A9 can be used for conversion of
//!  external signals when not using these channels as external reference inputs.
//!  A single sequence of conversions is performed - one conversion on A8 and
//!  then one conversion on A9. Each conversion uses AVcc and AVss for the
//!  references. The conversion results are stored in ADC12MEM0 and ADC12MEM1
//!  respectively and are moved to 'results[]' upon completion of the sequence.
//!  Test by applying voltages to pins VeREF+ for A8 and VREF-/VeREF- for A9,
//!  then setting and running to a break point at the "_BIC..." instruction in
//!  the ISR. To view the conversion results, open a watch window in debugger
//!  and view 'results' or view ADC12MEM0 and ADC12MEM1 in an ADC12_A SFR window.
//!  This can run even in LPM4 mode as ADC has its own clock
//!
//!  NOTE:  When using channels A8 and A9 for external signals, internal
//!  references must be used for the conversions. Refer to figure 18-1 in the
//!  MSP430x5xx Family User's Guide.
//!
//!                MSP430F552x
//!             -----------------
//!         /|\|                 |
//!          | |                 |
//!          --|RST     A8/VeREF+|<- Vin1
//!            |  A9/VREF-/VeREF-|<- Vin2
//!            |                 |
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC12_A peripheral
//! - GPIO Port peripheral
//! - A8/VeREF+
//! - A9/VREF-/VeREF-
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - ADC12_A_VECTOR
//!
//******************************************************************************

//Needs to be global in this example
//Otherwise, the compiler removes it
//because it is not used for anything.
volatile uint16_t results[2];

void main(void)
{
        //Stop Watchdog Timer
        WDT_A_hold(WDT_A_BASE);

        //Initialize the ADC12_A Module
        /*
         * Base address of ADC12_A Module
         * Use internal ADC12_A bit as sample/hold signal to start conversion
         * USE MODOSC 5MHZ Digital Oscillator as clock source
         * Use default clock divider of 1
         */
        ADC12_A_init(ADC12_A_BASE,
                     ADC12_A_SAMPLEHOLDSOURCE_SC,
                     ADC12_A_CLOCKSOURCE_ADC12OSC,
                     ADC12_A_CLOCKDIVIDER_1);

        ADC12_A_enable(ADC12_A_BASE);

        /*
         * Base address of ADC12_A Module
         * For memory buffers 0-7 sample/hold for 1024 clock cycles
         * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
         * Enable Multiple Sampling
         */
        ADC12_A_setupSamplingTimer(ADC12_A_BASE,
                                   ADC12_A_CYCLEHOLD_1024_CYCLES,
                                   ADC12_A_CYCLEHOLD_4_CYCLES,
                                   ADC12_A_MULTIPLESAMPLESENABLE);

        //Configure Memory Buffers
        /*
         * Base address of the ADC12_A Module
         * Configure memory buffer 0
         * Map input A8 to memory buffer 0
         * Vref+ = AVcc
         * Vref- = AVss
         * Memory buffer 0 is not the end of a sequence
         */
        ADC12_A_memoryConfigure(ADC12_A_BASE,
                                ADC12_A_MEMORY_0,
                                ADC12_A_INPUT_A8,
                                ADC12_A_VREFPOS_AVCC,
                                ADC12_A_VREFNEG_AVSS,
                                ADC12_A_NOTENDOFSEQUENCE);
        /*
         * Base address of the ADC12_A Module
         * Configure memory buffer 1
         * Map input A9 to memory buffer 1
         * Vref+ = AVcc
         * Vref- = AVss
         * Memory buffer 1 is not the end of a sequence
         */
        ADC12_A_memoryConfigure(ADC12_A_BASE,
                                ADC12_A_MEMORY_1,
                                ADC12_A_INPUT_A9,
                                ADC12_A_VREFPOS_AVCC,
                                ADC12_A_VREFNEG_AVSS,
                                ADC12_A_ENDOFSEQUENCE);

        //Enable memory buffer 1 interrupt
        ADC12_A_enableInterrupt(ADC12_A_BASE,
                                ADC12IFG1);
        ADC12_A_enableInterrupt(ADC12_A_BASE,
                                ADC12IE1);

        while (1) {
                //Enable/Start sampling and conversion
                /*
                 * Base address of ADC12_A Module
                 * Start the conversion into memory buffer 0
                 * Use the sequence of channels
                 */
                ADC12_A_startConversion(ADC12_A_BASE,
                                        ADC12_A_MEMORY_0,
                                        ADC12_A_SEQOFCHANNELS);

                //Enter LPM4, Enable interrupts
                __bis_SR_register(LPM4_bits + GIE);
                //For debugger
                __no_operation();
        }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC12_VECTOR)))
#endif
void ADC12ISR(void)
{
        switch (__even_in_range(ADC12IV, 34)) {
        case  0: break;         //Vector  0:  No interrupt
        case  2: break;         //Vector  2:  ADC overflow
        case  4: break;         //Vector  4:  ADC timing overflow
        case  6: break;         //Vector  6:  ADC12IFG0
        case  8:                //Vector  8:  ADC12IFG1
                //Move results, IFG is cleared
                results[0] =
                        ADC12_A_getResults(ADC12_A_BASE,
                                           ADC12_A_MEMORY_0);
                //Move results, IFG is cleared
                results[1] =
                        ADC12_A_getResults(ADC12_A_BASE,
                                           ADC12_A_MEMORY_1);

                //Exit active CPU,
                //SET BREAKPOINT HERE and watch results[]
                __bic_SR_register_on_exit(LPM4_bits);
        case 10: break;         //Vector 10:  ADC12IFG2
        case 12: break;         //Vector 12:  ADC12IFG3
        case 14: break;         //Vector 14:  ADC12IFG4
        case 16: break;         //Vector 16:  ADC12IFG5
        case 18: break;         //Vector 18:  ADC12IFG6
        case 20: break;         //Vector 20:  ADC12IFG7
        case 22: break;         //Vector 22:  ADC12IFG8
        case 24: break;         //Vector 24:  ADC12IFG9
        case 26: break;         //Vector 26:  ADC12IFG10
        case 28: break;         //Vector 28:  ADC12IFG11
        case 30: break;         //Vector 30:  ADC12IFG12
        case 32: break;         //Vector 32:  ADC12IFG13
        case 34: break;         //Vector 34:  ADC12IFG14
        default: break;
        }
}

