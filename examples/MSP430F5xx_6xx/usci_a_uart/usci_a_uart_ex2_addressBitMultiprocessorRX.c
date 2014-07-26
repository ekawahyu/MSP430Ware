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
//! This example shows how to configure the UART module to echo a received
//! character. To echo a received character, RX ISR is used.
//!
//!                MSP430F5438A
//!             -----------------
//!         /|\|                 |
//!          | |                 |
//!          --|RST              |
//!            |                 |
//!            |     P3.4/UCA0TXD|------------>
//!            |                 | 9600
//!            |     P3.5/UCA0RXD|<------------
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - UART peripheral
//! - GPIO Port peripheral (for UART pins)
//! - UCA0TXD
//! - UCA0RXD
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - USCI_A0_VECTOR.
//******************************************************************************

#define BAUD_RATE                               9600
#define RECEIVE_DATA_COUNT                      0x02
#define USCI_A_UART_MULTIPROCESSOR_MODE_ADDRESS        0xAA

uint8_t receivedData = 0x00;
uint8_t receiveDataCount = 0x00;

void main(void)
{
        //Stop WDT
        WDT_A_hold(WDT_A_BASE);

        //P3.4,5 = USCI_A0 TXD/RXD
        GPIO_setAsPeripheralModuleFunctionInputPin(
                GPIO_PORT_P3,
                GPIO_PIN4 + GPIO_PIN5
                );

        //Initialize USCI UART
        //Baudrate = 9600, clock freq = 1.048MHz
        //UCBRx = 109, UCBRFx = 0, UCBRSx = 2, UCOS16 = 0
        if ( STATUS_FAIL == USCI_A_UART_initAdvance(USCI_A0_BASE,
                                                    USCI_A_UART_CLOCKSOURCE_ACLK,
                                                    109,
                                                    0,
                                                    2,
                                                    USCI_A_UART_NO_PARITY,
                                                    USCI_A_UART_MSB_FIRST,
                                                    USCI_A_UART_ONE_STOP_BIT,
                                                    USCI_A_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE,
                                                    USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION ))
                return;

        //Enable UART module for operation
        USCI_A_UART_enable(USCI_A0_BASE);

        //Set USCI UART in sleep mode
        USCI_A_UART_setDormant(USCI_A0_BASE);

        //Enable Receive Interrupt
        USCI_A_UART_clearInterruptFlag(USCI_A0_BASE,
                                       USCI_A_UART_RECEIVE_INTERRUPT
                                       );
        USCI_A_UART_enableInterrupt(USCI_A0_BASE,
                                    USCI_A_UART_RECEIVE_INTERRUPT
                                    );

        //Enter LPM3, interrupts enabled
        __bis_SR_register(LPM3_bits + GIE);
        __no_operation();
}

//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A0_VECTOR)))
#endif
void USCI_A0_ISR(void)
{
        switch (__even_in_range(UCA0IV, 4)) {
        //Vector 2 - RXIFG
        case 2:

                //Receive the data
                receivedData = USCI_A_UART_receiveData(USCI_A0_BASE);

                if (receiveDataCount) {
                        receiveDataCount--;
                        //If RECEIVE_DATA_COUNT number of data received go into dormant mode
                        if (0x00 == receiveDataCount)
                                USCI_A_UART_setDormant(USCI_A0_BASE);
                } else {
                        //If address received, wake up from dormant mode for more data
                        if (USCI_A_UART_MULTIPROCESSOR_MODE_ADDRESS == receivedData) {
                                USCI_A_UART_resetDormant(USCI_A0_BASE);
                                receiveDataCount = RECEIVE_DATA_COUNT;
                        }
                }
                //Send data back "echo"
                USCI_A_UART_transmitData(USCI_A0_BASE,
                                         receivedData
                                         );

                break;
        default: break;
        }
}

