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
//!
//! MPY - 16x16 Signed Multiply
//!
//!  Hardware multiplier is used to multiply two numbers. The calculation is
//!  automatically initiated after the second operand is loaded. Results are
//!  stored in RESLO, RESHI and SUMEXT = FFFF if result is negative, SUMEXT = 0
//!  otherwise. Result is also stored as Result variable.
//!
//!  ACLK = 32.768kHz, MCLK = SMCLK = default DCO
//!
//!  Tested On: MSP430F5438A,MSP430FR5739,MSP430FR5969
//!             -----------------
//!         /|\|                 |
//!          | |                 |
//!          --|RST              |
//!            |                 |
//!            |                 |
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - MPY peripheral
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - NONE
//!
//******************************************************************************
#include "driverlib.h"

signed int multiplier = 0x1234;
signed int operand = -6578;
signed long Result;

void main(void)
{
        //Stop Watchdog Timer
        WDT_A_hold(WDT_A_BASE);

        //Set a 16-bit Operand into the specific Operand 1 register to specify
        //signed multiplication
        MPY32_setOperandOne16Bit(
                MPY32_MULTIPLY_SIGNED,
                multiplier);
        //Set 16-bit Operand 2 to begin the multiplication operation
        MPY32_setOperandTwo16Bit(
                operand);

        //Read out the result
        Result = MPY32_getResult32Bit();

        //Enter LPM4
        __bis_SR_register(LPM4_bits);

        //BREAKPOINT HERE to verify the correct Result
        __no_operation();
}
