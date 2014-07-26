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
//*****************************************************************************
//
// cs.h - Driver for the CS Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_CS_H__
#define __MSP430WARE_CS_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_CS__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The following are values that can be passed to the mode parameter for
// functions: CS_setupDCO().
//
//*****************************************************************************
#define CS_INTERNAL_RESISTOR                                               0x00
#define CS_EXTERNAL_RESISTOR                                               DCOR
#define CS_BYPASS_MODE                                                   DCOBYP

//*****************************************************************************
//
// The following are values that can be passed to the clockSource parameter for
// functions: CS_clockSignalInit().
//
//*****************************************************************************
#define CS_MCLK                                                            0x01
#define CS_SMCLK                                                           0x02

//*****************************************************************************
//
// The following are values that can be passed to the clockSourceDivider
// parameter for functions: CS_clockSignalInit().
//
//*****************************************************************************
#define CS_CLOCK_DIVIDER_1                                               DIVM_0
#define CS_CLOCK_DIVIDER_2                                               DIVM_1
#define CS_CLOCK_DIVIDER_4                                               DIVM_2
#define CS_CLOCK_DIVIDER_8                                               DIVM_3
#define CS_CLOCK_DIVIDER_16                                              DIVM_4

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: CS_faultFlagStatus() as well as returned by the
// CS_faultFlagStatus() function.
//
//*****************************************************************************
#define CS_DCO_FAULT_FLAG                                                  DCOF

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void CS_setupDCO(uint8_t mode);

extern void CS_clockSignalInit(uint8_t clockSource,
                               uint8_t clockSourceDivider);

extern uint32_t CS_getACLK(void);

extern uint32_t CS_getSMCLK(void);

extern uint32_t CS_getMCLK(void);

extern uint8_t CS_faultFlagStatus(uint8_t mask);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_CS_H__
