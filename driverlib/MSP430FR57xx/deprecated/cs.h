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
// The following are values that can be passed to the clockSourceDivider
// parameter for functions: CS_clockSignalInit().
//
//*****************************************************************************
#define CS_CLOCK_DIVIDER_1                                              DIVM__1
#define CS_CLOCK_DIVIDER_2                                              DIVM__2
#define CS_CLOCK_DIVIDER_4                                              DIVM__4
#define CS_CLOCK_DIVIDER_8                                              DIVM__8
#define CS_CLOCK_DIVIDER_16                                            DIVM__16
#define CS_CLOCK_DIVIDER_32                                            DIVM__32

//*****************************************************************************
//
// The following are values that can be passed to the selectClock parameter for
// functions: CS_enableClockRequest(), and CS_disableClockRequest(); the
// selectedClockSignal parameter for functions: CS_clockSignalInit().
//
//*****************************************************************************
#define CS_ACLK                                                            0x01
#define CS_MCLK                                                            0x02
#define CS_SMCLK                                                           0x04
#define CS_MODOSC                                                   MODCLKREQEN

//*****************************************************************************
//
// The following are values that can be passed to the clockSource parameter for
// functions: CS_clockSignalInit().
//
//*****************************************************************************
#define CS_XT1CLK_SELECT                                           SELM__XT1CLK
#define CS_VLOCLK_SELECT                                           SELM__VLOCLK
#define CS_DCOCLK_SELECT                                           SELM__DCOCLK
#define CS_XT2CLK_SELECT                                           SELM__XT2CLK

//*****************************************************************************
//
// The following are values that can be passed to the xt1drive parameter for
// functions: CS_XT1Start(), and CS_XT1StartWithTimeout().
//
//*****************************************************************************
#define CS_XT1DRIVE_0                                                XT1DRIVE_0
#define CS_XT1DRIVE_1                                                XT1DRIVE_1
#define CS_XT1DRIVE_2                                                XT1DRIVE_2
#define CS_XT1DRIVE_3                                                XT1DRIVE_3

//*****************************************************************************
//
// The following are values that can be passed to the xt2drive parameter for
// functions: CS_XT2Start(), and CS_XT2StartWithTimeout().
//
//*****************************************************************************
#define CS_XT2DRIVE_4MHZ_8MHZ                                        XT2DRIVE_0
#define CS_XT2DRIVE_8MHZ_16MHZ                                       XT2DRIVE_1
#define CS_XT2DRIVE_16MHZ_24MHZ                                      XT2DRIVE_2
#define CS_XT2DRIVE_24MHZ_32MHZ                                      XT2DRIVE_3

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: CS_faultFlagStatus(), and CS_clearFaultFlag() as well as returned
// by the CS_faultFlagStatus() function.
//
//*****************************************************************************
#define CS_XT2OFFG                                                      XT2OFFG
#define CS_XT1OFFG                                                      XT1OFFG

//*****************************************************************************
//
// The following are values that can be passed to the dcorsel parameter for
// functions: CS_setDCOFreq().
//
//*****************************************************************************
#define CS_DCORSEL_0                                                  DCOFSEL_0
#define CS_DCORSEL_1                                                    DCORSEL

//*****************************************************************************
//
// The following are values that can be passed to the dcofsel parameter for
// functions: CS_setDCOFreq().
//
//*****************************************************************************
#define CS_DCOFSEL_0                                                  DCOFSEL_0
#define CS_DCOFSEL_1                                                  DCOFSEL_1
#define CS_DCOFSEL_2                                                  DCOFSEL_2
#define CS_DCOFSEL_3                                                  DCOFSEL_3

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void CS_setExternalClockSource(uint16_t baseAddress,
                                      uint32_t XT1CLK_frequency,
                                      uint32_t XT2CLK_frequency);

extern void CS_clockSignalInit(uint16_t baseAddress,
                               uint8_t selectedClockSignal,
                               uint16_t clockSource,
                               uint16_t clockSourceDivider);

extern void CS_XT1Start(uint16_t baseAddress,
                        uint16_t xt1drive);

extern void CS_bypassXT1(uint16_t baseAddress);

extern bool CS_XT1StartWithTimeout(uint16_t baseAddress,
                                   uint16_t xt1drive,
                                   uint32_t timeout);

extern bool CS_bypassXT1WithTimeout(uint16_t baseAddress,
                                    uint32_t timeout);

extern void CS_XT1Off(uint16_t baseAddress);

extern void CS_XT2Start(uint16_t baseAddress,
                        uint16_t xt2drive);

extern void CS_bypassXT2(uint16_t baseAddress);

extern bool CS_XT2StartWithTimeout(uint16_t baseAddress,
                                   uint16_t xt2drive,
                                   uint32_t timeout);

extern bool CS_bypassXT2WithTimeout(uint16_t baseAddress,
                                    uint32_t timeout);

extern void CS_XT2Off(uint16_t baseAddress);

extern void CS_enableClockRequest(uint16_t baseAddress,
                                  uint8_t selectClock);

extern void CS_disableClockRequest(uint16_t baseAddress,
                                   uint8_t selectClock);

extern uint8_t CS_faultFlagStatus(uint16_t baseAddress,
                                  uint8_t mask);

extern void CS_clearFaultFlag(uint16_t baseAddress,
                              uint8_t mask);

extern uint32_t CS_getACLK(uint16_t baseAddress);

extern uint32_t CS_getSMCLK(uint16_t baseAddress);

extern uint32_t CS_getMCLK(uint16_t baseAddress);

extern uint16_t CS_clearAllOscFlagsWithTimeout(uint16_t baseAddress,
                                               uint32_t timeout);

extern void CS_setDCOFreq(uint16_t baseAddress,
                          uint16_t dcorsel,
                          uint16_t dcofsel);

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
