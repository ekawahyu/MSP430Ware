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
// ram.h - Driver for the RAM Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_RAM_H__
#define __MSP430WARE_RAM_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_RC_FRAM__

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
// The following are values that can be passed to the sector parameter for
// functions: RAM_setSectorOff(), and RAM_getSectorState() as well as returned
// by the RAM_getSectorState() function.
//
//*****************************************************************************
#define RAM_SECTOR0                                                       (0x0)
#define RAM_SECTOR1                                                       (0x1)
#define RAM_SECTOR2                                                       (0x2)
#define RAM_SECTOR3                                                       (0x3)

#define RAM_RETENTION_MODE                                                (0x0)
#define RAM_OFF_WAKEUP_MODE                                         (RCRS0OFF0)
#define RAM_OFF_NON_WAKEUP_MODE                                     (RCRS0OFF1)


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void RAM_setSectorOff(uint8_t sector, uint8_t mode);

extern uint8_t RAM_getSectorState(uint8_t sector);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_RAM_H__
