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
#ifndef __MSP430WARE_CRC32_H__
#define __MSP430WARE_CRC32_H__
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

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_CRC32__

#include "inc/hw_regaccess.h"

#define CRC16_MODE	0x00
#define CRC32_MODE	0x01

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void CRC32_setSeed (
        uint32_t seed,
        uint8_t crcMode
);
extern void CRC32_set8BitData (
        uint8_t dataIn,
        uint8_t crcMode
);
extern void CRC32_set16BitData (
        uint16_t dataIn,
        uint8_t crcMode
);
extern void CRC32_set32BitData(
        uint32_t dataIn
);
extern void CRC32_set8BitDataReversed (
        uint8_t dataIn,
        uint8_t crcMode
);
extern void CRC32_set16BitDataReversed (
        uint16_t dataIn,
        uint8_t crcMode
);
extern void CRC32_set32BitDataReversed (
        uint32_t dataIn
);
extern uint32_t CRC32_getResult (
        uint8_t crcMode
);
extern uint32_t CRC32_getResultReversed (
        uint8_t crcMode
);
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif
#endif
