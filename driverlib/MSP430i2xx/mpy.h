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
// mpy.h - Driver for the MPY Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_MPY_H__
#define __MSP430WARE_MPY_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_MPY__

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

//******************************************************************************
//
// The following is a struct that can be returned by MPY_getResult64Bit()
//
//******************************************************************************
typedef struct {
        uint16_t RES0;
        uint16_t RES1;
        uint16_t RES2;
        uint16_t RES3;
} uint64;

//*****************************************************************************
//
// The following are values that can be passed to the writeDelaySelect
// parameter for functions: MPY_setWriteDelay().
//
//*****************************************************************************
#define MPY_WRITEDELAY_OFF                          (!(MPYDLY32 + MPYDLYWRTEN))
#define MPY_WRITEDELAY_32BIT                                      (MPYDLYWRTEN)
#define MPY_WRITEDELAY_64BIT                           (MPYDLY32 + MPYDLYWRTEN)

//*****************************************************************************
//
// The following are values that can be passed to the multiplicationType
// parameter for functions: MPY_setOperandOne8Bit(), MPY_setOperandOne16Bit(),
// MPY_setOperandOne24Bit(), and MPY_setOperandOne32Bit().
//
//*****************************************************************************
#define MPY_MULTIPLY_UNSIGNED                                            (0x00)
#define MPY_MULTIPLY_SIGNED                                              (0x02)
#define MPY_MULTIPLYACCUMULATE_UNSIGNED                                  (0x04)
#define MPY_MULTIPLYACCUMULATE_SIGNED                                    (0x06)

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the MPY_getSaturationMode() function.
//
//*****************************************************************************
#define MPY_SATURATION_MODE_DISABLED                                       0x00
#define MPY_SATURATION_MODE_ENABLED                                      MPYSAT

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the MPY_getFractionalMode() function.
//
//*****************************************************************************
#define MPY_FRACTIONAL_MODE_DISABLED                                       0x00
#define MPY_FRACTIONAL_MODE_ENABLED                                     MPYFRAC

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void MPY_setOperandOne8Bit(uint8_t multiplicationType,
                                  uint8_t operand);

extern void MPY_setOperandOne16Bit(uint8_t multiplicationType,
                                   uint16_t operand);

extern void MPY_setOperandTwo8Bit(uint8_t operand);

extern void MPY_setOperandTwo16Bit(uint16_t operand);

extern uint32_t MPY_getResult(void);

extern uint16_t MPY_getSumExtension(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_MPY_H__
