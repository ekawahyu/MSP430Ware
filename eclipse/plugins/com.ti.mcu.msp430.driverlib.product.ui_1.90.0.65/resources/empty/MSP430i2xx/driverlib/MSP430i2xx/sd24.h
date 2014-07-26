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
// sd24.h - Driver for the SD24 Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_SD24_H__
#define __MSP430WARE_SD24_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_SD24__

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
// The following is a struct that is passed to SD24_initConverterAdvanced()
//
//******************************************************************************
typedef struct SD24_initConverterAdvancedParam {
    uint8_t converter;
    uint16_t conversionMode;
    uint8_t groupEnable;
    uint8_t inputChannel;
    uint8_t dataFormat;
    uint8_t interruptDelay;
    uint16_t oversampleRatio;
    uint8_t gain;
} SD24_initConverterAdvancedParam;

//*****************************************************************************
//
// The following are values that can be passed to the referenceSelect parameter
// for functions: SD24_init().
//
//*****************************************************************************
#define SD24_REF_EXTERNAL                                                (0x00)
#define SD24_REF_INTERNAL                                            (SD24REFS)

//*****************************************************************************
//
// The following are values that can be passed to the conversionMode parameter
// for functions: SD24_initConverter(), and SD24_initConverterAdvanced().
//
//*****************************************************************************
#define SD24_CONTINUOUS_MODE                                             (0x00)
#define SD24_SINGLE_MODE                                             (SD24SNGL)

//*****************************************************************************
//
// The following are values that can be passed to the converter parameter for
// functions: SD24_initConverter(), SD24_initConverterAdvanced(),
// SD24_setConverterDataFormat(), SD24_startConverterConversion(),
// SD24_stopConverterConversion(), SD24_setInputChannel(),
// SD24_setInterruptDelay(), SD24_setOversampling(), SD24_setGain(),
// SD24_getResults(), SD24_getHighWordResults(), SD24_enableInterrupt(),
// SD24_disableInterrupt(), SD24_clearInterrupt(), and
// SD24_getInterruptStatus().
//
//*****************************************************************************
#define SD24_CONVERTER_0                                                      0
#define SD24_CONVERTER_1                                                      1
#define SD24_CONVERTER_2                                                      2
#define SD24_CONVERTER_3                                                      3

//*****************************************************************************
//
// The following are values that can be passed to the oversampleRatio parameter
// for functions: SD24_initConverterAdvanced(), and SD24_setOversampling().
//
//*****************************************************************************
#define SD24_OVERSAMPLE_32                                         (SD24OSR_32)
#define SD24_OVERSAMPLE_64                                         (SD24OSR_64)
#define SD24_OVERSAMPLE_128                                       (SD24OSR_128)
#define SD24_OVERSAMPLE_256                                       (SD24OSR_256)

//*****************************************************************************
//
// The following are values that can be passed to the inputChannel parameter
// for functions: SD24_initConverterAdvanced(), and SD24_setInputChannel().
//
//*****************************************************************************
#define SD24_INPUT_CH_ANALOG                                       (SD24INCH_0)
#define SD24_INPUT_CH_TEMPERATURE                                  (SD24INCH_6)

//*****************************************************************************
//
// The following are values that can be passed to the dataFormat parameter for
// functions: SD24_initConverterAdvanced(), and SD24_setConverterDataFormat().
//
//*****************************************************************************
#define SD24_DATA_FORMAT_BINARY                                          (0x00)
#define SD24_DATA_FORMAT_2COMPLEMENT                                   (SD24DF)

//*****************************************************************************
//
// The following are values that can be passed to the gain parameter for
// functions: SD24_initConverterAdvanced(), and SD24_setGain().
//
//*****************************************************************************
#define SD24_GAIN_1                                                (SD24GAIN_1)
#define SD24_GAIN_2                                                (SD24GAIN_2)
#define SD24_GAIN_4                                                (SD24GAIN_4)
#define SD24_GAIN_8                                                (SD24GAIN_8)
#define SD24_GAIN_16                                              (SD24GAIN_16)

//*****************************************************************************
//
// The following are values that can be passed to the interruptDelay parameter
// for functions: SD24_initConverterAdvanced(), and SD24_setInterruptDelay().
//
//*****************************************************************************
#define SD24_FIRST_SAMPLE_INTERRUPT                                (SD24INTDLY)
#define SD24_FOURTH_SAMPLE_INTERRUPT                                     (0x00)

//*****************************************************************************
//
// The following are values that can be passed to the groupEnable parameter for
// functions: SD24_initConverterAdvanced().
//
//*****************************************************************************
#define SD24_NOT_GROUPED                                                 (0x00)
#define SD24_GROUPED                                                  (SD24GRP)

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: SD24_enableInterrupt(), SD24_disableInterrupt(),
// SD24_clearInterrupt(), and SD24_getInterruptStatus() as well as returned by
// the SD24_getInterruptStatus() function.
//
//*****************************************************************************
#define SD24_CONVERTER_INTERRUPT                                      (SD24IFG)
#define SD24_CONVERTER_OVERFLOW_INTERRUPT                           (SD24OVIFG)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void SD24_init(uint16_t baseAddress,
                      uint8_t referenceSelect);

extern void SD24_initConverter(uint16_t baseAddress,
                               uint16_t converter,
                               uint16_t conversionMode);

extern void SD24_initConverterAdvanced(uint16_t baseAddress,
                                       SD24_initConverterAdvancedParam *param);

extern void SD24_setConverterDataFormat(uint16_t baseAddress,
                                        uint16_t converter,
                                        uint16_t dataFormat);

extern void SD24_startConverterConversion(uint16_t baseAddress,
                                          uint8_t converter);

extern void SD24_stopConverterConversion(uint16_t baseAddress,
                                         uint8_t converter);

extern void SD24_setInputChannel(uint16_t baseAddress,
                                 uint8_t converter,
                                 uint8_t inputChannel);

extern void SD24_setInterruptDelay(uint16_t baseAddress,
                                   uint8_t converter,
                                   uint8_t interruptDelay);

extern void SD24_setOversampling(uint16_t baseAddress,
                                 uint8_t converter,
                                 uint16_t oversampleRatio);

extern void SD24_setGain(uint16_t baseAddress,
                         uint8_t converter,
                         uint8_t gain);

extern uint32_t SD24_getResults(uint16_t baseAddress,
                                uint8_t converter);

extern uint16_t SD24_getHighWordResults(uint16_t baseAddress,
                                        uint8_t converter);

extern void SD24_enableInterrupt(uint16_t baseAddress,
                                 uint8_t converter,
                                 uint16_t mask);

extern void SD24_disableInterrupt(uint16_t baseAddress,
                                  uint8_t converter,
                                  uint16_t mask);

extern void SD24_clearInterrupt(uint16_t baseAddress,
                                uint8_t converter,
                                uint16_t mask);

extern uint16_t SD24_getInterruptStatus(uint16_t baseAddress,
                                        uint8_t converter,
                                        uint16_t mask);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_SD24_H__
