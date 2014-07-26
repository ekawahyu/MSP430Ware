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
// tlv.h - Driver for the TLV Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_TLV_H__
#define __MSP430WARE_TLV_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_TLV__

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
// The following are values that can be passed to the tag parameter for
// functions: TLV_getInfo().
//
//*****************************************************************************
#define TLV_CHECKSUM                                      (0x40 | TLV_CHKSUM_L)
#define TLV_TAG_DIE_RECORD                                 (TLV_DIE_RECORD_TAG)
#define TLV_LENGTH_DIE_RECORD                              (TLV_DIE_RECORD_LEN)
#define TLV_WAFER_LOT_ID                              (0x80 | TLV_LOT_WAFER_ID)
#define TLV_DIE_X_POSITION                               (0x40 | TLV_DIE_X_POS)
#define TLV_DIE_Y_POSITION                               (0x40 | TLV_DIE_Y_POS)
#define TLV_TEST_RESULTS                          (0x40 | TLV_DIE_TEST_RESULTS)
#define TLV_REF_CALIBRATION_TAG                                   (TLV_REF_TAG)
#define TLV_REF_CALIBRATION_LENGTH                                (TLV_REF_LEN)
#define TLV_REF_CALIBRATION_REFCAL1                           (TLV_CAL_REFCAL1)
#define TLV_REF_CALIBRATION_REFCAL0                           (TLV_CAL_REFCAL0)
#define TLV_DCO_CALIBRATION_TAG                                   (TLV_DCO_TAG)
#define TLV_DCO_CALIBRATION_LENGTH                                (TLV_DCO_LEN)
#define TLV_DCO_CALIBRATION_CSIRFCAL                         (TLV_CAL_CSIRFCAL)
#define TLV_DCO_CALIBRATION_CSIRTCAL                         (TLV_CAL_CSIRTCAL)
#define TLV_DCO_CALIBRATION_CSERFCAL                         (TLV_CAL_CSERFCAL)
#define TLV_DCO_CALIBRATION_CSERTCAL                         (TLV_CAL_CSERTCAL)
#define TLV_SD24_CALIBRATION_TAG                                 (TLV_SD24_TAG)
#define TLV_SD24_CALIBRATION_LENGTH                              (TLV_SD24_LEN)
#define TLV_SD24_CALIBRATION_SD24TRIM                        (TLV_CAL_SD24TRIM)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void TLV_getInfo(uint8_t tag,
                        uint8_t *length,
                        uint16_t **data_address);

extern bool TLV_performChecksumCheck(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_TLV_H__
