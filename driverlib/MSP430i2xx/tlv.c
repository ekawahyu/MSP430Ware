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
// tlv.c - Driver for the tlv Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup tlv_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_TLV__
#include "tlv.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Gets TLV Info
//!
//! The TLV structure uses a tag or base address to identify segments of the
//! table where information is stored. This can be used to retrieve calibration
//! constants for the device or find out more information about the device.
//! This function retrieves the address of a tag and the length of the tag
//! request. Please check the device datasheet for tags available on your
//! device.
//!
//! \param tag represents the tag for which the information needs to be
//!        retrieved.
//!        Valid values are:
//!        - \b TLV_CHECKSUM
//!        - \b TLV_TAG_DIE_RECORD
//!        - \b TLV_LENGTH_DIE_RECORD
//!        - \b TLV_WAFER_LOT_ID
//!        - \b TLV_DIE_X_POSITION
//!        - \b TLV_DIE_Y_POSITION
//!        - \b TLV_TEST_RESULTS
//!        - \b TLV_REF_CALIBRATION_TAG
//!        - \b TLV_REF_CALIBRATION_LENGTH
//!        - \b TLV_REF_CALIBRATION_REFCAL1
//!        - \b TLV_REF_CALIBRATION_REFCAL0
//!        - \b TLV_DCO_CALIBRATION_TAG
//!        - \b TLV_DCO_CALIBRATION_LENGTH
//!        - \b TLV_DCO_CALIBRATION_CSIRFCAL
//!        - \b TLV_DCO_CALIBRATION_CSIRTCAL
//!        - \b TLV_DCO_CALIBRATION_CSERFCAL
//!        - \b TLV_DCO_CALIBRATION_CSERTCAL
//!        - \b TLV_SD24_CALIBRATION_TAG
//!        - \b TLV_SD24_CALIBRATION_LENGTH
//!        - \b TLV_SD24_CALIBRATION_SD24TRIM
//! \param length Acts as a return through indirect reference. The function
//!        retrieves the value of the TLV tag length. This value is pointed to
//!        by *length and can be used by the application level once the
//!        function is called.
//! \param data_address acts as a return through indirect reference. Once the
//!        function is called data_address points to the pointer that holds the
//!        value retrieved from the specified TLV tag.
//!
//! \return None
//
//*****************************************************************************
void TLV_getInfo(uint8_t tag, uint8_t *length, uint16_t **data_address)
{

        // Get access type
        *length = (tag & 0xC0) >> 5;
        if (*length == 0)
                *length = 1;

        *data_address = ((uint16_t*)((uint16_t)(TLV_START + (tag & 0x3F))));

        return;
}

//*****************************************************************************
//
//! \brief Performs checksum check on TLV
//!
//! The 2's complement checksum is calculated on the data stored in the TLV. If
//! the calculated checksum is equal to the checksum stored in the TLV then the
//! user knows that the TLV has not been corrupted. This API can be used after
//! a BOR before writing configuration constants to the appropriate registers.
//!
//!
//! \return true if checksum of TLV matches value stored in TLV, false
//!         otherwise
//
//*****************************************************************************
bool TLV_performChecksumCheck(void)
{

        uint16_t checksum = 0;
        uint16_t *tlv_address = ((uint16_t*)TLV_START) + TLV_CHKSUM_L + 1;
        uint16_t *tlv_end = (uint16_t*)TLV_END;

        do
                checksum ^= *tlv_address++;
        while (tlv_address <= tlv_end);

        checksum ^= 0xFFFF;
        checksum++;

        if (checksum == HWREG16(TLV_START + TLV_CHKSUM_L))
                return true;
        else
                return false;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for tlv_api
//! @}
//
//*****************************************************************************
