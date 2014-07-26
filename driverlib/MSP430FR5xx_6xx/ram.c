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
// ram.c - Driver for the ram Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup ram_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_RC_FRAM__
#include "ram.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Set specified RAM sector off
//!
//! \param sector is specified sector to be set off.
//!        Valid values are:
//!        - \b RAM_SECTOR0
//!        - \b RAM_SECTOR1
//!        - \b RAM_SECTOR2
//!        - \b RAM_SECTOR3
//! \param mode is sector off mode.
//!        Valid values are:
//!        - \b RAM_RETENTION_MODE
//!        - \b RAM_OFF_WAKEUP_MODE
//!        - \b RAM_OFF_NON_WAKEUP_MODE
//!
//! Modified bits of \b RCCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void RAM_setSectorOff(uint8_t sector, uint8_t mode)
{
        assert(sector <= RAM_SECTOR3);

        assert(mode <= RAM_OFF_NON_WAKEUP_MODE);

        uint8_t sectorPos = sector << 1;
        uint8_t val = HWREG8(RAM_BASE + OFS_RCCTL0_L) & ~(0x3 << sectorPos);

        HWREG16(RAM_BASE + OFS_RCCTL0) = (RCKEY | val | (mode << sectorPos));
}

//*****************************************************************************
//
//! \brief Get RAM sector off status
//!
//! \param sector is specified sector to get status.
//!        Valid values are:
//!        - \b RAM_SECTOR0
//!        - \b RAM_SECTOR1
//!        - \b RAM_SECTOR2
//!        - \b RAM_SECTOR3
//!
//! \return Return one of the following:
//!         - \b RAM_RETENTION_MODE
//!         - \b RAM_OFF_WAKEUP_MODE
//!         - \b RAM_OFF_NON_WAKEUP_MODE
//!         \n indicating the status of the masked sectors
//
//*****************************************************************************
uint8_t RAM_getSectorState(uint8_t sector)
{
        assert(sector <= RAM_SECTOR3);

        uint8_t sectorPos = sector << 1;
        return (HWREG8(RAM_BASE + OFS_RCCTL0_L) & (0x3 << sectorPos)) >> sectorPos;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for ram_api
//! @}
//
//*****************************************************************************
