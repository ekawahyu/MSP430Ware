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
// wdt.c - Driver for the wdt Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup wdt_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_WDT__
#include "wdt.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Holds the Watchdog Timer.
//!
//! This function stops the watchdog timer from running, that way no interrupt
//! or PUC is asserted.
//!
//! \param baseAddress is the base address of the WDT module.
//!
//! \return None
//
//*****************************************************************************
void WDT_hold (uint16_t baseAddress)
{
    // Set Hold bit
    uint8_t newWDTStatus =
        ((HWREG16(baseAddress + OFS_WDTCTL) & 0x00FF) | WDTHOLD);

    HWREG16(baseAddress + OFS_WDTCTL) = WDTPW + newWDTStatus;
}

//*****************************************************************************
//
//! \brief Starts the Watchdog Timer.
//!
//! This function starts the watchdog timer functionality to start counting
//! again.
//!
//! \param baseAddress is the base address of the WDT module.
//!
//! \return None
//
//*****************************************************************************
void WDT_start (uint16_t baseAddress)
{
    // Reset Hold bit
    uint8_t newWDTStatus =
        ((HWREG16(baseAddress + OFS_WDTCTL) & 0x00FF) & ~(WDTHOLD));

    HWREG16(baseAddress + OFS_WDTCTL) = WDTPW + newWDTStatus;
}

//*****************************************************************************
//
//! \brief Resets the timer counter of the Watchdog Timer.
//!
//! This function resets the watchdog timer to 0x0000h.
//!
//! \param baseAddress is the base address of the WDT module.
//!
//! \return None
//
//*****************************************************************************
void WDT_resetTimer (uint16_t baseAddress)
{
    // Set Counter Clear bit
    uint8_t newWDTStatus =
        ((HWREG16(baseAddress + OFS_WDTCTL) & 0x00FF) | WDTCNTCL);

    HWREG16(baseAddress + OFS_WDTCTL) = WDTPW + newWDTStatus;
}

//*****************************************************************************
//
//! \brief Sets the clock source for the Watchdog Timer in watchdog mode.
//!
//! This function sets the watchdog timer in watchdog mode, which will cause a
//! PUC when the timer overflows. When in the mode, a PUC can be avoided with a
//! call to WDT_resetTimer() before the timer runs out.
//!
//! \param baseAddress is the base address of the WDT module.
//! \param clockSelect is the clock source that the watchdog timer will use.
//!        - \b WDT_CLOCKSOURCE_SMCLK [Default]
//!        - \b WDT_CLOCKSOURCE_ACLK
//!        \n Modified bits are \b WDTSSEL of \b WDTCTL register.
//! \param clockDivider is the divider of the clock source, in turn setting the
//!        watchdog timer interval.
//!        - \b WDT_CLOCKDIVIDER_32K [Default]
//!        - \b WDT_CLOCKDIVIDER_8192
//!        - \b WDT_CLOCKDIVIDER_512
//!        - \b WDT_CLOCKDIVIDER_64
//!        \n Modified bits are \b WDTIS and \b WDTHOLD of \b WDTCTL register.
//!
//! \return None
//
//*****************************************************************************
void WDT_watchdogTimerInit (uint16_t baseAddress,
    uint8_t clockSelect,
    uint8_t clockDivider)
{
    HWREG16(baseAddress + OFS_WDTCTL) =
        WDTPW + WDTCNTCL + WDTHOLD + clockSelect + clockDivider;
}

//*****************************************************************************
//
//! \brief Sets the clock source for the Watchdog Timer in timer interval mode.
//!
//! This function sets the watchdog timer as timer interval mode, which will
//! assert an interrupt without causing a PUC.
//!
//! \param baseAddress is the base address of the WDT module.
//! \param clockSelect is the clock source that the watchdog timer will use.
//!        - \b WDT_CLOCKSOURCE_SMCLK [Default]
//!        - \b WDT_CLOCKSOURCE_ACLK
//!        \n Modified bits are \b WDTSSEL of \b WDTCTL register.
//! \param clockDivider is the divider of the clock source, in turn setting the
//!        watchdog timer interval.
//!        - \b WDT_CLOCKDIVIDER_32K [Default]
//!        - \b WDT_CLOCKDIVIDER_8192
//!        - \b WDT_CLOCKDIVIDER_512
//!        - \b WDT_CLOCKDIVIDER_64
//!        \n Modified bits are \b WDTIS and \b WDTHOLD of \b WDTCTL register.
//!
//! \return None
//
//*****************************************************************************
void WDT_intervalTimerInit (uint16_t baseAddress,
    uint8_t clockSelect,
    uint8_t clockDivider)
{
    HWREG16(baseAddress + OFS_WDTCTL) =
        WDTPW + WDTCNTCL + WDTHOLD + WDTTMSEL + clockSelect + clockDivider;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for wdt_api
//! @}
//
//*****************************************************************************
