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
// pmm.c - Driver for the pmm Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup pmm_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifndef DRIVERLIB_LEGACY_MODE

#ifdef __MSP430_HAS_PMM_FR5xx__
#include "pmm.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Enables the low-side SVS circuitry
//!
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_enableSVSL (void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) |= SVSLE;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Disables the low-side SVS circuitry
//!
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_disableSVSL (void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) &= ~SVSLE;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Enables the high-side SVS circuitry
//!
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_enableSVSH (void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0_L) |= SVSHE;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Disables the high-side SVS circuitry
//!
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_disableSVSH (void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0_L) &= ~SVSHE;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Makes the low-dropout voltage regulator (LDO) remain ON when going
//! into LPM 3/4.
//!
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_regOn (void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) &= ~PMMREGOFF;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Turns OFF the low-dropout voltage regulator (LDO) when going into
//! LPM3/4, thus the system will enter LPM3.5 or LPM4.5 respectively
//!
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_regOff (void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) |= PMMREGOFF;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Calling this function will trigger a software Power On Reset (POR).
//!
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_trigPOR (void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) |= PMMSWPOR;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Calling this function will trigger a software Brown Out Rest (BOR).
//!
//!
//! Modified bits of \b PMMCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_trigBOR (void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) |= PMMSWBOR;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Clears interrupt flags for the PMM
//!
//! \param mask is the mask for specifying the required flag
//!        Mask value is the logical OR of any of the following:
//!        - \b PMM_BOR_INTERRUPT - Software BOR interrupt
//!        - \b PMM_RST_INTERRUPT - RESET pin interrupt
//!        - \b PMM_POR_INTERRUPT - Software POR interrupt
//!        - \b PMM_SVSH_INTERRUPT - SVS high side interrupt
//!        - \b PMM_SVSL_INTERRUPT - SVS low side interrupt, not available for
//!           FR58xx/59xx
//!        - \b PMM_LPM5_INTERRUPT - LPM5 indication
//!        - \b PMM_ALL - All interrupts
//!
//! Modified bits of \b PMMCTL0 register and bits of \b PMMIFG register.
//!
//! \return None
//
//*****************************************************************************
void PMM_clearInterrupt (uint16_t mask)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG16(PMM_BASE + OFS_PMMIFG) &= ~mask;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Returns interrupt status
//!
//! \param mask is the mask for specifying the required flag
//!        Mask value is the logical OR of any of the following:
//!        - \b PMM_BOR_INTERRUPT - Software BOR interrupt
//!        - \b PMM_RST_INTERRUPT - RESET pin interrupt
//!        - \b PMM_POR_INTERRUPT - Software POR interrupt
//!        - \b PMM_SVSH_INTERRUPT - SVS high side interrupt
//!        - \b PMM_SVSL_INTERRUPT - SVS low side interrupt, not available for
//!           FR58xx/59xx
//!        - \b PMM_LPM5_INTERRUPT - LPM5 indication
//!        - \b PMM_ALL - All interrupts
//!
//! \return Logical OR of any of the following:
//!         - \b PMM_BOR_INTERRUPT Software BOR interrupt
//!         - \b PMM_RST_INTERRUPT RESET pin interrupt
//!         - \b PMM_POR_INTERRUPT Software POR interrupt
//!         - \b PMM_SVSH_INTERRUPT SVS high side interrupt
//!         - \b PMM_SVSL_INTERRUPT SVS low side interrupt, not available for
//!         FR58xx/59xx
//!         - \b PMM_LPM5_INTERRUPT LPM5 indication
//!         - \b PMM_ALL All interrupts
//!         \n indicating  the status of the selected  interrupt flags
//
//*****************************************************************************
uint16_t PMM_getInterruptStatus (uint16_t mask)
{
    return ( (HWREG16(PMM_BASE + OFS_PMMIFG)) & mask );
}

//*****************************************************************************
//
//! \brief Unlock LPM5
//!
//! LPMx.5 configuration is not locked and defaults to its reset condition.
//! Disable the GPIO power-on default high-impedance mode to activate
//! previously configured port settings.
//!
//!
//! \return None
//
//*****************************************************************************
void PMM_unlockLPM5 (void)
{
	HWREG8(PMM_BASE + OFS_PM5CTL0) &= ~LOCKLPM5;
}


#endif
#endif
//*****************************************************************************
//
//! Close the doxygen group for pmm_api
//! @}
//
//*****************************************************************************
