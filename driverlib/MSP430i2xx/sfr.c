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
// sfr.c - Driver for the sfr Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup sfr_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_SFR__
#include "sfr.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Enables selected SFR interrupt sources.
//!
//! This function enables the selected SFR interrupt sources. Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor. Does not clear interrupt flags.
//!
//! \param interruptMask is the bit mask of interrupts that will be enabled.
//!        - \b SFR_NMI_PIN_INTERRUPT - NMI pin interrupt, if NMI function is
//!           chosen
//!        - \b SFR_OSCILLATOR_FAULT_INTERRUPT - Oscillator fault interrupt
//!        - \b SFR_WATCHDOG_INTERRUPT - Watchdog interrupt
//!        - \b SFR_FLASH_ACCESS_VIOLATION_INTERRUPT - Flash access violation
//!           interrupt
//!
//! \return None
//
//*****************************************************************************
void SFR_enableInterrupt(uint8_t interruptMask)
{
        HWREG8(SFR_BASE + OFS_SFRIE1_L) |= interruptMask;
}

//*****************************************************************************
//
//! \brief Disables selected SFR interrupt sources.
//!
//! This function disables the selected SFR interrupt sources. Only the sources
//! that are enabled can be reflected to the processor interrupt; disabled
//! sources have no effect on the processor.
//!
//! \param interruptMask is the bit mask of interrupts that will be disabled.
//!        - \b SFR_NMI_PIN_INTERRUPT - NMI pin interrupt, if NMI function is
//!           chosen
//!        - \b SFR_OSCILLATOR_FAULT_INTERRUPT - Oscillator fault interrupt
//!        - \b SFR_WATCHDOG_INTERRUPT - Watchdog interrupt
//!        - \b SFR_FLASH_ACCESS_VIOLATION_INTERRUPT - Flash access violation
//!           interrupt
//!
//! \return None
//
//*****************************************************************************
void SFR_disableInterrupt(uint8_t interruptMask)
{
        HWREG8(SFR_BASE + OFS_SFRIE1_L) &= ~(interruptMask);
}

//*****************************************************************************
//
//! \brief Returns the status of the selected SFR interrupt flags.
//!
//! This function returns the status of the selected SFR interrupt flags in a
//! bit mask format matching that passed into the interruptFlagMask parameter.
//!
//! \param interruptFlagMask is the bit mask of interrupt flags that the status
//!        of should be returned.
//!        - \b SFR_NMI_PIN_INTERRUPT - NMI pin interrupt, if NMI function is
//!           chosen
//!        - \b SFR_OSCILLATOR_FAULT_INTERRUPT - Oscillator fault interrupt
//!        - \b SFR_WATCHDOG_INTERRUPT - Watchdog interrupt
//!        - \b SFR_EXTERNAL_RESET_INTERRUPT - External reset interrupt
//!        - \b SFR_BROWN_OUT_RESET_INTERRUPT - Brown out reset interrupt
//!
//! \return A bit mask of the status of the selected interrupt flags.
//!         - \b SFR_NMI_PIN_INTERRUPT NMI pin interrupt, if NMI function is
//!         chosen
//!         - \b SFR_OSCILLATOR_FAULT_INTERRUPT Oscillator fault interrupt
//!         - \b SFR_WATCHDOG_INTERRUPT Watchdog interrupt
//!         - \b SFR_EXTERNAL_RESET_INTERRUPT External reset interrupt
//!         - \b SFR_BROWN_OUT_RESET_INTERRUPT Brown out reset interrupt
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint8_t SFR_getInterruptStatus(uint8_t interruptFlagMask)
{
        return HWREG8(SFR_BASE + OFS_SFRIFG1_L) & interruptFlagMask;
}

//*****************************************************************************
//
//! \brief Clears the selected SFR interrupt flags.
//!
//! This function clears the status of the selected SFR interrupt flags.
//!
//! \param interruptFlagMask is the bit mask of interrupt flags that will be
//!        cleared.
//!        - \b SFR_NMI_PIN_INTERRUPT - NMI pin interrupt, if NMI function is
//!           chosen
//!        - \b SFR_OSCILLATOR_FAULT_INTERRUPT - Oscillator fault interrupt
//!        - \b SFR_WATCHDOG_INTERRUPT - Watchdog interrupt
//!        - \b SFR_EXTERNAL_RESET_INTERRUPT - External reset interrupt
//!        - \b SFR_BROWN_OUT_RESET_INTERRUPT - Brown out reset interrupt
//!
//! \return None
//
//*****************************************************************************
void SFR_clearInterrupt(uint8_t interruptFlagMask)
{
        HWREG8(SFR_BASE + OFS_SFRIFG1_L) &= ~(interruptFlagMask);
}


#endif
//*****************************************************************************
//
//! Close the doxygen group for sfr_api
//! @}
//
//*****************************************************************************
