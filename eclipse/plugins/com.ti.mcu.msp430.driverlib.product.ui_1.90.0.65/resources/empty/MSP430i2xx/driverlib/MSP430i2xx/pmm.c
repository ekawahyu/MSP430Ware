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

#ifdef __MSP430_HAS_PMM__
#include "pmm.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Sets up the voltage monitor.
//!
//! \param voltageMonitorLevel
//!        Valid values are:
//!        - \b PMM_DISABLE_VMON - Disable the voltage monitor
//!        - \b PMM_DVCC_2350MV - Compare DVCC to 2350mV
//!        - \b PMM_DVCC_2650MV - Compare DVCC to 2650mV
//!        - \b PMM_DVCC_2850MV - Compare DVCC to 2850mV
//!        - \b PMM_VMONIN_1160MV - Compare VMONIN to 1160mV
//!        \n Modified bits are \b VMONLVLx of \b VMONCTL register.
//!
//! Modified bits of \b VMONCTL register.
//!
//! \return None
//
//*****************************************************************************
void PMM_setupVoltageMonitor(uint8_t voltageMonitorLevel) {
    uint8_t currentStatus = VMONCTL;
    currentStatus &= ~(0x07);
    currentStatus |= voltageMonitorLevel;
    VMONCTL = currentStatus;
}

//*****************************************************************************
//
//! \brief Setup the calibration.
//!
//!
//! Modified bits of \b REFCAL0 register and bits of \b REFCAL1 register.
//!
//! \return None
//
//*****************************************************************************
void PMM_calibrateReference(void) {
    REFCAL0 = HWREG8(TLV_START + TLV_CAL_REFCAL0);
    REFCAL1 = HWREG8(TLV_START + TLV_CAL_REFCAL1);
}

//*****************************************************************************
//
//! \brief Set the status of the PMM regulator
//!
//! \param status
//!        Valid values are:
//!        - \b PMM_REGULATOR_ON - Turn the PMM regulator off
//!        - \b PMM_REGULATOR_OFF - Turn the PMM regulator on
//!        \n Modified bits are \b REGOFF of \b LPM45CTL register.
//!
//! Modified bits of \b LPM45CTL register.
//!
//! \return None
//
//*****************************************************************************
void PMM_setRegulatorStatus(uint8_t status) {
    uint8_t currentStatus = LPM45CTL;
    currentStatus &= ~(PMMREGOFF);
    currentStatus |= status;
    LPM45CTL = currentStatus;
}

//*****************************************************************************
//
//! \brief Unlocks the IO
//!
//!
//! Modified bits are \b LOCKLPM45 of \b LPM45CTL register.
//!
//! \return None
//
//*****************************************************************************
void PMM_unlockIOConfiguration(void) {
    LPM45CTL &= ~(LOCKLPM45);
}

//*****************************************************************************
//
//! \brief Enables interrupts
//!
//! \param mask
//!        Mask value is the logical OR of any of the following:
//!        - \b PMM_VMON_INTERRUPT - Voltage Monitor Interrupt
//!
//! \return None
//
//*****************************************************************************
void PMM_enableInterrupt(uint8_t mask) {
    VMONCTL |= mask;
}

//*****************************************************************************
//
//! \brief Disables interrupts
//!
//! \param mask
//!        Mask value is the logical OR of any of the following:
//!        - \b PMM_VMON_INTERRUPT - Voltage Monitor Interrupt
//!
//! \return None
//
//*****************************************************************************
void PMM_disableInterrupt(uint8_t mask) {
    VMONCTL &= ~(mask);
}

//*****************************************************************************
//
//! \brief Returns the interrupt status
//!
//! \param mask
//!        Mask value is the logical OR of any of the following:
//!        - \b PMM_VMON_INTERRUPT - Voltage Monitor Interrupt
//!        - \b PMM_LPM45_INTERRUPT - LPM 4.5 Interrupt
//!
//! \return Logical OR of any of the following:
//!         - \b PMM_VMON_INTERRUPT Voltage Monitor Interrupt
//!         - \b PMM_LPM45_INTERRUPT LPM 4.5 Interrupt
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint8_t PMM_getInterruptStatus(uint8_t mask) {
    uint8_t result = 0x00;

    if((mask & PMM_VMON_INTERRUPT) && (VMONCTL & VMONIFG)) {
        result |= PMM_VMON_INTERRUPT;
    }

    if(mask & PMM_LPM45_INTERRUPT) {
        result |= (LPM45CTL & LPM45IFG);
    }

    return result;
}

//*****************************************************************************
//
//! \brief Clears the masked interrupts
//!
//! \param mask
//!        - \b PMM_LPM45_INTERRUPT - LPM 4.5 Interrupt
//!
//! \return None
//
//*****************************************************************************
void PMM_clearInterrupt(uint8_t mask) {
    LPM45CTL &= ~(mask);
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for pmm_api
//! @}
//
//*****************************************************************************
