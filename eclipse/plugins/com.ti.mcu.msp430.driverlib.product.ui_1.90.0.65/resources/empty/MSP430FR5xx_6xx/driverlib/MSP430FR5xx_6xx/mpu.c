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
// mpu.c - Driver for the mpu Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup mpu_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_MPU__
#include "mpu.h"

#include <assert.h>

//*****************************************************************************
//
// The following value is used by createTwoSegments, createThreeSegments to
// check the user has passed a valid segmentation value. This value was
// obtained from the User's Guide.
//
//*****************************************************************************
#define MPU_MAX_SEG_VALUE                                                0x13C1

//*****************************************************************************
//
//! \brief Initializes MPU with two memory segments
//!
//! This function creates two memory segments in FRAM allowing the user to set
//! access right to each segment. To set the correct value for seg1boundary,
//! the user must consult the Device Family User's Guide and provide the MPUSBx
//! value corresponding to the memory address where the user wants to create
//! the partition. Consult the "Segment Border Setting" section in the User's
//! Guide to find the options available for MPUSBx.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param seg1boundary Valid values can be found in the Family User's Guide
//! \param seg1accmask is the bit mask of access right for memory segment 1.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_READ - Read rights
//!        - \b MPU_WRITE - Write rights
//!        - \b MPU_EXEC - Execute rights
//!        - \b MPU_NO_READ_WRITE_EXEC - no read/write/execute rights
//! \param seg2accmask is the bit mask of access right for memory segment 2
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_READ - Read rights
//!        - \b MPU_WRITE - Write rights
//!        - \b MPU_EXEC - Execute rights
//!        - \b MPU_NO_READ_WRITE_EXEC - no read/write/execute rights
//!
//! Modified bits of \b MPUSAM register, bits of \b MPUSEG register and bits of
//! \b MPUCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_initTwoSegments(uint16_t baseAddress,
        uint16_t seg1boundary,
        uint8_t seg1accmask,
        uint8_t seg2accmask
        )
{

    // Verify access right mask for segment1 is a valid selection
    assert(((seg1accmask==(MPU_EXEC|MPU_READ|MPU_WRITE))||
            (seg1accmask==(MPU_EXEC|MPU_READ))||
            (seg1accmask==(MPU_READ|MPU_WRITE))||
            (seg1accmask==(MPU_READ))
            (seg1accmask==(MPU_NO_READ_WRITE_EXEC))));

    // Verify access right mask for segment2 is a valid selection
    assert(((seg2accmask==(MPU_EXEC|MPU_READ|MPU_WRITE))||
            (seg2accmask==(MPU_EXEC|MPU_READ))||
            (seg2accmask==(MPU_READ|MPU_WRITE))||
            (seg2accmask==(MPU_READ))||
            (seg2accmask==(MPU_NO_READ_WRITE_EXEC))));

    // Verify segment1 boundary is valid selection
    assert(seg1boundary<MPU_MAX_SEG_VALUE);

    // Write MPU password to allow MPU register configuration
    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | HWREG8(baseAddress + OFS_MPUCTL0);

    // Create two memory segmentations
    HWREG16(baseAddress + OFS_MPUSEGB1) = seg1boundary;
    HWREG16(baseAddress + OFS_MPUSEGB2) = seg1boundary;

	// Set access rights based on user's selection for segment1
	switch (seg1accmask) {
		case MPU_EXEC|MPU_READ:
			HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEG1WE;
			HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG1XE + MPUSEG1RE;
			break;
		case MPU_READ|MPU_WRITE:
			HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEG1XE;
			HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG1RE + MPUSEG1WE;
			break;
		case MPU_READ:
			HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG1XE +MPUSEG1WE);
			HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG1RE;
			break;
        case MPU_EXEC|MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) |= (MPUSEG1XE + MPUSEG1WE + MPUSEG1RE);
            break;
        case MPU_NO_READ_WRITE_EXEC:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG1XE + MPUSEG1WE + MPUSEG1RE);
            break;
        default:
            break;
	}

	// Set access rights based on user's selection for segment2
	switch (seg2accmask) {
		case MPU_EXEC|MPU_READ:
			HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG3WE + MPUSEG2WE);
			HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG3XE + MPUSEG3RE +
				MPUSEG2XE + MPUSEG2RE;
			break;
		case MPU_READ|MPU_WRITE:
			HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG3XE + MPUSEG2XE);
			HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG3RE + MPUSEG3WE +
				MPUSEG2RE + MPUSEG2WE;
			break;
		case MPU_READ:
			HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG3XE + MPUSEG3WE +
				MPUSEG2XE + MPUSEG2WE);
			HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG3RE + MPUSEG2RE;
			break;
        case MPU_EXEC|MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) |= (MPUSEG3XE + MPUSEG3WE + 
                MPUSEG3RE + MPUSEG2XE + MPUSEG2WE + MPUSEG2RE);
            break;
        case MPU_NO_READ_WRITE_EXEC:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG3XE + MPUSEG3WE + 
                MPUSEG3RE + MPUSEG2XE + MPUSEG2WE + MPUSEG2RE);
            break;
        default:
            break;
	}

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief DEPRECATED - Initializes MPU with three memory segments
//!
//! This function creates three memory segments in FRAM allowing the user to
//! set access right to each segment. To set the correct value for
//! seg1boundary, the user must consult the Device Family User's Guide and
//! provide the MPUSBx value corresponding to the memory address where the user
//! wants to create the partition. Consult the "Segment Border Setting" section
//! in the User's Guide to find the options available for MPUSBx.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param seg1boundary Valid values can be found in the Family User's Guide
//! \param seg2boundary Valid values can be found in the Family User's Guide
//! \param seg1accmask is the bit mask of access right for memory segment 1.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_READ - Read rights
//!        - \b MPU_WRITE - Write rights
//!        - \b MPU_EXEC - Execute rights
//!        - \b MPU_NO_READ_WRITE_EXEC - no read/write/execute rights
//! \param seg2accmask is the bit mask of access right for memory segment 2.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_READ - Read rights
//!        - \b MPU_WRITE - Write rights
//!        - \b MPU_EXEC - Execute rights
//!        - \b MPU_NO_READ_WRITE_EXEC - no read/write/execute rights
//! \param seg3accmask is the bit mask of access right for memory segment 3.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_READ - Read rights
//!        - \b MPU_WRITE - Write rights
//!        - \b MPU_EXEC - Execute rights
//!        - \b MPU_NO_READ_WRITE_EXEC - no read/write/execute rights
//!
//! Modified bits of \b MPUSAM register, bits of \b MPUSEG register and bits of
//! \b MPUCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_createThreeSegments(uint16_t baseAddress,
        uint16_t seg1boundary,
        uint16_t seg2boundary,
        uint8_t seg1accmask,
        uint8_t seg2accmask,
        uint8_t seg3accmask
        )
{
    MPU_initThreeSegmentsParam param = {0};
    param.seg1boundary = seg1boundary;
    param.seg2boundary = seg2boundary;
    param.seg1accmask = seg1accmask;
    param.seg2accmask = seg2accmask;
    param.seg3accmask = seg3accmask;
    MPU_initThreeSegments(baseAddress, &param);
}

//*****************************************************************************
//
//! \brief Initializes MPU with three memory segments
//!
//! This function creates three memory segments in FRAM allowing the user to
//! set access right to each segment. To set the correct value for
//! seg1boundary, the user must consult the Device Family User's Guide and
//! provide the MPUSBx value corresponding to the memory address where the user
//! wants to create the partition. Consult the "Segment Border Setting" section
//! in the User's Guide to find the options available for MPUSBx.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param param is the pointer to struct for initializing three segments.
//!
//! Modified bits of \b MPUSAM register, bits of \b MPUSEG register and bits of
//! \b MPUCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_initThreeSegments(uint16_t baseAddress, 
    MPU_initThreeSegmentsParam *param)
{
    // Verify access right mask for segment1 is a valid selection
    assert((param->seg1accmask==(MPU_EXEC|MPU_READ|MPU_WRITE)) ||
        (param->seg1accmask==(MPU_EXEC|MPU_READ)) ||
        (param->seg1accmask==(MPU_READ|MPU_WRITE)) ||
        (param->seg1accmask==(MPU_READ)) ||
        (param->seg1accmask==(MPU_NO_READ_WRITE_EXEC)));

    // Verify access right mask for segment2 is a valid selection
    assert((param->seg2accmask==(MPU_EXEC|MPU_READ|MPU_WRITE)) ||
        (param->seg2accmask==(MPU_EXEC|MPU_READ)) ||
        (param->seg2accmask==(MPU_READ|MPU_WRITE)) ||
        (param->seg2accmask==(MPU_READ)) ||
        (param->seg2accmask==(MPU_NO_READ_WRITE_EXEC)));

    // Verify access right mask for segment3 is a valid selection
    assert((param->seg3accmask==(MPU_EXEC|MPU_READ|MPU_WRITE)) ||
        (param->seg3accmask==(MPU_EXEC|MPU_READ)) ||
        (param->seg3accmask==(MPU_READ|MPU_WRITE)) ||
        (param->seg3accmask==(MPU_READ)) ||
        (param->seg3accmask==(MPU_NO_READ_WRITE_EXEC)));

    // Verify segment1 boundary is valid selection
    assert(param->seg1boundary<MPU_MAX_SEG_VALUE);
    
    // Write MPU password to allow MPU register configuration
    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | HWREG8(baseAddress + OFS_MPUCTL0);

    // Create two memory segmentations
    HWREG16(baseAddress + OFS_MPUSEGB1) = param->seg1boundary;
    HWREG16(baseAddress + OFS_MPUSEGB2) = param->seg2boundary;

    // Set access rights based on user's selection for segment1
    switch (param->seg1accmask) {
        case MPU_EXEC|MPU_READ:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEG1WE;
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG1XE + MPUSEG1RE;
            break;
        case MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEG1XE;
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG1RE + MPUSEG1WE;
            break;
        case MPU_READ:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG1XE +MPUSEG1WE);
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG1RE;
            break;
        case MPU_EXEC|MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) |= (MPUSEG1XE + MPUSEG1WE + MPUSEG1RE);
            break;
        case MPU_NO_READ_WRITE_EXEC:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG1XE + MPUSEG1WE + MPUSEG1RE);
            break;
        default:
            break;
    }

    // Set access rights based on user's selection for segment2
    switch (param->seg2accmask) {
        case MPU_EXEC|MPU_READ:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEG2WE;
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG2XE + MPUSEG2RE;
            break;
        case MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEG2XE;
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG2RE + MPUSEG2WE;
            break;
        case MPU_READ:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG2XE +MPUSEG2WE);
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG2RE;
            break;
        case MPU_EXEC|MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) |= (MPUSEG2XE + MPUSEG2WE + MPUSEG2RE);
            break;
        case MPU_NO_READ_WRITE_EXEC:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG2XE + MPUSEG2WE + MPUSEG2RE);
            break;
        default:
            break;
    }

    // Set access rights based on user's selection for segment3
    switch (param->seg3accmask) {
        case MPU_EXEC|MPU_READ:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEG3WE;
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG3XE + MPUSEG3RE;
            break;
        case MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEG3XE;
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG3RE + MPUSEG3WE;
            break;
        case MPU_READ:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG3XE +MPUSEG3WE);
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEG3RE;
            break;
        case MPU_EXEC|MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) |= (MPUSEG3XE + MPUSEG3WE + MPUSEG3WE);
            break;
        case MPU_NO_READ_WRITE_EXEC:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEG3XE + MPUSEG3WE + MPUSEG3WE);
            break;
        default:
            break;
    }

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Initializes user information memory segment
//!
//! This function initializes user information memory segment with specified
//! access rights.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param accmask is the bit mask of access right for user information memory
//!        segment.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_READ - Read rights
//!        - \b MPU_WRITE - Write rights
//!        - \b MPU_EXEC - Execute rights
//!        - \b MPU_NO_READ_WRITE_EXEC - no read/write/execute rights
//!
//! Modified bits of \b MPUSAM register and bits of \b MPUCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_initInfoSegment(uint16_t baseAddress, uint8_t accmask)
{
    // Verify access right mask for segment1 is a valid selection
    assert(((accmask==(MPU_EXEC|MPU_READ|MPU_WRITE))||
            (accmask==(MPU_EXEC|MPU_READ))||
            (accmask==(MPU_READ|MPU_WRITE))||
            (accmask==(MPU_READ))||
            (accmask==(MPU_NO_READ_WRITE_EXEC))));

    // Write MPU password to allow MPU register configuration
    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | HWREG8(baseAddress + OFS_MPUCTL0);

    // Set access rights based on user's selection for segment1
    switch (accmask) {
        case MPU_EXEC|MPU_READ:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEGIWE;
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEGIXE + MPUSEGIRE;
            break;
        case MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~MPUSEGIXE;
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEGIRE + MPUSEGIWE;
            break;
        case MPU_READ:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEGIXE + MPUSEGIWE);
            HWREG16(baseAddress + OFS_MPUSAM) |= MPUSEGIRE;
            break;
        case MPU_EXEC|MPU_READ|MPU_WRITE:
            HWREG16(baseAddress + OFS_MPUSAM) |= (MPUSEGIXE + MPUSEGIWE + MPUSEGIRE);
            break;
        case MPU_NO_READ_WRITE_EXEC:
            HWREG16(baseAddress + OFS_MPUSAM) &= ~(MPUSEGIXE + MPUSEGIWE + MPUSEGIRE);
            break;
        default:
            break;
    }

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
}//*****************************************************************************
//
//! \brief The following function enables the NMI Event if a Segment violation
//! has occurred.
//!
//! \param baseAddress is the base address of the MPU module.
//!
//! Modified bits of \b MPUCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_enableNMIevent(uint16_t baseAddress)
{
    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | MPUSEGIE | 
                                            HWREG8(baseAddress + OFS_MPUCTL0);

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief The following function enables the MPU module in the device.
//!
//! This function needs to be called once all memory segmentation has been
//! done. If this function is not called the MPU module will not be activated.
//!
//! \param baseAddress is the base address of the MPU module.
//!
//! Modified bits of \b MPUCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_start(uint16_t baseAddress)
{
    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | MPUENA | HWREG8(baseAddress + OFS_MPUCTL0);

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief The following function enables PUC generation when an access
//! violation has Occurred on the memory segment selected by the user.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param segment is the bit mask of memory segment that will generate a PUC
//!        when an access violation occurs.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_FIRST_SEG - PUC generation on first memory segment
//!        - \b MPU_SECOND_SEG - PUC generation on second memory segment
//!        - \b MPU_THIRD_SEG - PUC generation on third memory segment
//!        - \b MPU_INFO_SEG - PUC generation on user information memory
//!           segment
//!
//! Modified bits of \b MPUSAM register and bits of \b MPUCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_enablePUCOnViolation(uint16_t baseAddress,
        uint16_t segment
        )
{
    // Verify user has selected a valid memory segment
    assert(0x00 != (segment & (MPU_FIRST_SEG + MPU_SECOND_SEG + MPU_THIRD_SEG)));

    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | HWREG8(baseAddress + OFS_MPUCTL0);
    HWREG16(baseAddress + OFS_MPUSAM) |= segment;

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief The following function disables PUC generation when an access
//! violation has Occurred on the memory segment selected by the user.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param segment is the bit mask of memory segment that will NOT generate a
//!        PUC when an access violation occurs.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_FIRST_SEG - PUC generation on first memory segment
//!        - \b MPU_SECOND_SEG - PUC generation on second memory segment
//!        - \b MPU_THIRD_SEG - PUC generation on third memory segment
//!        - \b MPU_INFO_SEG - PUC generation on user information memory
//!           segment
//!
//! Modified bits of \b MPUSAM register and bits of \b MPUCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_disablePUCOnViolation(uint16_t baseAddress,
        uint16_t segment
        )
{
    // Verify user has selected a valid memory segment
    assert(0x00 != (segment & (MPU_FIRST_SEG + MPU_SECOND_SEG + MPU_THIRD_SEG)));

    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | HWREG8(baseAddress + OFS_MPUCTL0);
    HWREG16(baseAddress + OFS_MPUSAM) &= ~segment;

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
}

//*****************************************************************************
//
//! \brief Returns the memory segment violation flag status requested by the
//! user.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param memAccFlag is the is the memory access violation flag.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_SEG_1_ACCESS_VIOLATION - is set if an access violation in
//!           Main Memory Segment 1 is detected
//!        - \b MPU_SEG_2_ACCESS_VIOLATION - is set if an access violation in
//!           Main Memory Segment 2 is detected
//!        - \b MPU_SEG_3_ACCESS_VIOLATION - is set if an access violation in
//!           Main Memory Segment 3 is detected
//!        - \b MPU_SEG_INFO_ACCESS_VIOLATION - is set if an access violation
//!           in User Information Memory Segment is detected
//!
//! \return Logical OR of any of the following:
//!         - \b MPU_SEG_1_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 1 is detected
//!         - \b MPU_SEG_2_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 2 is detected
//!         - \b MPU_SEG_3_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 3 is detected
//!         - \b MPU_SEG_INFO_ACCESS_VIOLATION is set if an access violation in
//!         User Information Memory Segment is detected
//!         \n indicating the status of the masked flags.
//
//*****************************************************************************
uint16_t MPU_getInterruptStatus(uint16_t baseAddress,
        uint16_t memAccFlag
        )
{
    return (HWREG16(baseAddress + OFS_MPUCTL1) & memAccFlag);
}

//*****************************************************************************
//
//! \brief Clears the masked interrupt flags
//!
//! Returns the memory segment violation flag status requested by the user or
//! if user is providing a bit mask value, the function will return a value
//! indicating if all flags were cleared.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param memAccFlag is the is the memory access violation flag.
//!        Mask value is the logical OR of any of the following:
//!        - \b MPU_SEG_1_ACCESS_VIOLATION - is set if an access violation in
//!           Main Memory Segment 1 is detected
//!        - \b MPU_SEG_2_ACCESS_VIOLATION - is set if an access violation in
//!           Main Memory Segment 2 is detected
//!        - \b MPU_SEG_3_ACCESS_VIOLATION - is set if an access violation in
//!           Main Memory Segment 3 is detected
//!        - \b MPU_SEG_INFO_ACCESS_VIOLATION - is set if an access violation
//!           in User Information Memory Segment is detected
//!
//! \return Logical OR of any of the following:
//!         - \b MPU_SEG_1_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 1 is detected
//!         - \b MPU_SEG_2_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 2 is detected
//!         - \b MPU_SEG_3_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 3 is detected
//!         - \b MPU_SEG_INFO_ACCESS_VIOLATION is set if an access violation in
//!         User Information Memory Segment is detected
//!         \n indicating the status of the masked flags.
//
//*****************************************************************************
uint16_t MPU_clearInterruptFlag(uint16_t baseAddress,
        uint16_t memAccFlag
    )
{

    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | HWREG8(baseAddress + OFS_MPUCTL0);
    HWREG16(baseAddress + OFS_MPUCTL1) &= ~memAccFlag;

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;

    return (HWREG16(baseAddress + OFS_MPUCTL1) & memAccFlag);
}

//*****************************************************************************
//
//! \brief Clears all Memory Segment Access Violation Interrupt Flags.
//!
//! \param baseAddress is the base address of the MPU module.
//!
//! Modified bits of \b MPUCTL1 register.
//!
//! \return Logical OR of any of the following:
//!         - \b MPU_SEG_1_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 1 is detected
//!         - \b MPU_SEG_2_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 2 is detected
//!         - \b MPU_SEG_3_ACCESS_VIOLATION is set if an access violation in
//!         Main Memory Segment 3 is detected
//!         - \b MPU_SEG_INFO_ACCESS_VIOLATION is set if an access violation in
//!         User Information Memory Segment is detected
//!         \n indicating the status of the interrupt flags.
//
//*****************************************************************************
uint16_t MPU_clearAllInterruptFlags(uint16_t baseAddress
    )
{

    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | HWREG8(baseAddress + OFS_MPUCTL0);
    HWREG16(baseAddress + OFS_MPUCTL1) &= ~(MPUSEG1IFG + MPUSEG2IFG + MPUSEG3IFG);

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
    
    return (HWREG16(baseAddress + OFS_MPUCTL1) & (MPUSEG1IFG + MPUSEG2IFG + MPUSEG3IFG));
}

//*****************************************************************************
//
//! \brief Lock MPU to protect from write access.
//!
//! Sets MPULOCK to protect MPU from write access on all MPU registers except
//! MPUCTL1, MPUIPC0 and MPUIPSEGBx until a BOR occurs. MPULOCK bit cannot be
//! cleared manually. MPU_clearInterruptFlag() and MPU_clearAllInterruptFlags()
//! still can be used after this API is called.
//!
//! \param baseAddress is the base address of the MPU module.
//!
//! Modified bits are \b MPULOCK of \b MPUCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void MPU_lockMPU(uint16_t baseAddress)
{
    HWREG16(baseAddress + OFS_MPUCTL0) = MPUPW | MPULOCK | 
                                            HWREG8(baseAddress + OFS_MPUCTL0);

    //Lock MPU to disable writing to all registers
    HWREG8(baseAddress + OFS_MPUCTL0_H) = 0x00;
}


#endif
//*****************************************************************************
//
//! Close the doxygen group for mpu_api
//! @}
//
//*****************************************************************************
