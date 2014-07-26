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
// lcd_c.c - Driver for the lcd_c Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup lcd_c_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_LCD_C__
#include "lcd_c.h"

#include <assert.h>

//*****************************************************************************
//
// Initialization parameter instance
//
//*****************************************************************************
const LCD_C_initParam LCD_C_INIT_PARAM = {
        LCD_C_CLOCKSOURCE_ACLK,
        LCD_C_CLOLKDIVIDER_1,
        LCD_C_CLOCKPRESCALAR_1,
        LCD_C_STATIC,
        LCD_C_STANDARD_WAVEFORMS,
        LCD_C_SEGMENTS_DISABLED
};

//*****************************************************************************
//
//! \brief internal function to set pin as LCD function.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param index indicates which LCDPCTL to be set.
//! \param value is set to corresponding LCDPCTL.
//!
//
//*****************************************************************************
static void setLCDFunction(uint16_t baseAddress, uint8_t index, uint16_t value)
{
        switch (index) {
        case 0:
                HWREG16(baseAddress + OFS_LCDCPCTL0) |= value;
                break;
        case 1:
                HWREG16(baseAddress + OFS_LCDCPCTL1) |= value;
                break;
        case 2:
                HWREG16(baseAddress + OFS_LCDCPCTL2) |= value;
                break;
        case 3:
                HWREG16(baseAddress + OFS_LCDCPCTL3) |= value;
                break;
        default: break;
        }
}

//*****************************************************************************
//
//! \brief Initializes the LCD Module.
//!
//! his function initializes the LCD but without turning on. It bascially setup
//! the clock source, clock divider, clock prescalar, mux rate, low-power
//! waveform and segments on/off. After calling this function, user can config
//! charge pump, internal reference voltage and voltage sources.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param initParams is the pointer to LCD_InitParam structure. See the
//!        following parameters for each field.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_init(uint16_t baseAddress, LCD_C_initParam *initParams)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~(LCDMX0 | LCDMX1 | LCDMX2 | LCDSSEL
                                                 | LCDLP | LCDSON | LCDDIV_31);

        HWREG16(baseAddress + OFS_LCDCCTL0) |= initParams->muxRate;
        HWREG16(baseAddress + OFS_LCDCCTL0) |= initParams->clockSource;
        HWREG16(baseAddress + OFS_LCDCCTL0) |= initParams->waveforms;
        HWREG16(baseAddress + OFS_LCDCCTL0) |= initParams->segments;
        HWREG16(baseAddress + OFS_LCDCCTL0) |= initParams->clockDivider;
        HWREG16(baseAddress + OFS_LCDCCTL0) |= initParams->clockPrescalar;
}

//*****************************************************************************
//
//! \brief Turns on the LCD module.
//!
//! \param baseAddress is the base address of the LCD_C module.
//!
//! Modified bits are \b LCDON of \b LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_on(uint16_t baseAddress)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) |= LCDON;
}

//*****************************************************************************
//
//! \brief Turns off the LCD module.
//!
//! \param baseAddress is the base address of the LCD_C module.
//!
//! Modified bits are \b LCDON of \b LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_off(uint16_t baseAddress)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;
}

//*****************************************************************************
//
//! \brief Clears the LCD interrupt flags.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param mask is the masked interrupt flag to be cleared.
//!        Valid values are:
//!        - \b LCD_C_NO_CAPACITANCE_CONNECTED_INTERRUPT
//!        - \b LCD_C_BLINKING_SEGMENTS_ON_INTERRUPT
//!        - \b LCD_C_BLINKING_SEGMENTS_OFF_INTERRUPT
//!        - \b LCD_C_FRAME_INTERRUPT
//!        \n Modified bits are \b LCDCAPIFG, \b LCDBLKONIFG, \b LCDBLKOFFIFG
//!        and \b LCDFRMIFG of \b LCDCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_clearInterrupt(uint16_t baseAddress, uint16_t mask)
{
        HWREG8(baseAddress + OFS_LCDCCTL1_L) &= ~(mask >> 8);
}

//*****************************************************************************
//
//! \brief Gets the LCD interrupt status.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param mask is the masked interrupt flags.
//!        Valid values are:
//!        - \b LCD_C_NO_CAPACITANCE_CONNECTED_INTERRUPT
//!        - \b LCD_C_BLINKING_SEGMENTS_ON_INTERRUPT
//!        - \b LCD_C_BLINKING_SEGMENTS_OFF_INTERRUPT
//!        - \b LCD_C_FRAME_INTERRUPT
//!
//! \return None
//!         Return Logical OR of any of the following:
//!         - \b LCD_C_NO_CAPACITANCE_CONNECTED_INTERRUPT
//!         - \b LCD_C_BLINKING_SEGMENTS_ON_INTERRUPT
//!         - \b LCD_C_BLINKING_SEGMENTS_OFF_INTERRUPT
//!         - \b LCD_C_FRAME_INTERRUPT
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint16_t LCD_C_getInterruptStatus(uint16_t baseAddress, uint16_t mask)
{
        return HWREG8(baseAddress + OFS_LCDCCTL1_L) & (mask >> 8);
}

//*****************************************************************************
//
//! \brief Enables LCD interrupt sources.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param mask is the interrupts to be enabled.
//!        Valid values are:
//!        - \b LCD_C_NO_CAPACITANCE_CONNECTED_INTERRUPT
//!        - \b LCD_C_BLINKING_SEGMENTS_ON_INTERRUPT
//!        - \b LCD_C_BLINKING_SEGMENTS_OFF_INTERRUPT
//!        - \b LCD_C_FRAME_INTERRUPT
//!        \n Modified bits are \b LCDCAPIE, \b LCDBLKONIE, \b LCDBLKOFFIE and
//!        \b LCDFRMIE of \b LCDCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_enableInterrupt(uint16_t baseAddress, uint16_t mask)
{
        HWREG16(baseAddress + OFS_LCDCCTL1) |= mask;
}

//*****************************************************************************
//
//! \brief Disables LCD interrupt sources.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param mask is the interrupts to be disabled.
//!        Valid values are:
//!        - \b LCD_C_NO_CAPACITANCE_CONNECTED_INTERRUPT
//!        - \b LCD_C_BLINKING_SEGMENTS_ON_INTERRUPT
//!        - \b LCD_C_BLINKING_SEGMENTS_OFF_INTERRUPT
//!        - \b LCD_C_FRAME_INTERRUPT
//!        \n Modified bits are \b LCDCAPIE, \b LCDBLKONIE, \b LCDBLKOFFIE and
//!        \b LCDFRMIE of \b LCDCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_disableInterrupt(uint16_t baseAddress, uint16_t mask)
{
        HWREG16(baseAddress + OFS_LCDCCTL1) &= ~mask;
}

//*****************************************************************************
//
//! \brief Clears all LCD memory registers.
//!
//! \param baseAddress is the base address of the LCD_C module.
//!
//! Modified bits are \b LCDCLRM of \b LCDMEMCTL register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_clearMemory(uint16_t baseAddress)
{
        HWREG8(baseAddress + OFS_LCDCMEMCTL) |= LCDCLRM;
}

//*****************************************************************************
//
//! \brief Clears all LCD blinking memory registers.
//!
//! \param baseAddress is the base address of the LCD_C module.
//!
//! Modified bits are \b LCDCLRBM of \b LCDMEMCTL register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_clearBlinkingMemory(uint16_t baseAddress)
{
        HWREG8(baseAddress + OFS_LCDCMEMCTL) |= LCDCLRBM;
}

//*****************************************************************************
//
//! \brief Selects display memory.
//!
//! This function selects display memory either from memory or blinking memory.
//! Please note if the blinking mode is selected as
//! LCD_BLINKMODE_INDIVIDUALSEGMENTS or LCD_BLINKMODE_ALLSEGMENTS or mux rate
//! >=5, display memory can not be changed. If
//! LCD_BLINKMODE_SWITCHDISPLAYCONTENTS is selected, display memory bit
//! reflects current displayed memory.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param displayMemory is the desired displayed memory.
//!        Valid values are:
//!        - \b LCD_C_DISPLAYSOURCE_MEMORY  [Default]
//!        - \b LCD_C_DISPLAYSOURCE_BLINKINGMEMORY
//!        \n Modified bits are \b LCDDISP of \b LCDMEMCTL register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_selectDisplayMemory(uint16_t baseAddress, uint16_t displayMemory)
{
        HWREG16(baseAddress + OFS_LCDCMEMCTL) &= ~LCDDISP;
        HWREG16(baseAddress + OFS_LCDCMEMCTL) |= displayMemory;
}

//*****************************************************************************
//
//! \brief Sets the blink settings.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param clockDivider is the clock divider for blinking frequency.
//!        Valid values are:
//!        - \b LCD_C_BLINK_FREQ_CLOCK_DIVIDER_1 [Default]
//!        - \b LCD_C_BLINK_FREQ_CLOCK_DIVIDER_2
//!        - \b LCD_C_BLINK_FREQ_CLOCK_DIVIDER_3
//!        - \b LCD_C_BLINK_FREQ_CLOCK_DIVIDER_4
//!        - \b LCD_C_BLINK_FREQ_CLOCK_DIVIDER_5
//!        - \b LCD_C_BLINK_FREQ_CLOCK_DIVIDER_6
//!        - \b LCD_C_BLINK_FREQ_CLOCK_DIVIDER_7
//!        - \b LCD_C_BLINK_FREQ_CLOCK_DIVIDER_8
//!        \n Modified bits are \b LCDBLKDIVx of \b LCDBLKCTL register.
//! \param clockPrescalar is the clock pre-scalar for blinking frequency.
//!        Valid values are:
//!        - \b LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_512 [Default]
//!        - \b LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_1024
//!        - \b LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_2048
//!        - \b LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_4096
//!        - \b LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_8162
//!        - \b LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_16384
//!        - \b LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_32768
//!        - \b LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_65536
//!        \n Modified bits are \b LCDBLKPREx of \b LCDBLKCTL register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setBlinkingControl(uint16_t baseAddress,
                              uint8_t clockDivider,
                              uint8_t clockPrescalar,
                              uint8_t mode)
{
        HWREG8(baseAddress + OFS_LCDCBLKCTL) &= ~(LCDBLKDIV0 | LCDBLKDIV1 | LCDBLKDIV2 |
                                                  LCDBLKPRE0 | LCDBLKPRE1 | LCDBLKPRE2 |
                                                  LCDBLKMOD0 | LCDBLKMOD1
                                                  );
        HWREG8(baseAddress + OFS_LCDCBLKCTL) |= clockDivider | clockPrescalar | mode;
}

//*****************************************************************************
//
//! \brief Enables the charge pump.
//!
//! \param baseAddress is the base address of the LCD_C module.
//!
//! Modified bits are \b LCDCPEN of \b LCDVCTL register; bits \b LCDON of \b
//! LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_enableChargePump(uint16_t baseAddress)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;
        HWREG8(baseAddress + OFS_LCDCVCTL) |= LCDCPEN;
}

//*****************************************************************************
//
//! \brief Disables the charge pump.
//!
//! \param baseAddress is the base address of the LCD_C module.
//!
//! Modified bits are \b LCDCPEN of \b LCDVCTL register; bits \b LCDON of \b
//! LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_disableChargePump(uint16_t baseAddress)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;
        HWREG8(baseAddress + OFS_LCDCVCTL) &= ~LCDCPEN;
}

//*****************************************************************************
//
//! \brief Selects the bias level.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param bias is the select for bias level.
//!        Valid values are:
//!        - \b LCD_C_BIAS_1_3 [Default] - 1/3 bias
//!        - \b LCD_C_BIAS_1_2 - 1/2 bias
//!
//! Modified bits are \b LCD2B of \b LCDVCTL register; bits \b LCDON of \b
//! LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_selectBias(uint16_t baseAddress, uint16_t bias)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;
        HWREG8(baseAddress + OFS_LCDCVCTL) &= ~LCD2B;

        HWREG8(baseAddress + OFS_LCDCVCTL) |= bias;
}

//*****************************************************************************
//
//! \brief Selects the charge pump reference.
//!
//! The charge pump reference does not support
//! LCD_C_EXTERNAL_REFERENCE_VOLTAGE,
//! LCD_C_INTERNAL_REFERNCE_VOLTAGE_SWITCHED_TO_EXTERNAL_PIN when
//! LCD_C_V2V3V4_SOURCED_EXTERNALLY or
//! LCD_C_V2V3V4_GENERATED_INTERNALLY_SWITCHED_TO_PINS is selected.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param reference is the select for charge pump reference.
//!        Valid values are:
//!        - \b LCD_C_INTERNAL_REFERNCE_VOLTAGE [Default]
//!        - \b LCD_C_EXTERNAL_REFERENCE_VOLTAGE
//!        - \b LCD_C_INTERNAL_REFERNCE_VOLTAGE_SWITCHED_TO_EXTERNAL_PIN
//!
//! Modified bits are \b VLCDREFx of \b LCDVCTL register; bits \b LCDON of \b
//! LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_selectChargePumpReference(uint16_t baseAddress, uint16_t reference)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;
        HWREG16(baseAddress + OFS_LCDCVCTL) &= ~VLCDREF_3;

        HWREG16(baseAddress + OFS_LCDCVCTL) |= reference;
}

//*****************************************************************************
//
//! \brief Sets the voltage source for V2/V3/V4 and V5.
//!
//! The charge pump reference does not support
//! LCD_C_EXTERNAL_REFERENCE_VOLTAGE,
//! LCD_C_INTERNAL_REFERNCE_VOLTAGE_SWITCHED_TO_EXTERNAL_PIN when
//! LCD_C_V2V3V4_SOURCED_EXTERNALLY or
//! LCD_C_V2V3V4_GENERATED_INTERNALLY_SWITCHED_TO_PINS is selected.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param vlcdSource is the V(LCD) source select.
//!        Valid values are:
//!        - \b LCD_C_VLCD_GENERATED_INTERNALLY [Default]
//!        - \b LCD_C_VLCD_SOURCED_EXTERNALLY
//! \param v2v3v4Source is the V2/V3/V4 source select.
//!        Valid values are:
//!        - \b LCD_C_V2V3V4_GENERATED_INTERNALLY_NOT_SWITCHED_TO_PINS
//!           [Default]
//!        - \b LCD_C_V2V3V4_GENERATED_INTERNALLY_SWITCHED_TO_PINS
//!        - \b LCD_C_V2V3V4_SOURCED_EXTERNALLY
//! \param v5Source is the V5 source select.
//!        Valid values are:
//!        - \b LCD_C_V5_VSS [Default]
//!        - \b LCD_C_V5_SOURCED_FROM_R03
//!
//! Modified bits are \b VLCDEXT, \b LCDREXT, \b LCDEXTBIAS and \b R03EXT of \b
//! LCDVCTL register; bits \b LCDON of \b LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setVLCDSource(uint16_t baseAddress, uint16_t vlcdSource,
                         uint16_t v2v3v4Source,
                         uint16_t v5Source)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;
        HWREG16(baseAddress + OFS_LCDCVCTL) &= ~VLCDEXT;
        HWREG16(baseAddress + OFS_LCDCVCTL) &= ~LCDREXT;
        HWREG16(baseAddress + OFS_LCDCVCTL) &= ~LCDEXTBIAS;
        HWREG16(baseAddress + OFS_LCDCVCTL) &= ~R03EXT;

        HWREG16(baseAddress + OFS_LCDCVCTL) |= vlcdSource;
        HWREG16(baseAddress + OFS_LCDCVCTL) |= v2v3v4Source;
        HWREG16(baseAddress + OFS_LCDCVCTL) |= v5Source;
}

//*****************************************************************************
//
//! \brief Selects the charge pump reference.
//!
//! Sets LCD charge pump voltage.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param voltage is the charge pump select.
//!        Valid values are:
//!        - \b LCD_C_CHARGEPUMP_DISABLED [Default]
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_2_60V_OR_2_17VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_2_66V_OR_2_22VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_2_72V_OR_2_27VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_2_78V_OR_2_32VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_2_84V_OR_2_37VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_2_90V_OR_2_42VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_2_96V_OR_2_47VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_3_02V_OR_2_52VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_3_08V_OR_2_57VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_3_14V_OR_2_62VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_3_20V_OR_2_67VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_3_26V_OR_2_72VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_3_32V_OR_2_77VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_3_38V_OR_2_82VREF
//!        - \b LCD_C_CHARGEPUMP_VOLTAGE_3_44V_OR_2_87VREF
//!
//! Modified bits are \b VLCDx of \b LCDVCTL register; bits \b LCDON of \b
//! LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setVLCDVoltage(uint16_t baseAddress, uint16_t voltage)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;
        HWREG8(baseAddress + OFS_LCDCVCTL) &= ~VLCD_15;

        HWREG16(baseAddress + OFS_LCDCVCTL) |= voltage;
}

//*****************************************************************************
//
//! \brief Sets the LCD Pin as LCD functions.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param pin is the select pin set as LCD function.
//!        Valid values are:
//!        - \b LCD_C_SEGMENT_LINE_0
//!        - \b LCD_C_SEGMENT_LINE_1
//!        - \b LCD_C_SEGMENT_LINE_2
//!        - \b LCD_C_SEGMENT_LINE_3
//!        - \b LCD_C_SEGMENT_LINE_4
//!        - \b LCD_C_SEGMENT_LINE_5
//!        - \b LCD_C_SEGMENT_LINE_6
//!        - \b LCD_C_SEGMENT_LINE_7
//!        - \b LCD_C_SEGMENT_LINE_8
//!        - \b LCD_C_SEGMENT_LINE_9
//!        - \b LCD_C_SEGMENT_LINE_10
//!        - \b LCD_C_SEGMENT_LINE_11
//!        - \b LCD_C_SEGMENT_LINE_12
//!        - \b LCD_C_SEGMENT_LINE_13
//!        - \b LCD_C_SEGMENT_LINE_14
//!        - \b LCD_C_SEGMENT_LINE_15
//!        - \b LCD_C_SEGMENT_LINE_16
//!        - \b LCD_C_SEGMENT_LINE_17
//!        - \b LCD_C_SEGMENT_LINE_18
//!        - \b LCD_C_SEGMENT_LINE_19
//!        - \b LCD_C_SEGMENT_LINE_20
//!        - \b LCD_C_SEGMENT_LINE_21
//!        - \b LCD_C_SEGMENT_LINE_22
//!        - \b LCD_C_SEGMENT_LINE_23
//!        - \b LCD_C_SEGMENT_LINE_24
//!        - \b LCD_C_SEGMENT_LINE_25
//!        - \b LCD_C_SEGMENT_LINE_26
//!        - \b LCD_C_SEGMENT_LINE_27
//!        - \b LCD_C_SEGMENT_LINE_28
//!        - \b LCD_C_SEGMENT_LINE_29
//!        - \b LCD_C_SEGMENT_LINE_30
//!        - \b LCD_C_SEGMENT_LINE_31
//!        - \b LCD_C_SEGMENT_LINE_32
//!        - \b LCD_C_SEGMENT_LINE_33
//!        - \b LCD_C_SEGMENT_LINE_34
//!        - \b LCD_C_SEGMENT_LINE_35
//!        - \b LCD_C_SEGMENT_LINE_36
//!        - \b LCD_C_SEGMENT_LINE_37
//!        - \b LCD_C_SEGMENT_LINE_38
//!        - \b LCD_C_SEGMENT_LINE_39
//!        - \b LCD_C_SEGMENT_LINE_40
//!        - \b LCD_C_SEGMENT_LINE_41
//!        - \b LCD_C_SEGMENT_LINE_42
//!        - \b LCD_C_SEGMENT_LINE_43
//!        - \b LCD_C_SEGMENT_LINE_44
//!        - \b LCD_C_SEGMENT_LINE_45
//!        - \b LCD_C_SEGMENT_LINE_46
//!        - \b LCD_C_SEGMENT_LINE_47
//!        - \b LCD_C_SEGMENT_LINE_48
//!        - \b LCD_C_SEGMENT_LINE_49
//!        - \b LCD_C_SEGMENT_LINE_50
//!        - \b LCD_C_SEGMENT_LINE_51
//!        - \b LCD_C_SEGMENT_LINE_52
//!        - \b LCD_C_SEGMENT_LINE_53
//!        - \b LCD_C_SEGMENT_LINE_54
//!        - \b LCD_C_SEGMENT_LINE_55
//!        - \b LCD_C_SEGMENT_LINE_56
//!        - \b LCD_C_SEGMENT_LINE_57
//!        - \b LCD_C_SEGMENT_LINE_58
//!        - \b LCD_C_SEGMENT_LINE_59
//!        - \b LCD_C_SEGMENT_LINE_60
//!        - \b LCD_C_SEGMENT_LINE_61
//!        - \b LCD_C_SEGMENT_LINE_62
//!        - \b LCD_C_SEGMENT_LINE_63
//!
//! Modified bits are \b LCDSx of \b LCDPCTLx register; bits \b LCDON of \b
//! LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setPinAsLCDFunction(uint16_t baseAddress, uint8_t pin)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;

        uint8_t idx = pin >> 4;
        uint16_t val = 1 << (pin & 0xF);

        setLCDFunction(baseAddress, idx, val);
}

//*****************************************************************************
//
//! \brief Sets the LCD Pin as Port functions.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param pin is the select pin set as Port function.
//!        Valid values are:
//!        - \b LCD_C_SEGMENT_LINE_0
//!        - \b LCD_C_SEGMENT_LINE_1
//!        - \b LCD_C_SEGMENT_LINE_2
//!        - \b LCD_C_SEGMENT_LINE_3
//!        - \b LCD_C_SEGMENT_LINE_4
//!        - \b LCD_C_SEGMENT_LINE_5
//!        - \b LCD_C_SEGMENT_LINE_6
//!        - \b LCD_C_SEGMENT_LINE_7
//!        - \b LCD_C_SEGMENT_LINE_8
//!        - \b LCD_C_SEGMENT_LINE_9
//!        - \b LCD_C_SEGMENT_LINE_10
//!        - \b LCD_C_SEGMENT_LINE_11
//!        - \b LCD_C_SEGMENT_LINE_12
//!        - \b LCD_C_SEGMENT_LINE_13
//!        - \b LCD_C_SEGMENT_LINE_14
//!        - \b LCD_C_SEGMENT_LINE_15
//!        - \b LCD_C_SEGMENT_LINE_16
//!        - \b LCD_C_SEGMENT_LINE_17
//!        - \b LCD_C_SEGMENT_LINE_18
//!        - \b LCD_C_SEGMENT_LINE_19
//!        - \b LCD_C_SEGMENT_LINE_20
//!        - \b LCD_C_SEGMENT_LINE_21
//!        - \b LCD_C_SEGMENT_LINE_22
//!        - \b LCD_C_SEGMENT_LINE_23
//!        - \b LCD_C_SEGMENT_LINE_24
//!        - \b LCD_C_SEGMENT_LINE_25
//!        - \b LCD_C_SEGMENT_LINE_26
//!        - \b LCD_C_SEGMENT_LINE_27
//!        - \b LCD_C_SEGMENT_LINE_28
//!        - \b LCD_C_SEGMENT_LINE_29
//!        - \b LCD_C_SEGMENT_LINE_30
//!        - \b LCD_C_SEGMENT_LINE_31
//!        - \b LCD_C_SEGMENT_LINE_32
//!        - \b LCD_C_SEGMENT_LINE_33
//!        - \b LCD_C_SEGMENT_LINE_34
//!        - \b LCD_C_SEGMENT_LINE_35
//!        - \b LCD_C_SEGMENT_LINE_36
//!        - \b LCD_C_SEGMENT_LINE_37
//!        - \b LCD_C_SEGMENT_LINE_38
//!        - \b LCD_C_SEGMENT_LINE_39
//!        - \b LCD_C_SEGMENT_LINE_40
//!        - \b LCD_C_SEGMENT_LINE_41
//!        - \b LCD_C_SEGMENT_LINE_42
//!        - \b LCD_C_SEGMENT_LINE_43
//!        - \b LCD_C_SEGMENT_LINE_44
//!        - \b LCD_C_SEGMENT_LINE_45
//!        - \b LCD_C_SEGMENT_LINE_46
//!        - \b LCD_C_SEGMENT_LINE_47
//!        - \b LCD_C_SEGMENT_LINE_48
//!        - \b LCD_C_SEGMENT_LINE_49
//!        - \b LCD_C_SEGMENT_LINE_50
//!        - \b LCD_C_SEGMENT_LINE_51
//!        - \b LCD_C_SEGMENT_LINE_52
//!        - \b LCD_C_SEGMENT_LINE_53
//!        - \b LCD_C_SEGMENT_LINE_54
//!        - \b LCD_C_SEGMENT_LINE_55
//!        - \b LCD_C_SEGMENT_LINE_56
//!        - \b LCD_C_SEGMENT_LINE_57
//!        - \b LCD_C_SEGMENT_LINE_58
//!        - \b LCD_C_SEGMENT_LINE_59
//!        - \b LCD_C_SEGMENT_LINE_60
//!        - \b LCD_C_SEGMENT_LINE_61
//!        - \b LCD_C_SEGMENT_LINE_62
//!        - \b LCD_C_SEGMENT_LINE_63
//!
//! Modified bits are \b LCDSx of \b LCDPCTLx register; bits \b LCDON of \b
//! LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setPinAsPortFunction(uint16_t baseAddress, uint8_t pin)
{
        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;

        uint8_t idx = pin >> 4;
        uint16_t val = 1 << (pin & 0xF);

        switch (idx) {
        case 0:
                HWREG16(baseAddress + OFS_LCDCPCTL0) &= ~val;
                break;
        case 1:
                HWREG16(baseAddress + OFS_LCDCPCTL1) &= ~val;
                break;
        case 2:
                HWREG16(baseAddress + OFS_LCDCPCTL2) &= ~val;
                break;
        case 3:
                HWREG16(baseAddress + OFS_LCDCPCTL3) &= ~val;
                break;
        default: break;
        }
}

//*****************************************************************************
//
//! \brief Sets the LCD pins as LCD function pin.
//!
//! This function sets the LCD pins as LCD function pin. Instead of passing the
//! all the possible pins, it just requires the start pin and the end pin.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param startPin is the starting pin to be configed as LCD function pin.
//!        Valid values are:
//!        - \b LCD_C_SEGMENT_LINE_0
//!        - \b LCD_C_SEGMENT_LINE_1
//!        - \b LCD_C_SEGMENT_LINE_2
//!        - \b LCD_C_SEGMENT_LINE_3
//!        - \b LCD_C_SEGMENT_LINE_4
//!        - \b LCD_C_SEGMENT_LINE_5
//!        - \b LCD_C_SEGMENT_LINE_6
//!        - \b LCD_C_SEGMENT_LINE_7
//!        - \b LCD_C_SEGMENT_LINE_8
//!        - \b LCD_C_SEGMENT_LINE_9
//!        - \b LCD_C_SEGMENT_LINE_10
//!        - \b LCD_C_SEGMENT_LINE_11
//!        - \b LCD_C_SEGMENT_LINE_12
//!        - \b LCD_C_SEGMENT_LINE_13
//!        - \b LCD_C_SEGMENT_LINE_14
//!        - \b LCD_C_SEGMENT_LINE_15
//!        - \b LCD_C_SEGMENT_LINE_16
//!        - \b LCD_C_SEGMENT_LINE_17
//!        - \b LCD_C_SEGMENT_LINE_18
//!        - \b LCD_C_SEGMENT_LINE_19
//!        - \b LCD_C_SEGMENT_LINE_20
//!        - \b LCD_C_SEGMENT_LINE_21
//!        - \b LCD_C_SEGMENT_LINE_22
//!        - \b LCD_C_SEGMENT_LINE_23
//!        - \b LCD_C_SEGMENT_LINE_24
//!        - \b LCD_C_SEGMENT_LINE_25
//!        - \b LCD_C_SEGMENT_LINE_26
//!        - \b LCD_C_SEGMENT_LINE_27
//!        - \b LCD_C_SEGMENT_LINE_28
//!        - \b LCD_C_SEGMENT_LINE_29
//!        - \b LCD_C_SEGMENT_LINE_30
//!        - \b LCD_C_SEGMENT_LINE_31
//!        - \b LCD_C_SEGMENT_LINE_32
//!        - \b LCD_C_SEGMENT_LINE_33
//!        - \b LCD_C_SEGMENT_LINE_34
//!        - \b LCD_C_SEGMENT_LINE_35
//!        - \b LCD_C_SEGMENT_LINE_36
//!        - \b LCD_C_SEGMENT_LINE_37
//!        - \b LCD_C_SEGMENT_LINE_38
//!        - \b LCD_C_SEGMENT_LINE_39
//!        - \b LCD_C_SEGMENT_LINE_40
//!        - \b LCD_C_SEGMENT_LINE_41
//!        - \b LCD_C_SEGMENT_LINE_42
//!        - \b LCD_C_SEGMENT_LINE_43
//!        - \b LCD_C_SEGMENT_LINE_44
//!        - \b LCD_C_SEGMENT_LINE_45
//!        - \b LCD_C_SEGMENT_LINE_46
//!        - \b LCD_C_SEGMENT_LINE_47
//!        - \b LCD_C_SEGMENT_LINE_48
//!        - \b LCD_C_SEGMENT_LINE_49
//!        - \b LCD_C_SEGMENT_LINE_50
//!        - \b LCD_C_SEGMENT_LINE_51
//!        - \b LCD_C_SEGMENT_LINE_52
//!        - \b LCD_C_SEGMENT_LINE_53
//!        - \b LCD_C_SEGMENT_LINE_54
//!        - \b LCD_C_SEGMENT_LINE_55
//!        - \b LCD_C_SEGMENT_LINE_56
//!        - \b LCD_C_SEGMENT_LINE_57
//!        - \b LCD_C_SEGMENT_LINE_58
//!        - \b LCD_C_SEGMENT_LINE_59
//!        - \b LCD_C_SEGMENT_LINE_60
//!        - \b LCD_C_SEGMENT_LINE_61
//!        - \b LCD_C_SEGMENT_LINE_62
//!        - \b LCD_C_SEGMENT_LINE_63
//! \param endPin is the ending pin to be configed as LCD function pin.
//!        Valid values are:
//!        - \b LCD_C_SEGMENT_LINE_0
//!        - \b LCD_C_SEGMENT_LINE_1
//!        - \b LCD_C_SEGMENT_LINE_2
//!        - \b LCD_C_SEGMENT_LINE_3
//!        - \b LCD_C_SEGMENT_LINE_4
//!        - \b LCD_C_SEGMENT_LINE_5
//!        - \b LCD_C_SEGMENT_LINE_6
//!        - \b LCD_C_SEGMENT_LINE_7
//!        - \b LCD_C_SEGMENT_LINE_8
//!        - \b LCD_C_SEGMENT_LINE_9
//!        - \b LCD_C_SEGMENT_LINE_10
//!        - \b LCD_C_SEGMENT_LINE_11
//!        - \b LCD_C_SEGMENT_LINE_12
//!        - \b LCD_C_SEGMENT_LINE_13
//!        - \b LCD_C_SEGMENT_LINE_14
//!        - \b LCD_C_SEGMENT_LINE_15
//!        - \b LCD_C_SEGMENT_LINE_16
//!        - \b LCD_C_SEGMENT_LINE_17
//!        - \b LCD_C_SEGMENT_LINE_18
//!        - \b LCD_C_SEGMENT_LINE_19
//!        - \b LCD_C_SEGMENT_LINE_20
//!        - \b LCD_C_SEGMENT_LINE_21
//!        - \b LCD_C_SEGMENT_LINE_22
//!        - \b LCD_C_SEGMENT_LINE_23
//!        - \b LCD_C_SEGMENT_LINE_24
//!        - \b LCD_C_SEGMENT_LINE_25
//!        - \b LCD_C_SEGMENT_LINE_26
//!        - \b LCD_C_SEGMENT_LINE_27
//!        - \b LCD_C_SEGMENT_LINE_28
//!        - \b LCD_C_SEGMENT_LINE_29
//!        - \b LCD_C_SEGMENT_LINE_30
//!        - \b LCD_C_SEGMENT_LINE_31
//!        - \b LCD_C_SEGMENT_LINE_32
//!        - \b LCD_C_SEGMENT_LINE_33
//!        - \b LCD_C_SEGMENT_LINE_34
//!        - \b LCD_C_SEGMENT_LINE_35
//!        - \b LCD_C_SEGMENT_LINE_36
//!        - \b LCD_C_SEGMENT_LINE_37
//!        - \b LCD_C_SEGMENT_LINE_38
//!        - \b LCD_C_SEGMENT_LINE_39
//!        - \b LCD_C_SEGMENT_LINE_40
//!        - \b LCD_C_SEGMENT_LINE_41
//!        - \b LCD_C_SEGMENT_LINE_42
//!        - \b LCD_C_SEGMENT_LINE_43
//!        - \b LCD_C_SEGMENT_LINE_44
//!        - \b LCD_C_SEGMENT_LINE_45
//!        - \b LCD_C_SEGMENT_LINE_46
//!        - \b LCD_C_SEGMENT_LINE_47
//!        - \b LCD_C_SEGMENT_LINE_48
//!        - \b LCD_C_SEGMENT_LINE_49
//!        - \b LCD_C_SEGMENT_LINE_50
//!        - \b LCD_C_SEGMENT_LINE_51
//!        - \b LCD_C_SEGMENT_LINE_52
//!        - \b LCD_C_SEGMENT_LINE_53
//!        - \b LCD_C_SEGMENT_LINE_54
//!        - \b LCD_C_SEGMENT_LINE_55
//!        - \b LCD_C_SEGMENT_LINE_56
//!        - \b LCD_C_SEGMENT_LINE_57
//!        - \b LCD_C_SEGMENT_LINE_58
//!        - \b LCD_C_SEGMENT_LINE_59
//!        - \b LCD_C_SEGMENT_LINE_60
//!        - \b LCD_C_SEGMENT_LINE_61
//!        - \b LCD_C_SEGMENT_LINE_62
//!        - \b LCD_C_SEGMENT_LINE_63
//!
//! Modified bits are \b LCDSx of \b LCDPCTLx register; bits \b LCDON of \b
//! LCDCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setPinAsLCDFunctionEx(uint16_t baseAddress, uint8_t startPin,
                                 uint8_t endPin)
{
        uint8_t startIdx = startPin >> 4;
        uint8_t endIdx = endPin >> 4;
        uint8_t startPos = startPin & 0xF;
        uint8_t endPos = endPin & 0xF;
        uint16_t val = 0;
        uint8_t i = 0;

        HWREG16(baseAddress + OFS_LCDCCTL0) &= ~LCDON;

        if (startIdx == endIdx) {
                val = (0xFFFF >> (15 - endPos)) & (0xFFFF << startPos);

                setLCDFunction(baseAddress, startIdx, val);

        }else  {
                val = 0xFFFF >> (15 - endPos);
                setLCDFunction(baseAddress, endIdx, val);

                for (i = endIdx - 1; i > startIdx; i--)
                        setLCDFunction(baseAddress, i, 0xFFFF);

                val = 0xFFFF << startPos;
                setLCDFunction(baseAddress, startIdx, val);
        }
}

//*****************************************************************************
//
//! \brief Sets the LCD memory register.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param pin is the select pin for setting value.
//!        Valid values are:
//!        - \b LCD_C_SEGMENT_LINE_0
//!        - \b LCD_C_SEGMENT_LINE_1
//!        - \b LCD_C_SEGMENT_LINE_2
//!        - \b LCD_C_SEGMENT_LINE_3
//!        - \b LCD_C_SEGMENT_LINE_4
//!        - \b LCD_C_SEGMENT_LINE_5
//!        - \b LCD_C_SEGMENT_LINE_6
//!        - \b LCD_C_SEGMENT_LINE_7
//!        - \b LCD_C_SEGMENT_LINE_8
//!        - \b LCD_C_SEGMENT_LINE_9
//!        - \b LCD_C_SEGMENT_LINE_10
//!        - \b LCD_C_SEGMENT_LINE_11
//!        - \b LCD_C_SEGMENT_LINE_12
//!        - \b LCD_C_SEGMENT_LINE_13
//!        - \b LCD_C_SEGMENT_LINE_14
//!        - \b LCD_C_SEGMENT_LINE_15
//!        - \b LCD_C_SEGMENT_LINE_16
//!        - \b LCD_C_SEGMENT_LINE_17
//!        - \b LCD_C_SEGMENT_LINE_18
//!        - \b LCD_C_SEGMENT_LINE_19
//!        - \b LCD_C_SEGMENT_LINE_20
//!        - \b LCD_C_SEGMENT_LINE_21
//!        - \b LCD_C_SEGMENT_LINE_22
//!        - \b LCD_C_SEGMENT_LINE_23
//!        - \b LCD_C_SEGMENT_LINE_24
//!        - \b LCD_C_SEGMENT_LINE_25
//!        - \b LCD_C_SEGMENT_LINE_26
//!        - \b LCD_C_SEGMENT_LINE_27
//!        - \b LCD_C_SEGMENT_LINE_28
//!        - \b LCD_C_SEGMENT_LINE_29
//!        - \b LCD_C_SEGMENT_LINE_30
//!        - \b LCD_C_SEGMENT_LINE_31
//!        - \b LCD_C_SEGMENT_LINE_32
//!        - \b LCD_C_SEGMENT_LINE_33
//!        - \b LCD_C_SEGMENT_LINE_34
//!        - \b LCD_C_SEGMENT_LINE_35
//!        - \b LCD_C_SEGMENT_LINE_36
//!        - \b LCD_C_SEGMENT_LINE_37
//!        - \b LCD_C_SEGMENT_LINE_38
//!        - \b LCD_C_SEGMENT_LINE_39
//!        - \b LCD_C_SEGMENT_LINE_40
//!        - \b LCD_C_SEGMENT_LINE_41
//!        - \b LCD_C_SEGMENT_LINE_42
//!        - \b LCD_C_SEGMENT_LINE_43
//!        - \b LCD_C_SEGMENT_LINE_44
//!        - \b LCD_C_SEGMENT_LINE_45
//!        - \b LCD_C_SEGMENT_LINE_46
//!        - \b LCD_C_SEGMENT_LINE_47
//!        - \b LCD_C_SEGMENT_LINE_48
//!        - \b LCD_C_SEGMENT_LINE_49
//!        - \b LCD_C_SEGMENT_LINE_50
//!        - \b LCD_C_SEGMENT_LINE_51
//!        - \b LCD_C_SEGMENT_LINE_52
//!        - \b LCD_C_SEGMENT_LINE_53
//!        - \b LCD_C_SEGMENT_LINE_54
//!        - \b LCD_C_SEGMENT_LINE_55
//!        - \b LCD_C_SEGMENT_LINE_56
//!        - \b LCD_C_SEGMENT_LINE_57
//!        - \b LCD_C_SEGMENT_LINE_58
//!        - \b LCD_C_SEGMENT_LINE_59
//!        - \b LCD_C_SEGMENT_LINE_60
//!        - \b LCD_C_SEGMENT_LINE_61
//!        - \b LCD_C_SEGMENT_LINE_62
//!        - \b LCD_C_SEGMENT_LINE_63
//! \param value is the designated value for corresponding pin.
//!
//! Modified bits are \b MBITx of \b LCDMx register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setMemory(uint16_t baseAddress, uint8_t pin, uint8_t value)
{
        uint8_t muxRate = HWREG16(baseAddress + OFS_LCDCCTL0)
                          & (LCDMX2 | LCDMX1 | LCDMX0);

        // static, 2-mux, 3-mux, 4-mux
        if (muxRate <= (LCDMX1 | LCDMX0)) {
                if (pin & 1) {
                        HWREG8(baseAddress + OFS_LCDM1 + pin / 2) &= 0x0F;
                        HWREG8(baseAddress + OFS_LCDM1 + pin / 2) |= (value & 0xF) << 4;
                }else  {
                        HWREG8(baseAddress + OFS_LCDM1 + pin / 2) &= 0xF0;
                        HWREG8(baseAddress + OFS_LCDM1 + pin / 2) |= (value & 0xF);
                }
        }else
                //5-mux, 6-mux, 7-mux, 8-mux
                HWREG8(baseAddress + OFS_LCDM1 + pin) = value;
}

//*****************************************************************************
//
//! \brief Sets the LCD blink memory register.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param pin is the select pin for setting value.
//!        Valid values are:
//!        - \b LCD_C_SEGMENT_LINE_0
//!        - \b LCD_C_SEGMENT_LINE_1
//!        - \b LCD_C_SEGMENT_LINE_2
//!        - \b LCD_C_SEGMENT_LINE_3
//!        - \b LCD_C_SEGMENT_LINE_4
//!        - \b LCD_C_SEGMENT_LINE_5
//!        - \b LCD_C_SEGMENT_LINE_6
//!        - \b LCD_C_SEGMENT_LINE_7
//!        - \b LCD_C_SEGMENT_LINE_8
//!        - \b LCD_C_SEGMENT_LINE_9
//!        - \b LCD_C_SEGMENT_LINE_10
//!        - \b LCD_C_SEGMENT_LINE_11
//!        - \b LCD_C_SEGMENT_LINE_12
//!        - \b LCD_C_SEGMENT_LINE_13
//!        - \b LCD_C_SEGMENT_LINE_14
//!        - \b LCD_C_SEGMENT_LINE_15
//!        - \b LCD_C_SEGMENT_LINE_16
//!        - \b LCD_C_SEGMENT_LINE_17
//!        - \b LCD_C_SEGMENT_LINE_18
//!        - \b LCD_C_SEGMENT_LINE_19
//!        - \b LCD_C_SEGMENT_LINE_20
//!        - \b LCD_C_SEGMENT_LINE_21
//!        - \b LCD_C_SEGMENT_LINE_22
//!        - \b LCD_C_SEGMENT_LINE_23
//!        - \b LCD_C_SEGMENT_LINE_24
//!        - \b LCD_C_SEGMENT_LINE_25
//!        - \b LCD_C_SEGMENT_LINE_26
//!        - \b LCD_C_SEGMENT_LINE_27
//!        - \b LCD_C_SEGMENT_LINE_28
//!        - \b LCD_C_SEGMENT_LINE_29
//!        - \b LCD_C_SEGMENT_LINE_30
//!        - \b LCD_C_SEGMENT_LINE_31
//!        - \b LCD_C_SEGMENT_LINE_32
//!        - \b LCD_C_SEGMENT_LINE_33
//!        - \b LCD_C_SEGMENT_LINE_34
//!        - \b LCD_C_SEGMENT_LINE_35
//!        - \b LCD_C_SEGMENT_LINE_36
//!        - \b LCD_C_SEGMENT_LINE_37
//!        - \b LCD_C_SEGMENT_LINE_38
//!        - \b LCD_C_SEGMENT_LINE_39
//!        - \b LCD_C_SEGMENT_LINE_40
//!        - \b LCD_C_SEGMENT_LINE_41
//!        - \b LCD_C_SEGMENT_LINE_42
//!        - \b LCD_C_SEGMENT_LINE_43
//!        - \b LCD_C_SEGMENT_LINE_44
//!        - \b LCD_C_SEGMENT_LINE_45
//!        - \b LCD_C_SEGMENT_LINE_46
//!        - \b LCD_C_SEGMENT_LINE_47
//!        - \b LCD_C_SEGMENT_LINE_48
//!        - \b LCD_C_SEGMENT_LINE_49
//!        - \b LCD_C_SEGMENT_LINE_50
//!        - \b LCD_C_SEGMENT_LINE_51
//!        - \b LCD_C_SEGMENT_LINE_52
//!        - \b LCD_C_SEGMENT_LINE_53
//!        - \b LCD_C_SEGMENT_LINE_54
//!        - \b LCD_C_SEGMENT_LINE_55
//!        - \b LCD_C_SEGMENT_LINE_56
//!        - \b LCD_C_SEGMENT_LINE_57
//!        - \b LCD_C_SEGMENT_LINE_58
//!        - \b LCD_C_SEGMENT_LINE_59
//!        - \b LCD_C_SEGMENT_LINE_60
//!        - \b LCD_C_SEGMENT_LINE_61
//!        - \b LCD_C_SEGMENT_LINE_62
//!        - \b LCD_C_SEGMENT_LINE_63
//! \param value is the designated value for corresponding blink pin.
//!
//! Modified bits are \b MBITx of \b LCDBMx register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setBlinkingMemory(uint16_t baseAddress, uint8_t pin, uint8_t value)
{
        uint8_t muxRate = HWREG16(baseAddress + OFS_LCDCCTL0)
                          & (LCDMX2 | LCDMX1 | LCDMX0);

        // static, 2-mux, 3-mux, 4-mux
        if (muxRate <= (LCDMX1 | LCDMX0)) {
                if (pin & 1) {
                        HWREG8(baseAddress + OFS_LCDBM1 + pin / 2) &= 0x0F;
                        HWREG8(baseAddress + OFS_LCDBM1 + pin / 2) |= (value & 0xF) << 4;
                }else  {
                        HWREG8(baseAddress + OFS_LCDBM1 + pin / 2) &= 0xF0;
                        HWREG8(baseAddress + OFS_LCDBM1 + pin / 2) |= (value & 0xF);
                }
        }else
                //5-mux, 6-mux, 7-mux, 8-mux
                HWREG8(baseAddress + OFS_LCDBM1 + pin) = value;

}

//*****************************************************************************
//
//! \brief Configs the charge pump for synchronization and disabled capability.
//!
//! This function is device-specific. The charge pump clock can be synchronized
//! to a device-specific clock, and also can be disabled by connected function.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param syncToClock is the synchronization select.
//!        Valid values are:
//!        - \b LCD_C_SYNCHRONIZATION_DISABLED [Default]
//!        - \b LCD_C_SYNCHRONIZATION_ENABLED
//! \param functionControl is the connected function control select. Setting 0
//!        to make connected function not disable charge pump.
//!
//! Modified bits are \b MBITx of \b LCDBMx register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_configChargePump(uint16_t baseAddress, uint16_t syncToClock,
                            uint16_t functionControl)
{
        HWREG16(baseAddress + OFS_LCDCCPCTL) &= ~(LCDCPCLKSYNC);
        HWREG16(baseAddress + OFS_LCDCCPCTL) &= ~(LCDCPDIS7 | LCDCPDIS6 | LCDCPDIS5
                                                  | LCDCPDIS4 | LCDCPDIS3 | LCDCPDIS2 | LCDCPDIS1 | LCDCPDIS0);

        HWREG16(baseAddress + OFS_LCDCCPCTL) |= syncToClock | functionControl;
}

//*****************************************************************************
//
//! \brief Sets the LCD memory with the specified character.
//!
//! This function sets the LCD_C memory with user specified character. It only
//! support 7-segment LCD layout and '0'~'9' characters.
//!
//! \param baseAddress is the base address of the LCD_C module.
//! \param startPin is the beginning index of LCD pin for character display.
//!        Valid values are:
//!        - \b LCD_C_SEGMENT_LINE_0
//!        - \b LCD_C_SEGMENT_LINE_1
//!        - \b LCD_C_SEGMENT_LINE_2
//!        - \b LCD_C_SEGMENT_LINE_3
//!        - \b LCD_C_SEGMENT_LINE_4
//!        - \b LCD_C_SEGMENT_LINE_5
//!        - \b LCD_C_SEGMENT_LINE_6
//!        - \b LCD_C_SEGMENT_LINE_7
//!        - \b LCD_C_SEGMENT_LINE_8
//!        - \b LCD_C_SEGMENT_LINE_9
//!        - \b LCD_C_SEGMENT_LINE_10
//!        - \b LCD_C_SEGMENT_LINE_11
//!        - \b LCD_C_SEGMENT_LINE_12
//!        - \b LCD_C_SEGMENT_LINE_13
//!        - \b LCD_C_SEGMENT_LINE_14
//!        - \b LCD_C_SEGMENT_LINE_15
//!        - \b LCD_C_SEGMENT_LINE_16
//!        - \b LCD_C_SEGMENT_LINE_17
//!        - \b LCD_C_SEGMENT_LINE_18
//!        - \b LCD_C_SEGMENT_LINE_19
//!        - \b LCD_C_SEGMENT_LINE_20
//!        - \b LCD_C_SEGMENT_LINE_21
//!        - \b LCD_C_SEGMENT_LINE_22
//!        - \b LCD_C_SEGMENT_LINE_23
//!        - \b LCD_C_SEGMENT_LINE_24
//!        - \b LCD_C_SEGMENT_LINE_25
//!        - \b LCD_C_SEGMENT_LINE_26
//!        - \b LCD_C_SEGMENT_LINE_27
//!        - \b LCD_C_SEGMENT_LINE_28
//!        - \b LCD_C_SEGMENT_LINE_29
//!        - \b LCD_C_SEGMENT_LINE_30
//!        - \b LCD_C_SEGMENT_LINE_31
//!        - \b LCD_C_SEGMENT_LINE_32
//!        - \b LCD_C_SEGMENT_LINE_33
//!        - \b LCD_C_SEGMENT_LINE_34
//!        - \b LCD_C_SEGMENT_LINE_35
//!        - \b LCD_C_SEGMENT_LINE_36
//!        - \b LCD_C_SEGMENT_LINE_37
//!        - \b LCD_C_SEGMENT_LINE_38
//!        - \b LCD_C_SEGMENT_LINE_39
//!        - \b LCD_C_SEGMENT_LINE_40
//!        - \b LCD_C_SEGMENT_LINE_41
//!        - \b LCD_C_SEGMENT_LINE_42
//!        - \b LCD_C_SEGMENT_LINE_43
//!        - \b LCD_C_SEGMENT_LINE_44
//!        - \b LCD_C_SEGMENT_LINE_45
//!        - \b LCD_C_SEGMENT_LINE_46
//!        - \b LCD_C_SEGMENT_LINE_47
//!        - \b LCD_C_SEGMENT_LINE_48
//!        - \b LCD_C_SEGMENT_LINE_49
//!        - \b LCD_C_SEGMENT_LINE_50
//!        - \b LCD_C_SEGMENT_LINE_51
//!        - \b LCD_C_SEGMENT_LINE_52
//!        - \b LCD_C_SEGMENT_LINE_53
//!        - \b LCD_C_SEGMENT_LINE_54
//!        - \b LCD_C_SEGMENT_LINE_55
//!        - \b LCD_C_SEGMENT_LINE_56
//!        - \b LCD_C_SEGMENT_LINE_57
//!        - \b LCD_C_SEGMENT_LINE_58
//!        - \b LCD_C_SEGMENT_LINE_59
//!        - \b LCD_C_SEGMENT_LINE_60
//!        - \b LCD_C_SEGMENT_LINE_61
//!        - \b LCD_C_SEGMENT_LINE_62
//!        - \b LCD_C_SEGMENT_LINE_63
//! \param character is the character to display.
//!
//! Modified bits are \b LCDCPCLKSYNC and \b LCDCPDISx of \b LCDCCPCTL register.
//!
//! \return None
//
//*****************************************************************************
void LCD_C_setMemoryCharacter(uint16_t baseAddress, uint8_t startPin,
                              uint8_t character)
{
        uint8_t muxRate = HWREG16(baseAddress + OFS_LCDCCTL0)
                          & (LCDMX2 | LCDMX1 | LCDMX0);

        uint8_t intlPin = startPin >> 1;
        uint32_t val = 0;

        if (muxRate == LCD_C_STATIC) {
                switch (character) {
                case 0x30:
                        val = 0x00111111;
                        break;
                case 0x31:
                        val = 0x00000110;
                        break;
                case 0x32:
                        val = 0x01011011;
                        break;
                case 0x33:
                        val = 0x01001111;
                        break;
                case 0x34:
                        val = 0x01100110;
                        break;
                case 0x35:
                        val = 0x01101101;
                        break;
                case 0x36:
                        val = 0x01111100;
                        break;
                case 0x37:
                        val = 0x00000111;
                        break;
                case 0x38:
                        val = 0x01111111;
                        break;
                case 0x39:
                        val = 0x01100111;
                        break;
                default:
                        break;
                }
                HWREG32(baseAddress + OFS_LCDM1 + intlPin) = val;
        }else if (muxRate == LCD_C_2_MUX) {
                switch (character) {
                case 0x30:
                        val = 0x1333;
                        break;
                case 0x31:
                        val = 0x0220;
                        break;
                case 0x32:
                        val = 0x3122;
                        break;
                case 0x33:
                        val = 0x2322;
                        break;
                case 0x34:
                        val = 0x2221;
                        break;
                case 0x35:
                        val = 0x1303;
                        break;
                case 0x36:
                        val = 0x3301;
                        break;
                case 0x37:
                        val = 0x0222;
                        break;
                case 0x38:
                        val = 0x3323;
                        break;
                case 0x39:
                        val = 0x2223;
                        break;
                default:
                        break;
                }
                HWREG16(baseAddress + OFS_LCDM1 + intlPin) = (uint16_t)val;
        }else if (muxRate == LCD_C_3_MUX) {
                switch (character) {
                case 0x30:
                        val = 0x0653;
                        break;
                case 0x31:
                        val = 0x0600;
                        break;
                case 0x32:
                        val = 0x0471;
                        break;
                case 0x33:
                        val = 0x0670;
                        break;
                case 0x34:
                        val = 0x0622;
                        break;
                case 0x35:
                        val = 0x0272;
                        break;
                case 0x36:
                        val = 0x0233;
                        break;
                case 0x37:
                        val = 0x0640;
                        break;
                case 0x38:
                        val = 0x0673;
                        break;
                case 0x39:
                        val = 0x0664;
                        break;
                default:
                        break;
                }
                if (intlPin % 3 == 0) {
                        HWREG8(baseAddress + OFS_LCDM1 + intlPin) = (uint8_t)val;
                        HWREG8(baseAddress + OFS_LCDM1 + intlPin + 1) &= 0xF0;
                        HWREG8(baseAddress + OFS_LCDM1 + intlPin + 1) |= (uint8_t)(val >> 8);
                }else  {
                        HWREG8(baseAddress + OFS_LCDM1 + intlPin) &= 0x0F;
                        HWREG8(baseAddress + OFS_LCDM1 + intlPin) |= (uint8_t)((val & 0x000F) << 4);
                        HWREG8(baseAddress + OFS_LCDM1 + intlPin + 1) = (uint8_t)((val & 0x0FF0) >> 4);
                }

        }else if (muxRate == LCD_C_4_MUX) {
                switch (character) {
                case 0x30:
                        val = 0xEB;
                        break;
                case 0x31:
                        val = 0x60;
                        break;
                case 0x32:
                        val = 0xC7;
                        break;
                case 0x33:
                        val = 0xE5;
                        break;
                case 0x34:
                        val = 0x6C;
                        break;
                case 0x35:
                        val = 0xAD;
                        break;
                case 0x36:
                        val = 0x2F;
                        break;
                case 0x37:
                        val = 0xE0;
                        break;
                case 0x38:
                        val = 0xEF;
                        break;
                case 0x39:
                        val = 0xEC;
                        break;
                default:
                        break;
                }
                HWREG8(baseAddress + OFS_LCDM1 + intlPin) = (uint8_t)val;
        }
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for lcd_c_api
//! @}
//
//*****************************************************************************
