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
// sd24.c - Driver for the sd24 Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup sd24_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_SD24__
#include "sd24.h"

#include <assert.h>

//*****************************************************************************
//
//! \brief Initializes the SD24 Module
//!
//! This function initializes the SD24 module sigma-delta analog-to-digital
//! conversions. Specifically the function sets up the clock source for the
//! SD24 core to use for conversions. Upon completion of the initialization the
//! SD24 interrupt registers will be reset and the given parameters will be
//! set. The converter configuration settings are independent of this function.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param referenceSelect selects the reference source for the SD24 core
//!        Valid values are:
//!        - \b SD24_REF_EXTERNAL [Default]
//!        - \b SD24_REF_INTERNAL
//!        \n Modified bits are \b SD24REFS of \b SD24BCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void SD24_init(uint16_t baseAddress, uint8_t referenceSelect)
{
        uint16_t address;
        uint8_t converter;

        assert(
                (SD24_REF_EXTERNAL == referenceSelect) ||
                (SD24_REF_INTERNAL == referenceSelect)
                );

        // Reset all interrupts and flags, and turn off all groups
        for ( converter = 0; converter < 4; converter++ ) {
                address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));
                HWREG16(address) &= ~(SD24SC + SD24IFG + SD24GRP);
        }

        // Turn off overflow interrupts, and configure SD24 reference
        HWREG16(baseAddress + OFS_SD24CTL) = referenceSelect;
        return;
}

//*****************************************************************************
//
//! \brief Configure SD24 converter
//!
//! This function initializes a converter of the SD24 module. Upon completion
//! the converter will be ready for a conversion and can be started with the
//! SD24_startConverterConversion(). Additional configuration such as data
//! format can be configured in SD24_setConverterDataFormat().
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter that will be configured. Check check
//!        datasheet for available converters on device.
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param conversionMode determines whether the converter will do continuous
//!        samples or a single sample
//!        Valid values are:
//!        - \b SD24_CONTINUOUS_MODE [Default]
//!        - \b SD24_SINGLE_MODE
//!        \n Modified bits are \b SD24SNGL of \b SD24CCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_initConverter(uint16_t baseAddress,
                        uint16_t converter,
                        uint16_t conversionMode
                        )
{

        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        // Clearing previous settings for configuration
        HWREG16(address) &= ~(SD24SNGL);

        HWREG16(address) |= conversionMode;
}

//*****************************************************************************
//
//! \brief Configure SD24 converter - Advanced Configure
//!
//! This function initializes a converter of the SD24 module. Upon completion
//! the converter will be ready for a conversion and can be started with the
//! SD24_startConverterConversion().
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param param is the pointer to struct for converter advanced configuration.
//!
//! \return None
//
//*****************************************************************************
void SD24_initConverterAdvanced(uint16_t baseAddress,
                                SD24_initConverterAdvancedParam *param)
{
        assert(param != 0);
        assert(
                (SD24_CONVERTER_0 == param->converter) ||
                (SD24_CONVERTER_1 == param->converter) ||
                (SD24_CONVERTER_2 == param->converter) ||
                (SD24_CONVERTER_3 == param->converter)
                );

        // Getting correct SD24CCTLx register
        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (param->converter * 0x02));

        // Clearing previous settings for configuration
        HWREG16(address) &= ~(SD24SNGL | SD24DF | SD24GRP | SD24OSR_32);

        HWREG16(address) |= (param->groupEnable | param->conversionMode | param->dataFormat
                             | param->oversampleRatio);

        // Getting correct SD24INTCTLx register
        address = baseAddress + (OFS_SD24INCTL0 + (param->converter));

        // Clearing previous settings for configuration
        HWREG8(address) &= ~(SD24_GAIN_8 | SD24_GAIN_16 | SD24INTDLY | SD24INCH_7);

        HWREG8(address) |= (param->gain | param->interruptDelay | param->inputChannel);
}

//*****************************************************************************
//
//! \brief Set SD24 converter data format
//!
//! This function sets the converter format so that the resulting data can be
//! viewed in either binary or 2's complement.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter that will be configured. Check check
//!        datasheet for available converters on device.
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param dataFormat selects how the data format of the results
//!        Valid values are:
//!        - \b SD24_DATA_FORMAT_BINARY [Default]
//!        - \b SD24_DATA_FORMAT_2COMPLEMENT
//!        \n Modified bits are \b SD24DFx of \b SD24CCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_setConverterDataFormat(uint16_t baseAddress,
                                 uint16_t converter,
                                 uint16_t dataFormat
                                 )
{
        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Clearing previous settings for configuration
        HWREG16(address) &= ~(SD24DF);

        HWREG16(address) |= dataFormat;
}

//*****************************************************************************
//
//! \brief Start Conversion for Converter
//!
//! This function starts a single converter.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter that will be started
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//!        \n Modified bits are \b SD24SC of \b SD24CCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_startConverterConversion(uint16_t baseAddress, uint8_t converter)
{
        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Setting SD24SC bit to start conversion
        HWREG16(address) |= SD24SC;
}

//*****************************************************************************
//
//! \brief Stop Conversion for Converter
//!
//! This function stops a single converter.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter that will be stopped
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//!        \n Modified bits are \b SD24SC of \b SD24CCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_stopConverterConversion(uint16_t baseAddress, uint8_t converter)
{
        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        // Setting SD24SC bit to start conversion
        HWREG16(address) &= ~(SD24SC);
}
//*****************************************************************************
//
//! \brief Configures the input channel
//!
//! This function configures the input channel. For MSP430i2xx devices, users
//! can choose either analog input or internal temperature input.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter that will be configured
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param inputChannel selects oversampling ratio for the converter
//!        Valid values are:
//!        - \b SD24_INPUT_CH_ANALOG
//!        - \b SD24_INPUT_CH_TEMPERATURE
//!        \n Modified bits are \b SD24INCHx of \b SD24INCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_setInputChannel(uint16_t baseAddress, uint8_t converter,
                          uint8_t inputChannel)
{
        uint16_t address = baseAddress + (OFS_SD24INCTL0 + (converter));

        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Clear previous settings
        HWREG8(address) &= ~(SD24INCH_7);

        HWREG8(address) |= inputChannel;
}

//*****************************************************************************
//
//! \brief Configures the delay for an interrupt to trigger
//!
//! This function configures the delay for the first interrupt service request
//! for the corresponding converter. This feature delays the interrupt request
//! for a completed conversion by up to four conversion cycles allowing the
//! digital filter to settle prior to generating an interrupt request.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter that will be stopped
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param interruptDelay selects the delay for the interrupt
//!        Valid values are:
//!        - \b SD24_FIRST_SAMPLE_INTERRUPT
//!        - \b SD24_FOURTH_SAMPLE_INTERRUPT [Default]
//!        \n Modified bits are \b SD24INTDLYx of \b SD24INCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_setInterruptDelay(uint16_t baseAddress,
                            uint8_t converter,
                            uint8_t interruptDelay
                            )
{
        uint16_t address = baseAddress + (OFS_SD24INCTL0 + (converter));

        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Clear previous settings
        HWREG16(address) &= ~(SD24INTDLY);

        HWREG16(address) |= interruptDelay;
}

//*****************************************************************************
//
//! \brief Configures the oversampling ratio for a converter
//!
//! This function configures the oversampling ratio for a given converter.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter that will be configured
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param oversampleRatio selects oversampling ratio for the converter
//!        Valid values are:
//!        - \b SD24_OVERSAMPLE_32
//!        - \b SD24_OVERSAMPLE_64
//!        - \b SD24_OVERSAMPLE_128
//!        - \b SD24_OVERSAMPLE_256
//!        \n Modified bits are \b SD24OSRx of \b SD24OSRx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_setOversampling(uint16_t baseAddress,
                          uint8_t converter,
                          uint16_t oversampleRatio
                          )
{
        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Clear previous settings
        HWREG16(address) &= ~(SD24OSR0 | SD24OSR1);

        HWREG16(address) |= oversampleRatio;
}

//*****************************************************************************
//
//! \brief Configures the gain for the converter
//!
//! This function configures the gain for a single converter.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter that will be configured
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param gain selects the gain for the converter
//!        Valid values are:
//!        - \b SD24_GAIN_1 [Default]
//!        - \b SD24_GAIN_2
//!        - \b SD24_GAIN_4
//!        - \b SD24_GAIN_8
//!        - \b SD24_GAIN_16
//!        \n Modified bits are \b SD24GAINx of \b SD24INCTLx register.
//!
//! \return None
//
//*****************************************************************************
void SD24_setGain(uint16_t baseAddress, uint8_t converter, uint8_t gain)
{
        uint16_t address = baseAddress + (OFS_SD24INCTL0 + (converter));

        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Clear previous settings
        HWREG8(address) &= ~(SD24GAIN0 | SD24GAIN1 | SD24GAIN2);

        HWREG8(address) |= gain;
}

//*****************************************************************************
//
//! \brief Returns the results for a converter
//!
//! This function gets the results from the SD24MEMx register for upper 16-bit
//! and lower 16-bit results, and concatenates them to form a long. The actual
//! result is a maximum 24 bits.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter who's results will be returned
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//!
//! \return Result of conversion
//
//*****************************************************************************
uint32_t SD24_getResults(uint16_t baseAddress, uint8_t converter)
{
        volatile uint16_t OSR;

        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Calculate address of MEM results
        uint16_t address = baseAddress + (OFS_SD24MEM0 + (converter * 0x02));

        // Get high word result
        HWREG16(baseAddress + (OFS_SD24CCTL0 + (converter * 0x02))) &= ~(SD24LSBACC);
        uint32_t highResult = (uint32_t)HWREG16(address);

        // Get low word result
        HWREG16(baseAddress + (OFS_SD24CCTL0 + (converter * 0x02))) |= SD24LSBACC;
        uint16_t lowResult = HWREG16(address);
        HWREG16(baseAddress + (OFS_SD24CCTL0 + (converter * 0x02))) &= ~(SD24LSBACC);

        // Determine the OSR and combine the high and low result words as appropriate
        OSR = HWREG16(baseAddress + (OFS_SD24CCTL0 + (converter * 0x02)))  & (SD24OSR0 | SD24OSR1);

        if (OSR == SD24_OVERSAMPLE_256)
                return (highResult << 8) | lowResult;
        else if (OSR == SD24_OVERSAMPLE_128)
                return (highResult << 5) | lowResult;
        else if (OSR == SD24_OVERSAMPLE_64)
                return (highResult << 2) | lowResult;
        else            // OSR = SD24_OVERSAMPLE_32

                return highResult;
}

//*****************************************************************************
//
//! \brief Returns the high word results for a converter
//!
//! This function gets the upper 16-bit result from the SD24MEMx register and
//! returns it.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter selects the converter who's results will be returned
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//!
//! \return Result of conversion
//
//*****************************************************************************
uint16_t SD24_getHighWordResults(uint16_t baseAddress, uint8_t converter)
{
        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Calculate address of MEM results
        uint16_t address = baseAddress + (OFS_SD24MEM0 + (converter * 0x02));

        // Get high word result
        HWREG16(baseAddress + (OFS_SD24CCTL0 + (converter * 0x02))) &= ~(SD24LSBACC);
        uint16_t highResult = HWREG16(address);

        return highResult;
}

//*****************************************************************************
//
//! \brief Enables interrupts for the SD24 Module
//!
//! This function enables interrupts for the SD24 module. Does not clear
//! interrupt flags.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter is the selected converter.
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param mask is the bit mask of the converter interrupt sources to be
//!        enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b SD24_CONVERTER_INTERRUPT
//!        - \b SD24_CONVERTER_OVERFLOW_INTERRUPT
//!        \n Modified bits are \b SD24OVIEx of \b SD24BIE register.
//!
//! \return None
//
//*****************************************************************************
void SD24_enableInterrupt(uint16_t baseAddress,
                          uint8_t converter,
                          uint16_t mask
                          )
{
        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Calculating address
        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        //Clear Interrupt Flags
        HWREG16(address) &= ~(SD24IFG | SD24OVIFG);

        //Enable Interrupt
        if (mask & SD24_CONVERTER_INTERRUPT)
                HWREG16(address) |= (SD24IE);
        if (mask & SD24_CONVERTER_OVERFLOW_INTERRUPT)
                HWREG16(baseAddress + OFS_SD24CTL) |= (SD24OVIE);
}

//*****************************************************************************
//
//! \brief Disables interrupts for the SD24 Module
//!
//! This function disables interrupts for the SD24 module.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter is the selected converter.
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param mask is the bit mask of the converter interrupt sources to be
//!        disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b SD24_CONVERTER_INTERRUPT
//!        - \b SD24_CONVERTER_OVERFLOW_INTERRUPT
//!        \n Modified bits are \b SD24OVIEx of \b SD24BIE register.
//!
//! Modified bits of \b SD24BIE register.
//!
//! \return None
//
//*****************************************************************************
void SD24_disableInterrupt(uint16_t baseAddress,
                           uint8_t converter,
                           uint16_t mask
                           )
{
        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Calculating address
        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        //Enable Interrupt
        if (mask & SD24_CONVERTER_INTERRUPT)
                HWREG16(address) &= ~(SD24IE);
        if (mask & SD24_CONVERTER_OVERFLOW_INTERRUPT)
                HWREG16(baseAddress + OFS_SD24CTL) &= ~(SD24OVIE);
}

//*****************************************************************************
//
//! \brief Clears interrupts for the SD24 Module
//!
//! This function clears interrupt flags for the SD24 module.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter is the selected converter.
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param mask is the bit mask of the converter interrupt sources to clear.
//!        Mask value is the logical OR of any of the following:
//!        - \b SD24_CONVERTER_INTERRUPT
//!        - \b SD24_CONVERTER_OVERFLOW_INTERRUPT
//!        \n Modified bits are \b SD24OVIFGx of \b SD24BIFG register.
//!
//! \return None
//
//*****************************************************************************
void SD24_clearInterrupt(uint16_t baseAddress,
                         uint8_t converter,
                         uint16_t mask
                         )
{
        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Calculating address
        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        //Clear Interrupt Flags
        HWREG16(address) &= ~(mask);
}

//*****************************************************************************
//
//! \brief Returns the interrupt status for the SD24 Module
//!
//! This function returns interrupt flag statuses for the SD24 module.
//!
//! \param baseAddress is the base address of the SD24 module.
//! \param converter is the selected converter.
//!        Valid values are:
//!        - \b SD24_CONVERTER_0
//!        - \b SD24_CONVERTER_1
//!        - \b SD24_CONVERTER_2
//!        - \b SD24_CONVERTER_3
//! \param mask is the bit mask of the converter interrupt sources to return.
//!        Mask value is the logical OR of any of the following:
//!        - \b SD24_CONVERTER_INTERRUPT
//!        - \b SD24_CONVERTER_OVERFLOW_INTERRUPT
//!
//! \return Logical OR of any of the following:
//!         - \b SD24_CONVERTER_INTERRUPT
//!         - \b SD24_CONVERTER_OVERFLOW_INTERRUPT
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint16_t SD24_getInterruptStatus(uint16_t baseAddress, uint8_t converter,
                                 uint16_t mask)
{
        assert(
                (SD24_CONVERTER_0 == converter) ||
                (SD24_CONVERTER_1 == converter) ||
                (SD24_CONVERTER_2 == converter) ||
                (SD24_CONVERTER_3 == converter)
                );

        // Calculate address
        uint16_t address = baseAddress + (OFS_SD24CCTL0 + (converter * 0x02));

        // Read and return interrupt statuses
        return HWREG16(address) & (mask) & (SD24IFG | SD24OVIFG);
}


#endif
//*****************************************************************************
//
//! Close the doxygen group for sd24_api
//! @}
//
//*****************************************************************************
