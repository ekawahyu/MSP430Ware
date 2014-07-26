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
// lcd_c.h - Driver for the LCD_C Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_LCD_C_H__
#define __MSP430WARE_LCD_C_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_LCD_C__

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

//******************************************************************************
//
// The following is a struct that is passed to LCD_C_initParam()
//
//******************************************************************************
typedef struct LCD_C_initParam {
        uint16_t clockSource;
        uint16_t clockDivider;
        uint16_t clockPrescalar;
        uint16_t muxRate;
        uint16_t waveforms;
        uint16_t segments;
} LCD_C_initParam;

extern const LCD_C_initParam LCD_C_INIT_PARAM;

//*****************************************************************************
//
// The following are values that can be passed to the clockSource parameter for
// functions: LCD_C_init().
//
//*****************************************************************************
#define LCD_C_CLOCKSOURCE_ACLK                                            (0x0)
#define LCD_C_CLOCKSOURCE_VLOCLK                                      (LCDSSEL)

//*****************************************************************************
//
// The following are values that can be passed to the clockDivider parameter
// for functions: LCD_C_init().
//
//*****************************************************************************
#define LCD_C_CLOLKDIVIDER_1                                         (LCDDIV_0)
#define LCD_C_CLOLKDIVIDER_2                                         (LCDDIV_1)
#define LCD_C_CLOLKDIVIDER_3                                         (LCDDIV_2)
#define LCD_C_CLOLKDIVIDER_4                                         (LCDDIV_3)
#define LCD_C_CLOLKDIVIDER_5                                         (LCDDIV_4)
#define LCD_C_CLOLKDIVIDER_6                                         (LCDDIV_5)
#define LCD_C_CLOLKDIVIDER_7                                         (LCDDIV_6)
#define LCD_C_CLOLKDIVIDER_8                                         (LCDDIV_7)
#define LCD_C_CLOLKDIVIDER_9                                         (LCDDIV_8)
#define LCD_C_CLOLKDIVIDER_10                                        (LCDDIV_9)
#define LCD_C_CLOLKDIVIDER_11                                       (LCDDIV_10)
#define LCD_C_CLOLKDIVIDER_12                                       (LCDDIV_11)
#define LCD_C_CLOLKDIVIDER_13                                       (LCDDIV_12)
#define LCD_C_CLOLKDIVIDER_14                                       (LCDDIV_13)
#define LCD_C_CLOLKDIVIDER_15                                       (LCDDIV_14)
#define LCD_C_CLOLKDIVIDER_16                                       (LCDDIV_15)
#define LCD_C_CLOLKDIVIDER_17                                       (LCDDIV_16)
#define LCD_C_CLOLKDIVIDER_18                                       (LCDDIV_17)
#define LCD_C_CLOLKDIVIDER_19                                       (LCDDIV_18)
#define LCD_C_CLOLKDIVIDER_20                                       (LCDDIV_19)
#define LCD_C_CLOLKDIVIDER_21                                       (LCDDIV_20)
#define LCD_C_CLOLKDIVIDER_22                                       (LCDDIV_21)
#define LCD_C_CLOLKDIVIDER_23                                       (LCDDIV_22)
#define LCD_C_CLOLKDIVIDER_24                                       (LCDDIV_23)
#define LCD_C_CLOLKDIVIDER_25                                       (LCDDIV_24)
#define LCD_C_CLOLKDIVIDER_26                                       (LCDDIV_25)
#define LCD_C_CLOLKDIVIDER_27                                       (LCDDIV_26)
#define LCD_C_CLOLKDIVIDER_28                                       (LCDDIV_27)
#define LCD_C_CLOLKDIVIDER_29                                       (LCDDIV_28)
#define LCD_C_CLOLKDIVIDER_30                                       (LCDDIV_29)
#define LCD_C_CLOLKDIVIDER_31                                       (LCDDIV_30)
#define LCD_C_CLOLKDIVIDER_32                                       (LCDDIV_31)

//*****************************************************************************
//
// The following are values that can be passed to the clockPrescalar parameter
// for functions: LCD_C_init().
//
//*****************************************************************************
#define LCD_C_CLOCKPRESCALAR_1                                       (LCDPRE_0)
#define LCD_C_CLOCKPRESCALAR_2                                       (LCDPRE_1)
#define LCD_C_CLOCKPRESCALAR_4                                       (LCDPRE_2)
#define LCD_C_CLOCKPRESCALAR_8                                       (LCDPRE_3)
#define LCD_C_CLOCKPRESCALAR_16                                      (LCDPRE_4)
#define LCD_C_CLOCKPRESCALAR_32                                      (LCDPRE_5)

//*****************************************************************************
//
// The following are values that can be passed to the muxRate parameter for
// functions: LCD_C_init().
//
//*****************************************************************************
#define LCD_C_STATIC                                                      (0x0)
#define LCD_C_2_MUX                                                    (LCDMX0)
#define LCD_C_3_MUX                                                    (LCDMX1)
#define LCD_C_4_MUX                                           (LCDMX1 | LCDMX0)
#define LCD_C_5_MUX                                                    (LCDMX2)
#define LCD_C_6_MUX                                           (LCDMX2 | LCDMX0)
#define LCD_C_7_MUX                                           (LCDMX2 | LCDMX1)
#define LCD_C_8_MUX                                  (LCDMX2 | LCDMX1 | LCDMX0)

//*****************************************************************************
//
// The following are values that can be passed to the waveforms parameter for
// functions: LCD_C_init().
//
//*****************************************************************************
#define LCD_C_STANDARD_WAVEFORMS                                          (0x0)
#define LCD_C_LOW_POWER_WAVEFORMS                                       (LCDLP)

//*****************************************************************************
//
// The following are values that can be passed to the segments parameter for
// functions: LCD_C_init().
//
//*****************************************************************************
#define LCD_C_SEGMENTS_DISABLED                                           (0x0)
#define LCD_C_SEGMENTS_ENABLED                                         (LCDSON)

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: LCD_C_clearInterrupt(), LCD_C_getInterruptStatus(),
// LCD_C_enableInterrupt(), and LCD_C_disableInterrupt() as well as returned by
// the LCD_C_getInterruptStatus() function.
//
//*****************************************************************************
#define LCD_C_NO_CAPACITANCE_CONNECTED_INTERRUPT                   (LCDNOCAPIE)
#define LCD_C_BLINKING_SEGMENTS_ON_INTERRUPT                       (LCDBLKONIE)
#define LCD_C_BLINKING_SEGMENTS_OFF_INTERRUPT                     (LCDBLKOFFIE)
#define LCD_C_FRAME_INTERRUPT                                        (LCDFRMIE)

//*****************************************************************************
//
// The following are values that can be passed to the displayMemory parameter
// for functions: LCD_C_selectDisplayMemory().
//
//*****************************************************************************
#define LCD_C_DISPLAYSOURCE_MEMORY                                        (0x0)
#define LCD_C_DISPLAYSOURCE_BLINKINGMEMORY                            (LCDDISP)

//*****************************************************************************
//
// The following are values that can be passed to the clockDivider parameter
// for functions: LCD_C_setBlinkingControl().
//
//*****************************************************************************
#define LCD_C_BLINK_FREQ_CLOCK_DIVIDER_1                                  (0x0)
#define LCD_C_BLINK_FREQ_CLOCK_DIVIDER_2                           (LCDBLKDIV0)
#define LCD_C_BLINK_FREQ_CLOCK_DIVIDER_3                           (LCDBLKDIV1)
#define LCD_C_BLINK_FREQ_CLOCK_DIVIDER_4              (LCDBLKDIV0 | LCDBLKDIV1)
#define LCD_C_BLINK_FREQ_CLOCK_DIVIDER_5                           (LCDBLKDIV2)
#define LCD_C_BLINK_FREQ_CLOCK_DIVIDER_6              (LCDBLKDIV2 | LCDBLKDIV0)
#define LCD_C_BLINK_FREQ_CLOCK_DIVIDER_7              (LCDBLKDIV2 | LCDBLKDIV1)
#define LCD_C_BLINK_FREQ_CLOCK_DIVIDER_8 (LCDBLKDIV2 | LCDBLKDIV1 | LCDBLKDIV0)

//*****************************************************************************
//
// The following are values that can be passed to the clockPrescalar parameter
// for functions: LCD_C_setBlinkingControl().
//
//*****************************************************************************
#define LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_512                              (0x0)
#define LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_1024                      (LCDBLKPRE0)
#define LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_2048                      (LCDBLKPRE1)
#define LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_4096         (LCDBLKPRE1 | LCDBLKPRE0)
#define LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_8162                      (LCDBLKPRE2)
#define LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_16384         LCDBLKPRE2 | LCDBLKPRE0)
#define LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_32768        (LCDBLKPRE2 | LCDBLKPRE1)
#define LCD_C_BLINK_FREQ_CLOCK_PRESCALAR_65536                                \
        (LCDBLKPRE2 | LCDBLKPRE1 | LCDBLKPRE0)

//*****************************************************************************
//
// The following are values that can be passed to the blinkingMode parameter
// for functions: LCD_C_setBlinkingControl().
//
//*****************************************************************************
#define LCD_C_BLINK_MODE_DISABLED                                 (LCDBLKMOD_0)
#define LCD_C_BLINK_MODE_INDIVIDUAL_SEGMENTS                      (LCDBLKMOD_1)
#define LCD_C_BLINK_MODE_ALL_SEGMENTS                             (LCDBLKMOD_2)
#define LCD_C_BLINK_MODE_SWITCHING_BETWEEN_DISPLAY_CONTENTS       (LCDBLKMOD_3)

//*****************************************************************************
//
// The following are values that can be passed to the bias parameter for
// functions: LCD_C_selectBias().
//
//*****************************************************************************
#define LCD_C_BIAS_1_3                                                    (0x0)
#define LCD_C_BIAS_1_2                                                  (LCD2B)

//*****************************************************************************
//
// The following are values that can be passed to the reference parameter for
// functions: LCD_C_selectChargePumpReference().
//
//*****************************************************************************
#define LCD_C_INTERNAL_REFERNCE_VOLTAGE                             (VLCDREF_0)
#define LCD_C_EXTERNAL_REFERENCE_VOLTAGE                            (VLCDREF_1)
#define LCD_C_INTERNAL_REFERNCE_VOLTAGE_SWITCHED_TO_EXTERNAL_PIN    (VLCDREF_2)

//*****************************************************************************
//
// The following are values that can be passed to the vlcdSource parameter for
// functions: LCD_C_setVLCDSource().
//
//*****************************************************************************
#define LCD_C_VLCD_GENERATED_INTERNALLY                                   (0x0)
#define LCD_C_VLCD_SOURCED_EXTERNALLY                                 (VLCDEXT)

//*****************************************************************************
//
// The following are values that can be passed to the v2v3v4Source parameter
// for functions: LCD_C_setVLCDSource().
//
//*****************************************************************************
#define LCD_C_V2V3V4_GENERATED_INTERNALLY_NOT_SWITCHED_TO_PINS            (0x0)
#define LCD_C_V2V3V4_GENERATED_INTERNALLY_SWITCHED_TO_PINS            (LCDREXT)
#define LCD_C_V2V3V4_SOURCED_EXTERNALLY                            (LCDEXTBIAS)

//*****************************************************************************
//
// The following are values that can be passed to the v5Source parameter for
// functions: LCD_C_setVLCDSource().
//
//*****************************************************************************
#define LCD_C_V5_VSS                                                      (0x0)
#define LCD_C_V5_SOURCED_FROM_R03                                      (R03EXT)

//*****************************************************************************
//
// The following are values that can be passed to the voltage parameter for
// functions: LCD_C_setVLCDVoltage().
//
//*****************************************************************************
#define LCD_C_CHARGEPUMP_DISABLED                                         (0x0)
#define LCD_C_CHARGEPUMP_VOLTAGE_2_60V_OR_2_17VREF                      (VLCD0)
#define LCD_C_CHARGEPUMP_VOLTAGE_2_66V_OR_2_22VREF                      (VLCD1)
#define LCD_C_CHARGEPUMP_VOLTAGE_2_72V_OR_2_27VREF              (VLCD1 | VLCD0)
#define LCD_C_CHARGEPUMP_VOLTAGE_2_78V_OR_2_32VREF                      (VLCD2)
#define LCD_C_CHARGEPUMP_VOLTAGE_2_84V_OR_2_37VREF              (VLCD2 | VLCD0)
#define LCD_C_CHARGEPUMP_VOLTAGE_2_90V_OR_2_42VREF              (VLCD2 | VLCD1)
#define LCD_C_CHARGEPUMP_VOLTAGE_2_96V_OR_2_47VREF      (VLCD2 | VLCD1 | VLCD0)
#define LCD_C_CHARGEPUMP_VOLTAGE_3_02V_OR_2_52VREF                      (VLCD3)
#define LCD_C_CHARGEPUMP_VOLTAGE_3_08V_OR_2_57VREF              (VLCD3 | VLCD0)
#define LCD_C_CHARGEPUMP_VOLTAGE_3_14V_OR_2_62VREF              (VLCD3 | VLCD1)
#define LCD_C_CHARGEPUMP_VOLTAGE_3_20V_OR_2_67VREF      (VLCD3 | VLCD1 | VLCD0)
#define LCD_C_CHARGEPUMP_VOLTAGE_3_26V_OR_2_72VREF              (VLCD3 | VLCD2)
#define LCD_C_CHARGEPUMP_VOLTAGE_3_32V_OR_2_77VREF      (VLCD3 | VLCD2 | VLCD0)
#define LCD_C_CHARGEPUMP_VOLTAGE_3_38V_OR_2_82VREF      (VLCD3 | VLCD2 | VLCD1)
#define LCD_C_CHARGEPUMP_VOLTAGE_3_44V_OR_2_87VREF                            \
        (VLCD3 | VLCD2 | VLCD1 | VLCD0)

//*****************************************************************************
//
// The following are values that can be passed to the startPin parameter for
// functions: LCD_C_setPinAsLCDFunctionEx(), and LCD_C_setMemoryCharacter();
// the endPin parameter for functions: LCD_C_setPinAsLCDFunctionEx(); the pin
// parameter for functions: LCD_C_setPinAsLCDFunction(),
// LCD_C_setPinAsPortFunction(), LCD_C_setMemory(), and
// LCD_C_setBlinkingMemory().
//
//*****************************************************************************
#define LCD_C_SEGMENT_LINE_0                                                (0)
#define LCD_C_SEGMENT_LINE_1                                                (1)
#define LCD_C_SEGMENT_LINE_2                                                (2)
#define LCD_C_SEGMENT_LINE_3                                                (3)
#define LCD_C_SEGMENT_LINE_4                                                (4)
#define LCD_C_SEGMENT_LINE_5                                                (5)
#define LCD_C_SEGMENT_LINE_6                                                (6)
#define LCD_C_SEGMENT_LINE_7                                                (7)
#define LCD_C_SEGMENT_LINE_8                                                (8)
#define LCD_C_SEGMENT_LINE_9                                                (9)
#define LCD_C_SEGMENT_LINE_10                                              (10)
#define LCD_C_SEGMENT_LINE_11                                              (11)
#define LCD_C_SEGMENT_LINE_12                                              (12)
#define LCD_C_SEGMENT_LINE_13                                              (13)
#define LCD_C_SEGMENT_LINE_14                                              (14)
#define LCD_C_SEGMENT_LINE_15                                              (15)
#define LCD_C_SEGMENT_LINE_16                                              (16)
#define LCD_C_SEGMENT_LINE_17                                              (17)
#define LCD_C_SEGMENT_LINE_18                                              (18)
#define LCD_C_SEGMENT_LINE_19                                              (19)
#define LCD_C_SEGMENT_LINE_20                                              (20)
#define LCD_C_SEGMENT_LINE_21                                              (21)
#define LCD_C_SEGMENT_LINE_22                                              (22)
#define LCD_C_SEGMENT_LINE_23                                              (23)
#define LCD_C_SEGMENT_LINE_24                                              (24)
#define LCD_C_SEGMENT_LINE_25                                              (25)
#define LCD_C_SEGMENT_LINE_26                                              (26)
#define LCD_C_SEGMENT_LINE_27                                              (27)
#define LCD_C_SEGMENT_LINE_28                                              (28)
#define LCD_C_SEGMENT_LINE_29                                              (29)
#define LCD_C_SEGMENT_LINE_30                                              (30)
#define LCD_C_SEGMENT_LINE_31                                              (31)
#define LCD_C_SEGMENT_LINE_32                                              (32)
#define LCD_C_SEGMENT_LINE_33                                              (33)
#define LCD_C_SEGMENT_LINE_34                                              (34)
#define LCD_C_SEGMENT_LINE_35                                              (35)
#define LCD_C_SEGMENT_LINE_36                                              (36)
#define LCD_C_SEGMENT_LINE_37                                              (37)
#define LCD_C_SEGMENT_LINE_38                                              (38)
#define LCD_C_SEGMENT_LINE_39                                              (39)
#define LCD_C_SEGMENT_LINE_40                                              (40)
#define LCD_C_SEGMENT_LINE_41                                              (41)
#define LCD_C_SEGMENT_LINE_42                                              (42)
#define LCD_C_SEGMENT_LINE_43                                              (43)
#define LCD_C_SEGMENT_LINE_44                                              (44)
#define LCD_C_SEGMENT_LINE_45                                              (45)
#define LCD_C_SEGMENT_LINE_46                                              (46)
#define LCD_C_SEGMENT_LINE_47                                              (47)
#define LCD_C_SEGMENT_LINE_48                                              (48)
#define LCD_C_SEGMENT_LINE_49                                              (49)
#define LCD_C_SEGMENT_LINE_50                                              (50)
#define LCD_C_SEGMENT_LINE_51                                              (51)
#define LCD_C_SEGMENT_LINE_52                                              (52)
#define LCD_C_SEGMENT_LINE_53                                              (53)
#define LCD_C_SEGMENT_LINE_54                                              (54)
#define LCD_C_SEGMENT_LINE_55                                              (55)
#define LCD_C_SEGMENT_LINE_56                                              (56)
#define LCD_C_SEGMENT_LINE_57                                              (57)
#define LCD_C_SEGMENT_LINE_58                                              (58)
#define LCD_C_SEGMENT_LINE_59                                              (59)
#define LCD_C_SEGMENT_LINE_60                                              (60)
#define LCD_C_SEGMENT_LINE_61                                              (61)
#define LCD_C_SEGMENT_LINE_62                                              (62)
#define LCD_C_SEGMENT_LINE_63                                              (63)

//*****************************************************************************
//
// The following are values that can be passed to the syncToClock parameter for
// functions: LCD_C_configChargePump().
//
//*****************************************************************************
#define LCD_C_SYNCHRONIZATION_DISABLED                                    (0x0)
#define LCD_C_SYNCHRONIZATION_ENABLED                            (LCDCPCLKSYNC)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void LCD_C_init(uint16_t baseAddress,
                       LCD_C_initParam *initParams);

extern void LCD_C_on(uint16_t baseAddress);

extern void LCD_C_off(uint16_t baseAddress);

extern void LCD_C_clearInterrupt(uint16_t baseAddress,
                                 uint16_t mask);

extern uint16_t LCD_C_getInterruptStatus(uint16_t baseAddress,
                                         uint16_t mask);

extern void LCD_C_enableInterrupt(uint16_t baseAddress,
                                  uint16_t mask);

extern void LCD_C_disableInterrupt(uint16_t baseAddress,
                                   uint16_t mask);

extern void LCD_C_clearMemory(uint16_t baseAddress);

extern void LCD_C_clearBlinkingMemory(uint16_t baseAddress);

extern void LCD_C_selectDisplayMemory(uint16_t baseAddress,
                                      uint16_t displayMemory);

extern void LCD_C_setBlinkingControl(uint16_t baseAddress,
                                     uint8_t clockDivider,
                                     uint8_t clockPrescalar,
                                     uint8_t mode);

extern void LCD_C_enableChargePump(uint16_t baseAddress);

extern void LCD_C_disableChargePump(uint16_t baseAddress);

extern void LCD_C_selectBias(uint16_t baseAddress,
                             uint16_t bias);

extern void LCD_C_selectChargePumpReference(uint16_t baseAddress,
                                            uint16_t reference);

extern void LCD_C_setVLCDSource(uint16_t baseAddress,
                                uint16_t vlcdSource,
                                uint16_t v2v3v4Source,
                                uint16_t v5Source);

extern void LCD_C_setVLCDVoltage(uint16_t baseAddress,
                                 uint16_t voltage);

extern void LCD_C_setPinAsLCDFunction(uint16_t baseAddress,
                                      uint8_t pin);

extern void LCD_C_setPinAsPortFunction(uint16_t baseAddress,
                                       uint8_t pin);

extern void LCD_C_setPinAsLCDFunctionEx(uint16_t baseAddress,
                                        uint8_t startPin,
                                        uint8_t endPin);

extern void LCD_C_setMemory(uint16_t baseAddress,
                            uint8_t pin,
                            uint8_t value);

extern void LCD_C_setBlinkingMemory(uint16_t baseAddress,
                                    uint8_t pin,
                                    uint8_t value);

extern void LCD_C_configChargePump(uint16_t baseAddress,
                                   uint16_t syncToClock,
                                   uint16_t functionControl);

extern void LCD_C_setMemoryCharacter(uint16_t baseAddress,
                                     uint8_t startPin,
                                     uint8_t character);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_LCD_C_H__
