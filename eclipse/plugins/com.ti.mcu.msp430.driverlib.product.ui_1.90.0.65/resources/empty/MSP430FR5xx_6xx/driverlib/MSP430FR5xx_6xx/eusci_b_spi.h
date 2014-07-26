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
// eusci_b_spi.h - Driver for the EUSCI_B_SPI Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_EUSCI_B_SPI_H__
#define __MSP430WARE_EUSCI_B_SPI_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_EUSCI_Bx__

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
// The following is a struct that is passed to EUSCI_B_SPI_initMaster()
//
//******************************************************************************
typedef struct EUSCI_B_SPI_initMasterParam {
    uint8_t selectClockSource;
    uint32_t clockSourceFrequency;
    uint32_t desiredSpiClock;
    uint16_t msbFirst;
    uint16_t clockPhase;
    uint16_t clockPolarity;
    uint16_t spiMode;
} EUSCI_B_SPI_initMasterParam;

//******************************************************************************
//
// The following is a struct that is passed to EUSCI_B_SPI_initSlave()
//
//******************************************************************************
typedef struct EUSCI_B_SPI_initSlaveParam {
    uint16_t msbFirst;
    uint16_t clockPhase;
    uint16_t clockPolarity;
    uint16_t spiMode;
} EUSCI_B_SPI_initSlaveParam;

//******************************************************************************
//
// The following is a struct that is passed to EUSCI_B_SPI_changeMasterClock()
//
//******************************************************************************
typedef struct EUSCI_B_SPI_changeMasterClockParam {
    uint32_t clockSourceFrequency;
    uint32_t desiredSpiClock;
} EUSCI_B_SPI_changeMasterClockParam;

//*****************************************************************************
//
// The following are values that can be passed to the clockPhase parameter for
// functions: EUSCI_B_SPI_masterInit(), EUSCI_B_SPI_slaveInit(), and
// EUSCI_B_SPI_changeClockPhasePolarity().
//
//*****************************************************************************
#define EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT            0x00
#define EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT          UCCKPH

//*****************************************************************************
//
// The following are values that can be passed to the msbFirst parameter for
// functions: EUSCI_B_SPI_masterInit(), and EUSCI_B_SPI_slaveInit().
//
//*****************************************************************************
#define EUSCI_B_SPI_MSB_FIRST                                             UCMSB
#define EUSCI_B_SPI_LSB_FIRST                                              0x00

//*****************************************************************************
//
// The following are values that can be passed to the clockPolarity parameter
// for functions: EUSCI_B_SPI_masterInit(), EUSCI_B_SPI_slaveInit(), and
// EUSCI_B_SPI_changeClockPhasePolarity().
//
//*****************************************************************************
#define EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH                        UCCKPL
#define EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW                           0x00

//*****************************************************************************
//
// The following are values that can be passed to the selectClockSource
// parameter for functions: EUSCI_B_SPI_masterInit().
//
//*****************************************************************************
#define EUSCI_B_SPI_CLOCKSOURCE_ACLK                               UCSSEL__ACLK
#define EUSCI_B_SPI_CLOCKSOURCE_SMCLK                             UCSSEL__SMCLK

//*****************************************************************************
//
// The following are values that can be passed to the spiMode parameter for
// functions: EUSCI_B_SPI_masterInit(), and EUSCI_B_SPI_slaveInit().
//
//*****************************************************************************
#define EUSCI_B_SPI_3PIN                                               UCMODE_0
#define EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_HIGH                            UCMODE_1
#define EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW                             UCMODE_2

//*****************************************************************************
//
// The following are values that can be passed to the select4PinFunctionality
// parameter for functions: EUSCI_B_SPI_select4PinFunctionality().
//
//*****************************************************************************
#define EUSCI_B_SPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS                   0x00
#define EUSCI_B_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE                        UCSTEM

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: EUSCI_B_SPI_enableInterrupt(), EUSCI_B_SPI_disableInterrupt(),
// EUSCI_B_SPI_getInterruptStatus(), and EUSCI_B_SPI_clearInterruptFlag() as
// well as returned by the EUSCI_B_SPI_getInterruptStatus() function.
//
//*****************************************************************************
#define EUSCI_B_SPI_TRANSMIT_INTERRUPT                                   UCTXIE
#define EUSCI_B_SPI_RECEIVE_INTERRUPT                                    UCRXIE

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the EUSCI_B_SPI_isBusy() function.
//
//*****************************************************************************
#define EUSCI_B_SPI_BUSY                                                 UCBUSY
#define EUSCI_B_SPI_NOT_BUSY                                               0x00

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void EUSCI_B_SPI_initMaster(uint16_t baseAddress,
                                   EUSCI_B_SPI_initMasterParam *param);

extern void EUSCI_B_SPI_select4PinFunctionality(uint16_t baseAddress,
                                                uint8_t select4PinFunctionality);

extern void EUSCI_B_SPI_changeMasterClock(uint16_t baseAddress,
                                          EUSCI_B_SPI_changeMasterClockParam *param);

extern void EUSCI_B_SPI_initSlave(uint16_t baseAddress,
                                  EUSCI_B_SPI_initSlaveParam *param);

extern void EUSCI_B_SPI_changeClockPhasePolarity(uint16_t baseAddress,
                                                 uint16_t clockPhase,
                                                 uint16_t clockPolarity);

extern void EUSCI_B_SPI_transmitData(uint16_t baseAddress,
                                     uint8_t transmitData);

extern uint8_t EUSCI_B_SPI_receiveData(uint16_t baseAddress);

extern void EUSCI_B_SPI_enableInterrupt(uint16_t baseAddress,
                                        uint8_t mask);

extern void EUSCI_B_SPI_disableInterrupt(uint16_t baseAddress,
                                         uint8_t mask);

extern uint8_t EUSCI_B_SPI_getInterruptStatus(uint16_t baseAddress,
                                              uint8_t mask);

extern void EUSCI_B_SPI_clearInterruptFlag(uint16_t baseAddress,
                                           uint8_t mask);

extern void EUSCI_B_SPI_enable(uint16_t baseAddress);

extern void EUSCI_B_SPI_disable(uint16_t baseAddress);

extern uint32_t EUSCI_B_SPI_getReceiveBufferAddress(uint16_t baseAddress);

extern uint32_t EUSCI_B_SPI_getTransmitBufferAddress(uint16_t baseAddress);

extern uint16_t EUSCI_B_SPI_isBusy(uint16_t baseAddress);

//*****************************************************************************
//
// The following are deprecated APIs.
//
//*****************************************************************************
#define EUSCI_B_SPI_getTransmitBufferAddressForDMA                            \
                                           EUSCI_B_SPI_getTransmitBufferAddress

//*****************************************************************************
//
// The following are deprecated APIs.
//
//*****************************************************************************
#define EUSCI_B_SPI_getReceiveBufferAddressForDMA                             \
                                            EUSCI_B_SPI_getReceiveBufferAddress

//*****************************************************************************
//
// The following are deprecated APIs.
//
//*****************************************************************************
extern void EUSCI_B_SPI_masterInit(uint16_t baseAddress,
                                   uint8_t selectClockSource,
                                   uint32_t clockSourceFrequency,
                                   uint32_t desiredSpiClock,
                                   uint16_t msbFirst,
                                   uint16_t clockPhase,
                                   uint16_t clockPolarity,
                                   uint16_t spiMode);

extern void EUSCI_B_SPI_masterChangeClock(uint16_t baseAddress,
                                          uint32_t clockSourceFrequency,
                                          uint32_t desiredSpiClock);

extern void EUSCI_B_SPI_slaveInit(uint16_t baseAddress,
                                  uint16_t msbFirst,
                                  uint16_t clockPhase,
                                  uint16_t clockPolarity,
                                  uint16_t spiMode);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_EUSCI_B_SPI_H__