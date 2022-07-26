/**
 *  @file spi.c
 *  @author Cristian Cristea - M70957
 *  @date 17 August 2022
 *
 *  @brief Source file for the SPI module
 *
 *  @copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
 *
 *  Subject to your compliance with these terms, you may use Microchip software
 *  and any derivatives exclusively with Microchip products. You’re responsible
 *  for complying with 3rd party license terms applicable to your use of 3rd
 *  party software (including open source software) that may accompany
 *  Microchip software.
 *
 *  SOFTWARE IS “AS IS.” NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 *  APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
 *  NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 *  INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 *  WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 *  BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 *  FULLEST EXTENT ALLOWED BY LAW, MICROCHIP’S TOTAL LIABILITY ON ALL CLAIMS
 *  RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID
 *  DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 **/


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "spi.h"

#include "config.h"

#include <avr/io.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#pragma switch speed


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize the SPI module on the SPI0 bus.
 **/
__attribute__((always_inline)) inline static void SPI0_Inititialize(void);

/**
 * @brief Enables (active low) chip select on the SPI0 bus.
 *
 * @param[in] chipSelect The corresponding chip select pin
 **/
__attribute__((always_inline)) inline static void SPI0_ClientSelect(spi_chip_select_t const chipSelect);

/**
 * @brief Disables (active low) chip select on the SPI0 bus.
 *
 * @param[in] chipSelect The corresponding chip select pin
 **/
__attribute__((always_inline)) inline static void SPI0_ClientDeselect(spi_chip_select_t const chipSelect);

/**
 * @brief Waits for the SPI0 bus to be ready.
 **/
__attribute__((always_inline)) inline static void SPI0_WaitDataReady(void);

/**
 * @brief Sends a byte over the SPI0 bus.
 *
 * @param[in] byte The byte to send
 **/
__attribute__((always_inline)) inline static void SPI0_SendByte(uint8_t const byte);

/**
 * @brief Receives a byte over the SPI0 bus.
 *
 * @return uint8_t The byte received
 **/
__attribute__((always_inline)) inline static uint8_t SPI0_ReceiveByte(void);

/**
 * @brief Sends a byte over the SPI0 bus and receives a byte back.
 *
 * @param[in] byte The byte to send
 *
 * @return uint8_t The byte received
 **/
__attribute__((always_inline)) inline static uint8_t SPI0_ExchangeByte(uint8_t const byte);

/**
 * @brief Sends a specified number of bytes to the device using the SPI bus.
 *
 * @param[in] dataForSend Pointer to the data to be sent
 * @param[in] initialLength The length of the data to be sent
 *
 * @return spi_error_code_t
 **/
static spi_error_code_t SPI0_SendData(uint8_t const * const dataForSend, uint8_t const initialLength);

/**
 * @brief Receives a specified number of bytes from the device using the SPI bus.
 *
 * @param[out] dataForReceive Pointer to the data to be received
 * @param[in]  initialLength The length of the data to be received
 *
 * @return spi_error_code_t
 **/
static spi_error_code_t SPI0_ReceiveData(uint8_t * const dataForReceive, uint8_t const initialLength);

/**
 * @brief Sends a specified number of bytes to the device using the SPI bus and
 *        receives the same number of bytes back.
 *
 * @param[in, out] dataForExchange Pointer to the data to be sent and received
 * @param[in]      initialLength The length of the data to be sent and received
 *
 * @return spi_error_code_t
 **/
static spi_error_code_t SPI0_ExchangeData(uint8_t * const dataForExchange, uint8_t const initialLength);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline static void SPI0_Inititialize(void)
{
    // PA4 - MOSI - OUT | PA5 - MISO - IN | PA6 - SCK - OUT | PA7 - CS - OUT
    PORTA.DIRSET = PIN4_bm | PIN6_bm | PIN7_bm;
    PORTA.DIRCLR = PIN5_bm;

    // PA4 - MOSI - LOW | PA6 - SCK - HIGH | PA7 - CS - HIGH
    PORTA.OUTCLR = PIN4_bm | PIN6_bm;
    PORTA.OUTSET = PIN7_bm;

    // Disable internal pull-ups
    PORTA.PIN4CTRL &= ~PORT_PULLUPEN_bm;
    PORTA.PIN5CTRL &= ~PORT_PULLUPEN_bm;
    PORTA.PIN6CTRL &= ~PORT_PULLUPEN_bm;
    PORTA.PIN7CTRL &= ~PORT_PULLUPEN_bm;

    SPI0.CTRLB = SPI_SSD_bm | SPI_MODE_0_gc;

    SPI0.CTRLA = SPI_MASTER_bm | SPI_CLK2X_bm | SPI_PRESC_DIV4_gc | SPI_ENABLE_bm;

    return;
}

static spi_error_code_t SPI0_SendData(uint8_t const * const dataForSend, uint8_t const initialLength)
{
    if (dataForSend == NULL)
    {
        return SPI_NULL_POINTER;
    }

    uint8_t length = initialLength;
    uint8_t const * dataPointer = dataForSend;

    while (length != 0)
    {
        SPI0_SendByte(*dataPointer);

        SPI0_WaitDataReady();

        ++dataPointer;

        --length;
    }

    return SPI_OK;
}

static spi_error_code_t SPI0_ReceiveData(uint8_t * const dataForReceive, uint8_t const initialLength)
{
    if (dataForReceive == NULL)
    {
        return SPI_NULL_POINTER;
    }

    uint8_t length = initialLength;
    uint8_t * dataPointer = dataForReceive;

    while (length != 0)
    {
        *dataPointer = SPI0_ExchangeByte(0x00);

        ++dataPointer;

        --length;
    }

    return SPI_OK;
}

static spi_error_code_t SPI0_ExchangeData(uint8_t * const dataForExchange, uint8_t const initialLength)
{
    if (dataForExchange == NULL)
    {
        return SPI_NULL_POINTER;
    }

    uint8_t length = initialLength;
    uint8_t * dataPointer = dataForExchange;

    while (length != 0)
    {
        *dataPointer = SPI0_ExchangeByte(*dataPointer);

        ++dataPointer;

        --length;
    }

    return SPI_OK;
}

__attribute__((always_inline)) inline static void SPI0_ClientSelect(spi_chip_select_t const chipSelect)
{
    switch (chipSelect)
    {
        case SPI_CS1:
            PORTA.OUTCLR = PIN7_bm;
            break;
        case SPI_CS2:
            PORTE.OUTCLR = PIN2_bm;
            break;
        case SPI_CS3:
            PORTE.OUTCLR = PIN3_bm;
            break;
        default:
            LOG_ERROR("Invalid SPI chip select pin");
            break;
    }

    return;
}

__attribute__((always_inline)) inline static void SPI0_ClientDeselect(spi_chip_select_t const chipSelect)
{
    switch (chipSelect)
    {
        case SPI_CS1:
            PORTA.OUTSET = PIN7_bm;
            break;
        case SPI_CS2:
            PORTE.OUTSET = PIN2_bm;
            break;
        case SPI_CS3:
            PORTE.OUTSET = PIN3_bm;
            break;
        default:
            LOG_WARNING("Invalid SPI chip select pin");
            break;
    }

    return;
}

__attribute__((always_inline)) inline static void SPI0_WaitDataReady(void)
{
    while (!(SPI0.INTFLAGS & SPI_RXCIF_bm))
    {
        TightLoopContents();
    }

    return;
}

__attribute__((always_inline)) inline static void SPI0_SendByte(uint8_t const byte)
{
    SPI0.DATA = byte;

    return;
}

__attribute__((always_inline)) inline static uint8_t SPI0_ReceiveByte(void)
{
    return SPI0.DATA;
}

__attribute__((always_inline)) inline static uint8_t SPI0_ExchangeByte(uint8_t const byte)
{
    SPI0_SendByte(byte);

    SPI0_WaitDataReady();

    return SPI0_ReceiveByte();
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Module for SPI0
 **/
spi_t const spi_0 = {
    .Initialize = SPI0_Inititialize,
    .SendData = SPI0_SendData,
    .ReceiveData = SPI0_ReceiveData,
    .ExchangeData = SPI0_ExchangeData,
    .ClientSelect = SPI0_ClientSelect,
    .ClientDeselect = SPI0_ClientDeselect
};
