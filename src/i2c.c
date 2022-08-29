/**
 *  @file i2c.c
 *  @author Cristian Cristea - M70957
 *  @date 22 July 2022
 *
 *  @brief Source file for the I2C module
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

#include "i2c.h"

#include <avr/io.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define UINT8(X) ((uint8_t) (X))

#define I2C_DATA_bm    UINT8(0x01)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum I2C_STATE
{
    I2C_INIT   = 0x00,
    I2C_ACKED  = 0x01,
    I2C_NACKED = 0x02,
    I2C_READY  = 0x03,
    I2C_ERROR  = 0x04
} i2c_state_t;

typedef enum I2C_DATA_DIRECTION
{
    I2C_DATA_SEND    = 0x00,
    I2C_DATA_RECEIVE = 0x01
} i2c_data_direction_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize the I2C module on the TWI0 bus with the given mode.
 *
 * @param[in] mode The mode of the I2C bus: Standard, Fast or Fast Plus
 **/
__attribute__((always_inline)) inline static void I2C0_Inititialize(i2c_mode_t const mode);

/**
 * @brief Sets the I2C bus address of the device based on the given chip address
 *        and the read/write bit.
 *
 * @param[in] deviceAddress The chip address of the device
 * @param[in] dataDirection The direction of the data: send or receive
 *
 * @return i2c_error_code_t The error code of the operation
 **/
static i2c_error_code_t I2C0_SetAdressDirectionBit(uint8_t const deviceAddress, i2c_data_direction_t const dataDirection);

/**
 * @brief Waits for the I2C bus to be ready after write operation.
 *
 * @return i2c_state_t The response of the device: ACK, NACK or ERROR
 **/
static i2c_state_t I2C0_WaitWrite(void);

/**
 * @brief Waits for the I2C bus to be ready after read operation.
 *
 * @return i2c_state_t The response of the device: ACK, NACK or ERROR
 **/
static i2c_state_t I2C0_WaitRead(void);

/**
 * @brief Sends a specific number of bytes to the device using the I2C bus.
 *
 * @param[in] address The address of the device
 * @param[in] dataForSend Pointer to the data to be sent
 * @param[in] length The length of the data to be sent
 *
 * @return i2c_error_code_t The error code of the operation
 **/
static i2c_error_code_t I2C0_SendData(uint8_t const address, uint8_t const * const dataForSend, uint8_t const initialLength);

/**
 * @brief Receives a specific number of bytes from the device using the I2C bus.
 *
 * @param[in]  address The address of the device
 * @param[out] dataForReceive Pointer to the data to be received
 * @param[in]  length The length of the data to be received
 *
 * @return i2c_error_code_t The error code of the operation
 **/
static i2c_error_code_t I2C0_ReceiveData(uint8_t const address, uint8_t * dataForReceive, uint8_t const initialLength);

/**
 * @brief Ends the I2C communication by sending a stop condition.
 **/
__attribute__((always_inline)) inline static void I2C0_EndTransation(void);

/**
 * @brief Checks if a device is available on the I2C bus.
 *
 * @param[in] address The address of the device
 *
 * @return true If the device is available
 * @return false If the device is not available
 **/
static bool I2C0_ClientAvailable(uint8_t const clientAddress);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline static void I2C0_Inititialize(i2c_mode_t const mode)
{
    // Select I2C pins to PC2 - SDA and PC3 - SCL
    PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT2_gc;

    // Enable internal pull-ups
    PORTC.PIN2CTRL = PORT_PULLUPEN_bm;
    PORTC.PIN3CTRL = PORT_PULLUPEN_bm;

    // Host baud rate control
    TWI0.MBAUD = (uint8_t) mode;

    // Enable Fast Mode Plus
    if (mode == I2C_MODE_FAST_PLUS)
    {
        TWI0.CTRLA = TWI_FMPEN_ON_gc;
    }

    // Enable I2C as host
    TWI0.MCTRLA = TWI_ENABLE_bm;

    // Initialise the address register
    TWI0.MADDR = 0x00;

    // Initialise the data register
    TWI0.MDATA = 0x00;

    // Set the bus state to idle
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;

    return;
}

static uint8_t I2C0_SetAdressDirectionBit(uint8_t const deviceAddress, i2c_data_direction_t const dataDirection)
{
    if (dataDirection == I2C_DATA_SEND)
    {
        return (uint8_t) (deviceAddress << 1) & ~I2C_DATA_bm;
    }
    else if (dataDirection == I2C_DATA_RECEIVE)
    {
        return (uint8_t) (deviceAddress << 1) | I2C_DATA_bm;
    }
    else
    {
        return I2C_INVALID_ADDRESS;
    }
}

static i2c_state_t I2C0_WaitWrite(void)
{
    i2c_state_t state = I2C_INIT;

    do
    {
        if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
        {
            // Gets here only in case of bus error or arbitration lost
            state = I2C_ERROR;
        }
        else if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
        {
            if (!(TWI0.MSTATUS & TWI_RXACK_bm))
            {
                // Client responded with ACK
                state = I2C_ACKED;
            }
            else
            {
                // Address sent but no ACK received
                state = I2C_NACKED;
            }
        }
    }
    while (state == I2C_INIT);

    return state;
}

static i2c_state_t I2C0_WaitRead(void)
{
    i2c_state_t state = I2C_INIT;

    do
    {
        if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
        {
            // Gets here only in case of bus error or arbitration lost
            state = I2C_ERROR;
        }
        else if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
        {
            state = I2C_READY;
        }
    }
    while (state == I2C_INIT);

    return state;
}

__attribute__((always_inline)) inline static void I2C0_EndTransation(void)
{
    // Sends STOP condition to the bus and clears the internal state of the host
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;

    return;
}

static i2c_error_code_t I2C0_SendData(uint8_t const address, uint8_t const * const dataForSend, uint8_t const initialLength)
{
    TWI0.MADDR = I2C0_SetAdressDirectionBit(address, I2C_DATA_SEND);

    if (I2C0_WaitWrite() != I2C_ACKED)
    {
        return I2C_NACK_OF_ADDRESS;
    }

    if (dataForSend == NULL)
    {
        return I2C_NULL_POINTER;
    }

    uint8_t bytesSent = 0;
    uint8_t length = initialLength;
    uint8_t const * dataPointer = dataForSend;

    while (length != 0)
    {
        --length;

        TWI0.MDATA = *dataPointer;

        if (I2C0_WaitWrite() == I2C_ACKED)
        {
            ++bytesSent;
            ++dataPointer;
        }
        else
        {
            break;
        }
    }

    if (bytesSent == initialLength)
    {
        return I2C_OK;
    }
    else
    {
        return I2C_COMMUNICATION_ERROR;
    }
}

static i2c_error_code_t I2C0_ReceiveData(uint8_t const address, uint8_t * const dataForReceive, uint8_t const initialLength)
{
    TWI0.MADDR = I2C0_SetAdressDirectionBit(address, I2C_DATA_RECEIVE);

    if (I2C0_WaitWrite() != I2C_ACKED)
    {
        return I2C_NACK_OF_ADDRESS;
    }

    if (dataForReceive == NULL)
    {
        return I2C_NULL_POINTER;
    }

    uint8_t bytesReceived = 0;
    uint8_t length = initialLength;
    uint8_t * dataPointer = dataForReceive;

    while (length != 0)
    {
        --length;

        if (I2C0_WaitRead() == I2C_READY)
        {
            *dataPointer = TWI0.MDATA;

            TWI0.MCTRLB = (length == 0) ? (TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc) : TWI_MCMD_RECVTRANS_gc;

            ++bytesReceived;
            ++dataPointer;
        }
        else
        {
            break;
        }
    }

    if (bytesReceived == initialLength)
    {
        return I2C_OK;
    }
    else
    {
        return I2C_COMMUNICATION_ERROR;
    }
}

static bool I2C0_ClientAvailable(uint8_t const clientAddress)
{
    i2c_error_code_t returnValue = I2C0_SendData(clientAddress, NULL, 0);
    I2C0_EndTransation();
    return (returnValue != I2C_NACK_OF_ADDRESS);
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Module for I2C0
 **/
i2c_t const i2c_0 = {
    .Initialize = I2C0_Inititialize,
    .SendData = I2C0_SendData,
    .ReceiveData = I2C0_ReceiveData,
    .EndTransaction = I2C0_EndTransation,
    .ClientAvailable = I2C0_ClientAvailable
};
