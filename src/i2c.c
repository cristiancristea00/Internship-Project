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


#include "i2c.h"


/**
 * @brief Module for I2C0
 *
 **/
i2c_t const i2c_0 = {
    .Init = I2C0_Init,
    .SendData = I2C0_SendData,
    .ReceiveData = I2C0_ReceiveData,
    .EndSession = I2C0_EndSession,
    .ClientAvailable = I2C0_ClientAvailable
};

static void I2C0_Init(i2c_mode_baud_t const modeBaud)
{
    // Select I2C pins to PC2 - SDA and PC3 - SCL
    PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT2_gc;

    // Enable internal pull-ups
    PORTC.PIN2CTRL = PORT_PULLUPEN_bm;
    PORTC.PIN3CTRL = PORT_PULLUPEN_bm;

    // Host baud rate control
    TWI0.MBAUD = modeBaud;

    // Enable Fast Mode Plus
    if (modeBaud == I2C_FAST_MODE_PLUS)
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

static uint8_t I2C0_SetAdress(uint8_t const deviceAddress, i2c_data_direction_t const dataDirection)
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

static void I2C0_EndSession(void)
{
    // Sends STOP condition to the bus and clears the internal state of the host
    // TWI0.MCTRLB = (TWI_MCMD_STOP_gc | TWI_FLUSH_bm);
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;

    return;
}

static i2c_error_code_t I2C0_SendData(uint8_t const address, uint8_t const * dataForSend, uint8_t const initLength)
{
    TWI0.MADDR = I2C0_SetAdress(address, I2C_DATA_SEND);

    if (I2C0_WaitWrite() != I2C_ACKED)
    {
        return I2C_NACK_OF_ADDRESS;
    }

    if (dataForSend == NULL)
    {
        return I2C_NULL_POINTER;
    }

    uint8_t bytesSent = 0;
    uint8_t length = initLength;

    while (length != 0)
    {
        --length;

        TWI0.MDATA = *dataForSend;

        if (I2C0_WaitWrite() == I2C_ACKED)
        {
            ++bytesSent;
            ++dataForSend;
        }
        else
        {
            break;
        }
    }

    if (bytesSent == initLength)
    {
        return I2C_OK;
    }
    else
    {
        return I2C_COMMUNICATION_ERROR;
    }
}

static i2c_error_code_t I2C0_ReceiveData(uint8_t const address, uint8_t * dataForReceive, uint8_t const initLength)
{
    TWI0.MADDR = I2C0_SetAdress(address, I2C_DATA_RECEIVE);

    if (I2C0_WaitWrite() != I2C_ACKED)
    {
        return I2C_NACK_OF_ADDRESS;
    }

    if (dataForReceive == NULL)
    {
        return I2C_NULL_POINTER;
    }

    uint8_t bytesReceived = 0;
    uint8_t length = initLength;

    while (length != 0)
    {
        --length;

        if (I2C0_WaitRead() == I2C_READY)
        {
            *dataForReceive = TWI0.MDATA;

            TWI0.MCTRLB = (length == 0) ? (TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc) : TWI_MCMD_RECVTRANS_gc;

            ++bytesReceived;
            ++dataForReceive;
        }
        else
        {
            break;
        }
    }



    if (bytesReceived == initLength)
    {
        return I2C_OK;
    }
    else
    {
        return I2C_COMMUNICATION_ERROR;
    }
}

static bool I2C0_ClientAvailable(uint8_t const address)
{
    i2c_error_code_t returnValue = I2C0_SendData(address, NULL, 0);
    I2C0_EndSession();
    return (returnValue != I2C_NACK_OF_ADDRESS);
}
