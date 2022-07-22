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

void I2c0Init(uint8_t const baudRate)
{
    // Select I2C pins to PC2 - SDA and PC3 - SCL
    PORTMUX.TWIROUTEA = PORTMUX_TWI0_ALT2_gc;

    // Host baud rate control
    TWI0.MBAUD = baudRate;

    // Enable I2C as host
    TWI0.MCTRLA = TWI_ENABLE_bm;

    // Initialise the address register
    TWI0.MADDR = 0x00;

    // Initialise the data register
    TWI0.MDATA = 0x00;

    // Set the bus state to idle
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

static uint8_t I2c0SetAdress(uint8_t const deviceAddress, i2c_data_direction_t const dataDirection)
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

static uint8_t I2c0WaitWrite(void)
{
    uint8_t state = I2C_INIT;

    do
    {
        if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
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
        else if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
        {
            // Gets here only in case of bus error or arbitration lost
            state = I2C_ERROR;
        }
    }
    while (state == I2C_INIT);

    return state;
}

static uint8_t I2c0WaitRead(void)
{
    uint8_t state = I2C_INIT;

    do
    {
        if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
        {
            state = I2C_READY;
        }
        else if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
        {
            // Gets here only in case of bus error or arbitration lost
            state = I2C_ERROR;
        }
    }
    while (state == I2C_INIT);

    return state;
}

int8_t I2c0SendData(uint8_t const address, uint8_t const * dataForSend, uint8_t const length)
{
    int8_t bytesSent = I2C_NACK_OF_ADDRESS;

    TWI0.MADDR = I2c0SetAdress(address, I2C_DATA_SEND);

    if (I2c0WaitWrite() != I2C_ACKED)
    {
        return bytesSent;
    }

    bytesSent = 0;

    if (dataForSend == NULL)
    {
        return bytesSent;
    }

    while (length != 0)
    {
        TWI0.MDATA = *dataForSend;

        if (I2c0WaitWrite() == I2C_ACKED)
        {
            ++bytesSent;
            ++dataForSend;

            continue;
        }
        else
        {
            break;
        }
    }

    return bytesSent;
}

int8_t I2c0ReceiveData(uint8_t const address, uint8_t * dataForReceive, uint8_t const length)
{
    int8_t bytesReceived = I2C_NACK_OF_ADDRESS;

    TWI0.MADDR = I2c0SetAdress(address, I2C_DATA_RECEIVE);

    if (I2c0WaitWrite() != I2C_ACKED)
    {
        return bytesReceived;
    }

    bytesReceived = 0;

    if (dataForReceive == NULL)
    {
        return bytesReceived;
    }

    while (length != 0)
    {
        if (I2c0WaitRead() == I2C_READY)
        {
            *dataForReceive = TWI0.MDATA;

            TWI0.MCTRLB = (length == 0) ? TWI_ACKACT_bm | TWI_MCMD_STOP_gc : TWI_MCMD_RECVTRANS_gc;

            ++bytesReceived;
            ++dataForReceive;

            continue;
        }
        else
        {
            break;
        }
    }

    return bytesReceived;
}

void I2c0EndSession(void)
{
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;

    return;
}

bool I2c0ClientAvailable(uint8_t const address)
{
    uint8_t returnValue = I2c0SendData(address, NULL, 0);
    I2c0EndSession();
    return returnValue != I2C_NACK_OF_ADDRESS;
}
