/**
 *  @file i2c.h
 *  @author Cristian Cristea - M70957
 *  @date 22 July 2022
 *
 *  @brief Header file for the I2C module
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


#ifndef I2C_H
#define	I2C_H

#include <avr/io.h>

#include <stddef.h>
#include <stdbool.h>

#define I2C_NACK_OF_ADDRESS -1

#define I2C_INVALID_ADDRESS 0xFF

#define I2C_DATA_bm 0x01

#define I2C_ADRESS_MIN 0x00
#define I2C_ADRESS_MAX 0x7F

enum I2C_STATE
{
    I2C_INIT   = 0,
    I2C_ACKED  = 1,
    I2C_NACKED = 2,
    I2C_READY  = 3,
    I2C_ERROR  = 4
};

typedef enum I2C_MODE_BAUD
{
    I2C_STANDARD_MODE  = 115,
    I2C_FAST_MODE      = 25,
    I2C_FAST_MODE_PLUS = 7
} i2c_mode_baud_t;

typedef enum I2C_DATA_DIRECTION
{
    I2C_DATA_SEND    = 0,
    I2C_DATA_RECEIVE = 1
} i2c_data_direction_t;

void I2c0Init(i2c_mode_baud_t const modeBaud);

static uint8_t I2c0SetAdress(uint8_t const deviceAddress, i2c_data_direction_t const dataDirection);

static uint8_t I2c0WaitWrite(void);

static uint8_t I2c0WaitRead(void);

int8_t I2c0SendData(uint8_t const address, uint8_t const * dataForSend, uint8_t const length);

int8_t I2c0ReceiveData(uint8_t const address, uint8_t * dataForReceive, uint8_t const length);

void I2c0EndSession(void);

bool I2c0ClientAvailable(uint8_t const address);

#endif // I2C_H
