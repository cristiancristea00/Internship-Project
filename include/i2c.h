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

#define I2C_DATA_bm 0x01

#define I2C_ADRESS_MIN 0x00
#define I2C_ADRESS_MAX 0x7F

typedef enum I2C_ERROR_CODE
{
    I2C_OK                  = 0,
    I2C_NULL_POINTER        = 1,
    I2C_INVALID_ADDRESS     = 2,
    I2C_NACK_OF_ADDRESS     = 3,
    I2C_COMMUNICATION_ERROR = 4
} i2c_error_code_t;

typedef enum I2C_STATE
{
    I2C_INIT   = 0,
    I2C_ACKED  = 1,
    I2C_NACKED = 2,
    I2C_READY  = 3,
    I2C_ERROR  = 4
} i2c_state_t;


typedef enum I2C_MODE_BAUD
{
    I2C_STANDARD_MODE  = 101,
    I2C_FAST_MODE      = 21,
    I2C_FAST_MODE_PLUS = 6
} i2c_mode_baud_t;


typedef enum I2C_DATA_DIRECTION
{
    I2C_DATA_SEND    = 0,
    I2C_DATA_RECEIVE = 1
} i2c_data_direction_t;


typedef void (* i2c_init_t) (i2c_mode_baud_t const);
typedef int8_t (* i2c_send_data_t) (uint8_t const, uint8_t const *, uint8_t);
typedef int8_t (* i2c_receive_data_t) (uint8_t const, uint8_t *, uint8_t);
typedef void (* i2c_end_session_t) (void);
typedef bool (* i2c_client_available_t) (uint8_t const);

/**
 * @brief Object struct for the I2C module
 *
 **/
typedef struct I2C
{
    i2c_init_t Init;
    i2c_send_data_t SendData;
    i2c_receive_data_t ReceiveData;
    i2c_end_session_t EndSession;
    i2c_client_available_t ClientAvailable;
} i2c_t;

/**
 * @brief Initialize the I2C module on the TWI0 bus with the given mode.
 *
 * @param modeBaud The mode of the I2C bus: Standard, Fast or Fast Plus
 **/
static void I2C0_Init(i2c_mode_baud_t const modeBaud);

/**
 * @brief Sets the I2C bus address of the device based on the given chip address
 *        and the read/write bit.
 *
 * @param deviceAddress The chip address of the device
 * @param dataDirection The direction of the data: send or receive
 * @return i2c_error_code_t The error code of the operation
 **/
static i2c_error_code_t I2C0_SetAdress(uint8_t const deviceAddress, i2c_data_direction_t const dataDirection);

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
 * @param address The address of the device
 * @param dataForSend Pointer to the data to be sent
 * @param length The length of the data to be sent
 * @return i2c_error_code_t The error code of the operation
 **/
static i2c_error_code_t I2C0_SendData(uint8_t const address, uint8_t const * dataForSend, uint8_t const initLength);

/**
 * @brief Receives a specific number of bytes from the device using the I2C bus.
 *
 * @param address The address of the device
 * @param dataForReceive Pointer to the data to be received
 * @param length The length of the data to be received
 * @return i2c_error_code_t The error code of the operation
 **/
static i2c_error_code_t I2C0_ReceiveData(uint8_t const address, uint8_t * dataForReceive, uint8_t const initLength);

/**
 * @brief Ends the I2C communication by sending a stop condition.
 *
 **/
static void I2C0_EndSession(void);

/**
 * @brief Checks if a device is available on the I2C bus.
 *
 * @param address The address of the device
 * @return true If the device is available
 * @return false If the device is not available
 **/
static bool I2C0_ClientAvailable(uint8_t const address);

#endif // I2C_H
