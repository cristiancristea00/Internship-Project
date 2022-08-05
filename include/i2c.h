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

#define UINT8(X) ((uint8_t) (X))

#define I2C_DATA_bm       UINT8(0x01)

#define I2C_ADRESS_MIN    UINT8(0x00)
#define I2C_ADRESS_MAX    UINT8(0x7F)

typedef enum I2C_ERROR_CODE
{
    I2C_OK                  = 0x00,
    I2C_NULL_POINTER        = 0x01,
    I2C_INVALID_ADDRESS     = 0x02,
    I2C_NACK_OF_ADDRESS     = 0x03,
    I2C_COMMUNICATION_ERROR = 0x04
} i2c_error_code_t;

typedef enum I2C_STATE
{
    I2C_INIT   = 0x00,
    I2C_ACKED  = 0x01,
    I2C_NACKED = 0x02,
    I2C_READY  = 0x03,
    I2C_ERROR  = 0x04
} i2c_state_t;


typedef enum I2C_MODE
{
    I2C_STANDARD_MODE  = 101,
    I2C_FAST_MODE      = 21,
    I2C_FAST_MODE_PLUS = 6
} i2c_mode_t;


typedef enum I2C_DATA_DIRECTION
{
    I2C_DATA_SEND    = 0x00,
    I2C_DATA_RECEIVE = 0x01
} i2c_data_direction_t;


typedef void (* i2c_initialize_t) (i2c_mode_t const);
typedef int8_t (* i2c_send_data_t) (uint8_t const, uint8_t const *, uint8_t);
typedef int8_t (* i2c_receive_data_t) (uint8_t const, uint8_t *, uint8_t);
typedef void (* i2c_end_transaction_t) (void);
typedef bool (* i2c_client_available_t) (uint8_t const);

/**
 * @brief Object struct for the I2C module
 *
 **/
typedef struct I2C
{
    i2c_initialize_t Initialize;
    i2c_send_data_t SendData;
    i2c_receive_data_t ReceiveData;
    i2c_end_transaction_t EndTransaction;
    i2c_client_available_t ClientAvailable;
} i2c_t;

/**
 * @brief Initialize the I2C module on the TWI0 bus with the given mode.
 *
 * @param[in] mode The mode of the I2C bus: Standard, Fast or Fast Plus
 **/
static void I2C0_Inititialize(i2c_mode_t const mode);

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
 *
 **/
static void I2C0_EndTransation(void);

/**
 * @brief Checks if a device is available on the I2C bus.
 *
 * @param[in] address The address of the device
 *
 * @return true If the device is available
 * @return false If the device is not available
 **/
static bool I2C0_ClientAvailable(uint8_t const clientAddress);

#endif // I2C_H
