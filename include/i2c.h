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


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <stdbool.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define I2C_ADRESS_MIN    UINT8(0x00)
#define I2C_ADRESS_MAX    UINT8(0x7F)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum I2C_ERROR_CODE
{
    I2C_OK                  = 0x00,
    I2C_NULL_POINTER        = 0x01,
    I2C_INVALID_ADDRESS     = 0x02,
    I2C_NACK_OF_ADDRESS     = 0x03,
    I2C_COMMUNICATION_ERROR = 0x04
} i2c_error_code_t;

typedef enum I2C_MODE
{
    I2C_STANDARD_MODE  = 101,
    I2C_FAST_MODE      = 21,
    I2C_FAST_MODE_PLUS = 6
} i2c_mode_t;

typedef void (* i2c_initialize_t) (i2c_mode_t const);
typedef i2c_error_code_t (* i2c_send_data_t) (uint8_t const, uint8_t const *, uint8_t);
typedef i2c_error_code_t (* i2c_receive_data_t) (uint8_t const, uint8_t *, uint8_t);
typedef void (* i2c_end_transaction_t) (void);
typedef bool (* i2c_client_available_t) (uint8_t const);

/**
 * @brief Object struct for the I2C module
 **/
typedef struct I2C
{
    i2c_initialize_t Initialize;
    i2c_send_data_t SendData;
    i2c_receive_data_t ReceiveData;
    i2c_end_transaction_t EndTransaction;
    i2c_client_available_t ClientAvailable;
} i2c_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

extern i2c_t const i2c_0;

#endif // I2C_H
