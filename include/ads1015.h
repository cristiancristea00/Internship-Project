/**
 *  @file ads1015.h
 *  @author Cristian Cristea - M70957
 *  @date 24 August 2022
 *
 *  @brief Header file for the ADS1015 module
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


#ifndef ADS1015_H
#define	ADS1015_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "i2c.h"

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// Integer macros

#define UINT8(X)                            ((uint8_t) (X))
#define UINT16(X)                           ((uint16_t) (X))

// Principal and secondary I2C addresses of the chip

#define ADS1015_I2C_ADDRESS                 UINT8(0x48)
#define ADS1015_I2C_ADDRESS_SEC             UINT8(0x49)

// Input multiplexer configuration

#define ADS1015_AIN0_AIN1                   UINT16(0x0000)
#define ADS1015_AIN0_AIN3                   UINT16(0x1000)
#define ADS1015_AIN1_AIN3                   UINT16(0x2000)
#define ADS1015_AIN2_AIN3                   UINT16(0x3000)
#define ADS1015_AIN0_GND                    UINT16(0x4000)
#define ADS1015_AIN1_GND                    UINT16(0x5000)
#define ADS1015_AIN2_GND                    UINT16(0x6000)
#define ADS1015_AIN3_GND                    UINT16(0x7000)

// Programmable gain amplifier configuration

#define ADS1015_PGA_6_144V                  UINT16(0x0000)
#define ADS1015_PGA_4_096V                  UINT16(0x0200)
#define ADS1015_PGA_2_048V                  UINT16(0x0400)
#define ADS1015_PGA_1_024V                  UINT16(0x0600)
#define ADS1015_PGA_0_512V                  UINT16(0x0800)
#define ADS1015_PGA_0_256V                  UINT16(0x0A00)

// Configuration mode

#define ADS1015_CONTINUOUS_MODE             UINT16(0x0000)
#define ADS1015_SINGLE_SHOT_MODE            UINT16(0x0100)

// Data rate

#define ADS1015_DATA_RATE_128_SPS           UINT16(0x0000)
#define ADS1015_DATA_RATE_250_SPS           UINT16(0x0020)
#define ADS1015_DATA_RATE_490_SPS           UINT16(0x0040)
#define ADS1015_DATA_RATE_920_SPS           UINT16(0x0060)
#define ADS1015_DATA_RATE_1600_SPS          UINT16(0x0080)
#define ADS1015_DATA_RATE_2400_SPS          UINT16(0x00A0)
#define ADS1015_DATA_RATE_3300_SPS          UINT16(0x00C0)

// Comparator mode

#define ADS1015_COMPARATOR_TRADITIONAL      UINT16(0x0000)
#define ADS1015_COMPARATOR_WINDOW           UINT16(0x0010)

// Comparator polarity

#define ADS1015_COMPARATOR_ACTIVE_LOW       UINT16(0x0000)
#define ADS1015_COMPARATOR_ACTIVE_HIGH      UINT16(0x0008)

// Latching comparator

#define ADS1015_COMPARATOR_NON_LATCHING     UINT16(0x0000)
#define ADS1015_COMPARATOR_LATCHING         UINT16(0x0004)

// Comparator queue and disable

#define ADS1015_COMPARATOR_QUEUE_1_CONV     UINT16(0x0000)
#define ADS1015_COMPARATOR_QUEUE_2_CONV     UINT16(0x0001)
#define ADS1015_COMPARATOR_QUEUE_4_CONV     UINT16(0x0002)
#define ADS1015_COMPARATOR_DISABLE          UINT16(0x0003)

// Data rate delay

#define ADS1015_DATA_RATE_128_SPS_DELAY     10000
#define ADS1015_DATA_RATE_250_SPS_DELAY     8000
#define ADS1015_DATA_RATE_490_SPS_DELAY     5000
#define ADS1015_DATA_RATE_920_SPS_DELAY     3000
#define ADS1015_DATA_RATE_1600_SPS_DELAY    2000
#define ADS1015_DATA_RATE_2400_SPS_DELAY    1500
#define ADS1015_DATA_RATE_3300_SPS_DELAY    1000         


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum ADS1015_ERROR_CODE
{
    ADS1015_OK                  = 0x00,
    ADS1015_DEVICE_NOT_FOUND    = 0x01,
    ADS1015_NULL_POINTER        = 0x02,
    ADS1015_COMMUNICATION_ERROR = 0x03,
} ads1015_error_code_t;

typedef enum ADS1015_POINTER_REGISTER
{
    ADS1015_POINTER_CONVERSION     = 0x00,
    ADS1015_POINTER_CONFIG         = 0x01,
    ADS1015_POINTER_LOW_THRESHOLD  = 0x02,
    ADS1015_POINTER_HIGH_THRESHOLD = 0x03
} ads1015_pointer_register_t;

typedef ads1015_error_code_t (* ads1015_i2c_read_t) (i2c_t const * const, uint8_t const, ads1015_pointer_register_t const, uint8_t * const, uint8_t const);
typedef ads1015_error_code_t (* ads1015_i2c_write_t) (i2c_t const * const, uint8_t const, ads1015_pointer_register_t const, uint8_t const * const, uint8_t const);

typedef struct ADS1015_HANDLER
{
    ads1015_i2c_read_t I2C_Read;
    ads1015_i2c_write_t I2C_Write;
} ads1015_handler_t;

typedef struct ADS1015_DEVICE
{
    // I2C address of the device
    uint8_t i2cAddress;

    // I2C device
    i2c_t const * i2cDevice;

    // Handler
    ads1015_handler_t const * handler;
} ads1015_device_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the ADS1015 device.
 *
 * @param[in, out] device ADS1015 device
 * @param[in]      handler Read and write operations handler
 * @param[in]      i2cDevice I2C device
 * @param[in]      i2cAddress I2C device address
 * @param[in]      configuration The configuration to set
 *
 * @return ads1015_error_code_t Error code
 **/
ads1015_error_code_t ADS1015_Initialize(ads1015_device_t * const device, ads1015_handler_t const * const handler, i2c_t const * const i2cDevice,  uint8_t const i2cAddress, uint16_t const configuration);

/**
 * @brief Sets the configuration of the ADS1015 device.
 *
 * @param[in] device ADS1015 device
 * @param[in] configuration The configuration to set
 *
 * @return ads1015_error_code_t Error code
 **/
ads1015_error_code_t ADS1015_SetConfiguration(ads1015_device_t * const device, uint16_t const configuration);

/**
 * @brief Reads the configuration from the ADS1015 device.
 *
 * @param[in]  device ADS1015 device
 * @param[out] configuration The configuration read
 *
 * @return ads1015_error_code_t Error code
 **/
ads1015_error_code_t ADS1015_GetConfiguration(ads1015_device_t const * const device, uint16_t * const configuration);

/**
 * @brief Sets the comparator's low threshold.
 *
 * @param[in] device ADS1015 device
 * @param[in] lowThreshold The low threshold to set
 *
 * @return ads1015_error_code_t Error code
 **/
ads1015_error_code_t ADS1015_SetLowThreshold(ads1015_device_t const * const device, uint16_t const lowThreshold);

/**
 * @brief Reads the comparator's low threshold.
 *
 * @param[in]  device ADS1015 device
 * @param[out] lowThreshold The low threshold read
 *
 * @return ads1015_error_code_t Error code
 **/
ads1015_error_code_t ADS1015_GetLowThreshold(ads1015_device_t const * const device, uint16_t * const lowThreshold);

/**
 * @brief Sets the comparator's high threshold.
 *
 * @param[in] device ADS1015 device
 * @param[in] highThreshold The high threshold to set
 *
 * @return ads1015_error_code_t
 **/
ads1015_error_code_t ADS1015_SetHighThreshold(ads1015_device_t const * const device, uint16_t const highThreshold);

/**
 * @brief Reads the comparator's high threshold.
 *
 * @param[in] device ADS1015 device
 * @param[out] highThreshold The high threshold read
 *
 * @return ads1015_error_code_t
 **/
ads1015_error_code_t ADS1015_GetHighThreshold(ads1015_device_t const * const device, uint16_t * const highThreshold);

/**
 * @brief Reads the conversion value from the ADS1015 device.
 *
 * @param[in]  device ADS1015 device
 * @param[out] conversion The conversion read
 *
 * @return ads1015_error_code_t
 **/
ads1015_error_code_t ADS1015_GetConversion(ads1015_device_t const * const device, uint16_t * const conversion);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

extern ads1015_handler_t const ADS1015_I2C0_Handler;

#endif // ADS1015_H
