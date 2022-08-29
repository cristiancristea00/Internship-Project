/**
 *  @file ads1015.c
 *  @author Cristian Cristea - M70957
 *  @date 24 August 2022
 *
 *  @brief Source file for the ADS1015 module
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

#include "ads1015.h"

#include "config.h"
#include "i2c.h"

#include <stddef.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Checks if the device, its bus handler and I2C device are valid.
 *
 * @param[in] device ADS1015 device
 *
 * @return ads1015_error_code_t Error code
 **/
__attribute__((always_inline)) inline static ads1015_error_code_t ADS1015_CheckNull(ads1015_device_t const * const device);

/**
 * @brief Reads a register by selecting it with the help of the Pointer.
 *
 * @param[in]  i2c The I2C device to use
 * @param[in]  address The address of the device
 * @param[in]  pointerRegister The register to read
 * @param[out] dataBuffer The buffer to store the data in
 * @param[in]  bufferLength The number of bytes to read
 *
 * @return ads1015_error_code_t Error code
 **/
static ads1015_error_code_t ADS1015_I2C_ReadRegister(i2c_t const * const i2c, uint8_t const address, ads1015_pointer_register_t const pointerRegister, uint8_t * const dataBuffer, uint8_t const bufferLength);

/**
 * @brief Writes a register by selecting it with the help of the Pointer.
 *
 * @param[in] i2c The I2C device to use
 * @param[in] address The address of the device
 * @param[in] pointerRegister The register to write
 * @param[in] dataBuffer The buffer containing the data to write
 * @param[in] bufferLength The number of bytes to write
 *
 * @return ads1015_error_code_t Error code
 **/
static ads1015_error_code_t ADS1015_I2C_WriteRegister(i2c_t const * const i2c, uint8_t const address, ads1015_pointer_register_t const pointerRegister, uint8_t const * const dataBuffer, uint8_t const bufferLength);

/**
 * @brief Sets a word (16 bits) value in the specified register.
 *
 * @param[in] device ADS1015 device
 * @param[in] word The word to set
 * @param[in] pointer Register to set
 *
 * @return ads1015_error_code_t Error code
 **/
__attribute__((always_inline)) inline static ads1015_error_code_t ADS1015_SetWordValue(ads1015_device_t const * const device, uint16_t const word, ads1015_pointer_register_t const pointer);

/**
 * @brief Gets a word (16 bits) value from the specified register.
 *
 * @param[in] device ADS1015 device
 * @param[in] word The word to get
 * @param[in] pointer Register to get
 *
 * @return ads1015_error_code_t Error code
 **/
__attribute__((always_inline)) inline static ads1015_error_code_t ADS1015_GetWordValue(ads1015_device_t const * const device, uint16_t * const word, ads1015_pointer_register_t const pointer);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline static ads1015_error_code_t ADS1015_CheckNull(ads1015_device_t const * const device)
{
    if (device == NULL || device->handler == NULL || device->i2cDevice == NULL)
    {
        LOG_ERROR("Found NULL pointer on ADS1015");
        return ADS1015_NULL_POINTER;
    }
    else
    {
        return ADS1015_OK;
    }
}

static ads1015_error_code_t ADS1015_I2C_ReadRegister(i2c_t const * const i2c, uint8_t const address, ads1015_pointer_register_t const pointerRegister, uint8_t * const dataBuffer, uint8_t const bufferLength)
{
    ads1015_error_code_t readResult = ADS1015_OK;

    if (i2c->SendData(address, &pointerRegister, 1) != I2C_OK)
    {
        readResult = ADS1015_COMMUNICATION_ERROR;
    }

    i2c->EndTransaction();

    if (i2c->ReceiveData(address, dataBuffer, bufferLength) != I2C_OK)
    {
        readResult = ADS1015_COMMUNICATION_ERROR;
    }

    i2c->EndTransaction();

    return readResult;
}

static ads1015_error_code_t ADS1015_I2C_WriteRegister(i2c_t const * const i2c, uint8_t const address, ads1015_pointer_register_t const pointerRegister, uint8_t const * const dataBuffer, uint8_t const bufferLength)
{
    ads1015_error_code_t writeResult = ADS1015_OK;

    uint8_t const newBufferLength = bufferLength + 1;
    uint8_t newDataBuffer[newBufferLength];

    newDataBuffer[0] = (uint8_t) pointerRegister;

    for (uint8_t registerIdx = 1; registerIdx < newBufferLength; ++registerIdx)
    {
        newDataBuffer[registerIdx] = dataBuffer[registerIdx - 1];
    }

    if (i2c->SendData(address, newDataBuffer, newBufferLength) != I2C_OK)
    {
        writeResult = ADS1015_COMMUNICATION_ERROR;
    }

    i2c->EndTransaction();

    return writeResult;
}

__attribute__((always_inline)) inline static ads1015_error_code_t ADS1015_SetWordValue(ads1015_device_t const * const device, uint16_t const word, ads1015_pointer_register_t const pointer)
{
    ads1015_error_code_t setWordResult = ADS1015_OK;

    uint8_t const data[2] = { word >> 8, word & 0x00FF };

    setWordResult = device->handler->I2C_Write(device->i2cDevice, device->i2cAddress, pointer, data, 2);

    return setWordResult;
}

__attribute__((always_inline)) inline static ads1015_error_code_t ADS1015_GetWordValue(ads1015_device_t const * const device, uint16_t * const word, ads1015_pointer_register_t const pointer)
{
    ads1015_error_code_t getWordResult = ADS1015_OK;

    uint8_t data[2] = { 0 };

    getWordResult = device->handler->I2C_Read(device->i2cDevice, device->i2cAddress, pointer, data, 2);

    uint16_t const mostSignificantByte  = ((uint16_t) data[0]) << 8;
    uint16_t const leastSignificantByte = (uint16_t) data[1];
    *word = mostSignificantByte | leastSignificantByte;

    return getWordResult;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

ads1015_error_code_t ADS1015_Initialize(ads1015_device_t * const device, ads1015_handler_t const * const handler, i2c_t const * const i2cDevice,  uint8_t const i2cAddress, uint16_t const configuration)
{
    device->i2cDevice = i2cDevice;
    device->i2cAddress = i2cAddress;
    device->handler = handler;

    ads1015_error_code_t initResult = ADS1015_OK;

    LOG_INFO("Started ADS1015 initialization");

    initResult = ADS1015_CheckNull(device);

    if (initResult == ADS1015_OK)
    {
        initResult = ADS1015_SetConfiguration(device, configuration);
    }

    if (initResult == ADS1015_OK)
    {
        LOG_INFO("Finished the ADS1015 initialization");
    }
    else
    {
        LOG_ERROR("Couldn't finish the ADS1015 initialization");
    }

    return initResult;
}

ads1015_error_code_t ADS1015_SetConfiguration(ads1015_device_t * const device, uint16_t const configuration)
{
    return ADS1015_SetWordValue(device, configuration, ADS1015_POINTER_CONFIG);
}

ads1015_error_code_t ADS1015_GetConfiguration(ads1015_device_t const * const device, uint16_t * const configuration)
{
    return ADS1015_GetWordValue(device, configuration, ADS1015_POINTER_CONFIG);
}

ads1015_error_code_t ADS1015_SetLowThreshold(ads1015_device_t const * const device, uint16_t const lowThreshold)
{
    return ADS1015_SetWordValue(device, lowThreshold, ADS1015_POINTER_LOW_THRESHOLD);
}

ads1015_error_code_t ADS1015_GetLowThreshold(ads1015_device_t const * const device, uint16_t * const lowThreshold)
{
    return ADS1015_GetWordValue(device, lowThreshold, ADS1015_POINTER_LOW_THRESHOLD);
}

ads1015_error_code_t ADS1015_SetHighThreshold(ads1015_device_t const * const device, uint16_t const highThreshold)
{
    return ADS1015_SetWordValue(device, highThreshold, ADS1015_POINTER_HIGH_THRESHOLD);
}

ads1015_error_code_t ADS1015_GetHighThreshold(ads1015_device_t const * const device, uint16_t * const highThreshold)
{
    return ADS1015_GetWordValue(device, highThreshold, ADS1015_POINTER_HIGH_THRESHOLD);
}

ads1015_error_code_t ADS1015_GetConversion(ads1015_device_t const * const device, uint16_t * const conversion)
{
    ads1015_error_code_t conversionResult = ADS1015_GetWordValue(device, conversion, ADS1015_POINTER_CONVERSION);

    *conversion >>= 4;

    return conversionResult;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

ads1015_handler_t const ADS1015_I2C0_Handler = {
    .I2C_Read = ADS1015_I2C_ReadRegister,
    .I2C_Write = ADS1015_I2C_WriteRegister
};
