/**
 *  @file bme280.c
 *  @author Cristian Cristea - M70957
 *  @date July 25, 2022
 *
 *  @brief Source file for the BME280 module
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


#include "bme280.h"


bme280_handler_t const defaultHandler = {
    .Read = Bm280ReadRegisters,
    .Write = Bm280WriteRegister,
};

static bme280_error_code_t Bme280CheckNull(bme280_device_t const * const device)
{
    if (device == NULL || device->handler == NULL)
    {
        return BME280_NULL_POINTER;
    }
    else
    {
        return BME280_OK;
    }
}

static bme280_error_code_t Bm280ReadRegisters(i2c_t * const i2c, uint8_t const address, uint8_t const registerAddress, uint8_t * const data, uint8_t const length)
{
    bme280_error_code_t readResult = BME280_OK;

    if (i2c->SendData(address, &registerAddress, 1) != I2C_OK)
    {
        readResult = BME280_COMMUNICATION_ERROR;
    }

    if (i2c->ReceiveData(address, data, length) != I2C_OK)
    {
        readResult = BME280_COMMUNICATION_ERROR;
    }

    i2c->EndSession();

    return readResult;
}

static bme280_error_code_t Bm280WriteRegister(i2c_t * const i2c, uint8_t const address, uint8_t const registerAddress, uint8_t const * const data)
{
    bme280_error_code_t writeResult = BME280_OK;

    if (i2c->SendData(address, &registerAddress, 1) != I2C_OK)
    {
        writeResult = BME280_COMMUNICATION_ERROR;
    }
    if (i2c->SendData(address, data, 1) != I2C_OK)
    {
        writeResult = BME280_COMMUNICATION_ERROR;
    }

    i2c->EndSession();

    return writeResult;
}

bme280_error_code_t Bme280Init(bme280_device_t * const device, bme280_handler_t const * const handler, i2c_t const * const i2cDevice, uint8_t const i2cAddress)
{
    device->i2cDevice = NULL;
    device->i2cAddress = i2cAddress;
    device->handler = handler;

    bme280_error_code_t initResult = BME280_OK;

    uint8_t tryCount = 10;
    uint8_t chipId = 0;

    initResult = Bme280CheckNull(device);

    if (initResult == BME280_OK)
    {
        device->i2cDevice = i2cDevice;

        while (tryCount != 0)
        {
            initResult = Bme280GetRegisters(device, BME280_CHIP_ID_ADDRESS, &chipId, 1);

            printf("Chip ID: 0x%x", chipId);

            if (initResult == BME280_OK && chipId == BME280_CHIP_ID)
            {
                /*
                TODO
                initResult = Bme280SoftReset(device);
                 */

                if (initResult == BME280_OK)
                {
                    /*
                    TODO
                    initResult = Bm280GetCalibrationData(device);
                     */
                }

                break;
            }

            --tryCount;
            _delay_ms(1);
        }

        if (tryCount == 0)
        {
            initResult = BME280_DEVICE_NOT_FOUND;
        }
    }

    return initResult;
}

bme280_error_code_t Bme280GetRegisters(bme280_device_t * const device, uint8_t const registerAddress, uint8_t * data, uint8_t const length)
{
    bme280_error_code_t readResult = BME280_OK;

    readResult = Bme280CheckNull(device);

    if (readResult == BME280_OK && data != NULL)
    {
        if (device->handler->Read(device->i2cDevice, device->i2cAddress, registerAddress, data, length) != BME280_OK)
        {
            readResult = BME280_COMMUNICATION_ERROR;
        }
    }
    else
    {
        readResult = BME280_NULL_POINTER;
    }

    return readResult;
}
