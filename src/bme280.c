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
    .Write = Bm280WriteRegisters,
};

static bme280_error_code_t Bme280CheckNull(bme280_device_t const * const device)
{
    if (device == NULL || device->handler == NULL)
    {
        LOG_ERROR("Found NULL pointer");
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

static bme280_error_code_t Bm280WriteRegisters(i2c_t * const i2c, uint8_t const address, uint8_t const * const registerAddresses, uint8_t const * const data, uint8_t const length)
{
    bme280_error_code_t writeResult = BME280_OK;

    uint8_t const bufferLength = 2U * length;
    uint8_t dataBuffer[bufferLength];

    for (uint8_t registerIdx = 0; registerIdx < length; ++registerIdx)
    {
        dataBuffer[2 * registerIdx] = registerAddresses[registerIdx];
        dataBuffer[2 * registerIdx + 1] = data[registerIdx];
    }

    if (i2c->SendData(address, &dataBuffer, bufferLength) != I2C_OK)
    {
        writeResult = BME280_COMMUNICATION_ERROR;
    }

    i2c->EndSession();

    return writeResult;
}

static bme280_error_code_t Bme280GetRegisters(bme280_device_t * const device, uint8_t const registerAddress, uint8_t * const data, uint8_t const length)
{
    bme280_error_code_t getResult = BME280_OK;

    getResult = Bme280CheckNull(device);

    if (getResult == BME280_OK && data != NULL)
    {
        if (device->handler->Read(device->i2cDevice, device->i2cAddress, registerAddress, data, length) != BME280_OK)
        {
            getResult = BME280_COMMUNICATION_ERROR;
        }
    }
    else
    {
        getResult = BME280_NULL_POINTER;
    }

    return getResult;
}

static bme280_error_code_t Bme280SetRegisters(bme280_device_t * const device, uint8_t const * const registerAddresses, uint8_t const * const data, uint8_t const length)
{
    bme280_error_code_t setResult = BME280_OK;

    setResult = Bme280CheckNull(device);

    if (setResult == BME280_OK && data != NULL && registerAddresses != NULL)
    {
        if (length != 0)
        {
            if (device->handler->Write(device->i2cDevice, device->i2cAddress, registerAddresses, data, length) != BME280_OK)
            {
                setResult = BME280_COMMUNICATION_ERROR;
            }
        }
        else
        {
            setResult = BME280_INVALID_LENGTH;
        }
    }
    else
    {

        setResult = BME280_NULL_POINTER;
    }

    return setResult;
}

static bme280_error_code_t Bme280SoftReset(bme280_device_t * const device)
{
    bme280_error_code_t resetResult = BME280_OK;

    resetResult = Bme280CheckNull(device);

    if (resetResult == BME280_OK)
    {
        uint8_t const registerAdress = BME280_RESET_ADDRESS;
        uint8_t const resetCommand = BME280_SOFT_RESET_COMMAND;

        resetResult = Bme280SetRegisters(device, &registerAdress, &resetCommand, 1);

        if (resetResult == BME280_OK)
        {
            uint8_t tryCount = 10;
            uint8_t statusRegister = 0;

            do
            {
                _delay_ms(2);
                resetResult = Bme280GetRegisters(device, BME280_STATUS_REGISTER_ADDRESS, &statusRegister, 1);
                --tryCount;
            }
            while ((resetResult == BME280_OK) && (tryCount != 0) && (statusRegister & BME280_STATUS_UPDATE));

            if (statusRegister & BME280_STATUS_UPDATE)
            {
                resetResult = BME280_NVM_COPY_FAILED;
            }
        }
    }

    return resetResult;
}

static void Bme280ParseTemperatureAndPressureCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData)
{
    // Temperature

    calibrationData->coefTemperature1 = (uint16_t) BME280_CONCAT_BYTES(rawData[1], rawData[0]);
    calibrationData->coefTemperature2 = (int16_t) BME280_CONCAT_BYTES(rawData[3], rawData[2]);
    calibrationData->coefTemperature3 = (int16_t) BME280_CONCAT_BYTES(rawData[5], rawData[4]);

    //Pressure

    calibrationData->coefPressure1 = (uint16_t) BME280_CONCAT_BYTES(rawData[7], rawData[6]);
    calibrationData->coefPressure2 = (int16_t) BME280_CONCAT_BYTES(rawData[9], rawData[8]);
    calibrationData->coefPressure3 = (int16_t) BME280_CONCAT_BYTES(rawData[11], rawData[10]);
    calibrationData->coefPressure4 = (int16_t) BME280_CONCAT_BYTES(rawData[13], rawData[12]);
    calibrationData->coefPressure5 = (int16_t) BME280_CONCAT_BYTES(rawData[15], rawData[14]);
    calibrationData->coefPressure6 = (int16_t) BME280_CONCAT_BYTES(rawData[17], rawData[16]);
    calibrationData->coefPressure7 = (int16_t) BME280_CONCAT_BYTES(rawData[19], rawData[18]);
    calibrationData->coefPressure8 = (int16_t) BME280_CONCAT_BYTES(rawData[21], rawData[20]);
    calibrationData->coefPressure9 = (int16_t) BME280_CONCAT_BYTES(rawData[23], rawData[22]);

    // One lonely humidity coefficient

    calibrationData->coefHumidity1 = (uint8_t) rawData[25];
}

static void Bme280ParseHumidityCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData)
{
    // Humidity

    calibrationData->coefHumidity2 = (int16_t) BME280_CONCAT_BYTES(rawData[1], rawData[0]);
    calibrationData->coefHumidity3 = (uint8_t) rawData[2];

    int16_t coefHumidity4_msb = (int16_t) ((int8_t) rawData[3] * 16);
    int16_t coefHumidity4_lsb = (int16_t) (rawData[4] & 0x0F);
    calibrationData->coefHumidity4 = (int16_t) (coefHumidity4_msb | coefHumidity4_lsb);

    int16_t coefHumidity5_msb = (int16_t) ((int8_t) rawData[5] * 16);
    int16_t coefHumidity5_lsb = (int16_t) (rawData[4] >> 4);
    calibrationData->coefHumidity5 = (int16_t) (coefHumidity5_msb | coefHumidity5_lsb);

    calibrationData->coefHumidity6 = (int8_t) rawData[6];
}

static bme280_error_code_t Bm280GetCalibrationData(bme280_device_t * const device)
{
    // TODO
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

            LOG_INFO("Chip ID = 0x%02X", chipId);

            if (initResult == BME280_OK && chipId == BME280_CHIP_ID)
            {

                initResult = Bme280SoftReset(device);

                if (initResult == BME280_OK)
                {
                    initResult = Bm280GetCalibrationData(device);
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
