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

    uint8_t const bufferLength = 2 * length;
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

    LOG_INFO("Started BME280 soft reset");

    if (resetResult == BME280_OK)
    {
        uint8_t const registerAdress = BME280_RESET_ADDRESS;
        uint8_t const resetCommand = BME280_SOFT_RESET_COMMAND;

        resetResult = Bme280SetRegisters(device, &registerAdress, &resetCommand, 1);

        if (resetResult == BME280_OK)
        {
            uint8_t tryCount = 10;
            uint8_t statusRegister = 0;

            LOG_INFO("Finished BME280 soft reset successfully");

            do
            {
                LOG_INFO("Waiting for the BME280 NVM copying...");
                _delay_ms(2);
                resetResult = Bme280GetRegisters(device, BME280_STATUS_REGISTER_ADDRESS, &statusRegister, 1);
                --tryCount;
            }
            while ((resetResult == BME280_OK) && (tryCount != 0) && (statusRegister & BME280_STATUS_UPDATE));

            if (statusRegister & BME280_STATUS_UPDATE)
            {
                resetResult = BME280_NVM_COPY_FAILED;

                LOG_ERROR("The BME280 NVM copy failed");
            }
        }
    }

    return resetResult;
}

static void Bme280ParseTemperatureAndPressureCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData)
{
    // Temperature

    calibrationData->temperatureCoef1 = (uint16_t) BME280_CONCAT_BYTES(rawData[1], rawData[0]);
    calibrationData->temperatureCoef2 = (int16_t) BME280_CONCAT_BYTES(rawData[3], rawData[2]);
    calibrationData->temperatureCoef3 = (int16_t) BME280_CONCAT_BYTES(rawData[5], rawData[4]);

    //Pressure

    calibrationData->pressureCoef1 = (uint16_t) BME280_CONCAT_BYTES(rawData[7], rawData[6]);
    calibrationData->pressureCoef2 = (int16_t) BME280_CONCAT_BYTES(rawData[9], rawData[8]);
    calibrationData->pressureCoef3 = (int16_t) BME280_CONCAT_BYTES(rawData[11], rawData[10]);
    calibrationData->pressureCoef4 = (int16_t) BME280_CONCAT_BYTES(rawData[13], rawData[12]);
    calibrationData->pressureCoef5 = (int16_t) BME280_CONCAT_BYTES(rawData[15], rawData[14]);
    calibrationData->pressureCoef6 = (int16_t) BME280_CONCAT_BYTES(rawData[17], rawData[16]);
    calibrationData->pressureCoef7 = (int16_t) BME280_CONCAT_BYTES(rawData[19], rawData[18]);
    calibrationData->pressureCoef8 = (int16_t) BME280_CONCAT_BYTES(rawData[21], rawData[20]);
    calibrationData->pressureCoef9 = (int16_t) BME280_CONCAT_BYTES(rawData[23], rawData[22]);

    // One lonely humidity coefficient

    calibrationData->humidityCoef1 = (uint8_t) rawData[25];
}

static void Bme280ParseHumidityCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData)
{
    // Humidity

    calibrationData->humidityCoef2 = (int16_t) BME280_CONCAT_BYTES(rawData[1], rawData[0]);
    calibrationData->humidityCoef3 = (uint8_t) rawData[2];

    int16_t coefHumidity4_msb = (int16_t) ((int8_t) rawData[3] << 4);
    int16_t coefHumidity4_lsb = (int16_t) (rawData[4] & 0x0F);
    calibrationData->humidityCoef4 = (int16_t) (coefHumidity4_msb | coefHumidity4_lsb);

    int16_t coefHumidity5_msb = (int16_t) ((int8_t) rawData[5] << 4);
    int16_t coefHumidity5_lsb = (int16_t) (rawData[4] >> 4);
    calibrationData->humidityCoef5 = (int16_t) (coefHumidity5_msb | coefHumidity5_lsb);

    calibrationData->humidityCoef6 = (int8_t) rawData[6];
}

static bme280_error_code_t Bm280GetCalibrationData(bme280_device_t * const device)
{
    bme280_error_code_t calibrationResult = BME280_OK;

    uint8_t calibrationData[BME280_TEMP_PRESS_CALIB_LENGTH] = { 0 };

    calibrationResult = Bme280GetRegisters(device, BME280_TEMP_PRESS_CALIB_ADDRESS, calibrationData, BME280_TEMP_PRESS_CALIB_LENGTH);

    if (calibrationResult == BME280_OK)
    {
        Bme280ParseTemperatureAndPressureCalibration(&device->calibrationData, calibrationData);

        calibrationResult = Bme280GetRegisters(device, BME280_HUMIDITY_CALIB_ADDRESS, calibrationData, BME280_HUMIDITY_CALIB_LENGTH);

        if (calibrationResult == BME280_OK)
        {
            Bme280ParseHumidityCalibration(&device->calibrationData, calibrationData);
        }
    }

    return calibrationResult;
}

static int32_t Bme280CompensateTemperature(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData)
{
    int32_t temperature = 0;

    int32_t temp1 = ((int32_t) ((uncompensatedData->temperature >> 3)) - ((int32_t) calibrationData->temperatureCoef1 << 1));
    temp1 = (temp1 * ((int32_t) calibrationData->temperatureCoef2)) >> 11;
    int32_t temp2 = (int32_t) ((uncompensatedData->temperature >> 4) - ((int32_t) calibrationData->temperatureCoef1));
    temp2 = (((temp2 * temp2) >> 12) * ((int32_t) calibrationData->temperatureCoef3)) >> 14;
    calibrationData->temperatureTemporary = temp1 + temp2;
    temperature = (calibrationData->temperatureTemporary * 5 + 128) >> 8;

    if (temperature < BME280_MIN_TEMPERATURE)
    {
        temperature = BME280_MIN_TEMPERATURE;
        LOG_WARNING("BME280 read temperature is lower than the expected minimum");
    }
    else if (temperature > BME280_MAX_TEMPERATURE)
    {
        temperature = BME280_MAX_TEMPERATURE;
        LOG_WARNING("BME280 read temperature is higher than the expected maximum");
    }

    return temperature;
}

static uint32_t Bme280CompensatePressure(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData)
{
    uint32_t pressure = 0;

    int64_t temp1 = ((int64_t) calibrationData->temperatureTemporary) - 128000;
    int64_t temp2 = temp1 * temp1 * (int64_t) calibrationData->pressureCoef6;
    temp2 = temp2 + ((temp1 * (int64_t) calibrationData->pressureCoef5) << 17);
    temp2 = temp2 + (((int64_t) calibrationData->pressureCoef4) << 35);
    temp1 = ((temp1 * temp1 * (int64_t) calibrationData->pressureCoef3) >> 8) + (temp1 * ((int64_t) calibrationData->pressureCoef2) << 12);
    temp1 = ((((int64_t) 1) << 47) + temp1) * ((int64_t) calibrationData->pressureCoef1) >> 33;

    if (temp1 != 0)
    {
        int64_t temp3 = ((int64_t) 1 << 20) - uncompensatedData->pressure;
        temp3 = (((temp3 * ((int64_t) 1 << 31)) - temp2) * 3125) / temp1;
        temp1 = (((int64_t) calibrationData->pressureCoef9) * (temp3 >> 13) * (temp3 >> 13)) >> 25;
        temp2 = (((int64_t) calibrationData->pressureCoef8) * temp3) >> 19;
        temp3 = ((temp3 + temp1 + temp2) >> 8) + (((int64_t) calibrationData->pressureCoef7) << 4);
        pressure = (uint32_t) (((temp3 >> 1) * 100) >> 7);

        if (pressure < BME280_MIN_PRESSURE)
        {
            pressure = BME280_MIN_PRESSURE;
            LOG_WARNING("BME280 read pressure is lower than the expected minimum");
        }
        else if (pressure > BME280_MAX_PRESSURE)
        {
            pressure = BME280_MAX_PRESSURE;
            LOG_WARNING("BME280 read pressure is higher than the expected maximum");
        }
    }
    else
    {
        pressure = BME280_MIN_PRESSURE;
        LOG_WARNING("BME280 read pressure is lower than the expected minimum");
    }

    return pressure;
}

static uint32_t Bme280CompensateHumidity(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData)
{
    uint32_t humidity = 0;

    int32_t temp1 = calibrationData->temperatureTemporary - ((int32_t) 76800);
    int32_t temp2 = (int32_t) (uncompensatedData->humidity << 14);
    int32_t temp3 = (int32_t) (((int32_t) calibrationData->humidityCoef4) << 20);
    int32_t temp4 = ((int32_t) calibrationData->humidityCoef5) * temp1;
    int32_t temp5 = (((temp2 - temp3) - temp4) + ((int32_t) 1 << 14)) << 15;
    temp2 = (temp1 * ((int32_t) calibrationData->humidityCoef6)) >> 10;
    temp3 = (temp1 * ((int32_t) calibrationData->humidityCoef3)) >> 11;
    temp4 = ((temp2 * (temp3 + ((int32_t) 1 << 15))) >> 10) + ((int32_t) 1 << 21);
    temp2 = ((temp4 * ((int32_t) calibrationData->humidityCoef2)) + ((int32_t) 1 << 13)) >> 14;
    temp3 = temp5 * temp2;
    temp4 = ((temp3 >> 15) * (temp3 >> 15)) >> 7;
    temp5 = temp3 - ((temp4 * ((int32_t) calibrationData->humidityCoef1)) >> 4);
    temp5 = temp5 > 0 ? 0 : temp5;
    temp5 = temp5 > 419430400 ? 419430400 : temp5;
    humidity = (uint32_t) (temp5 >> 12);

    if (humidity > BME280_MAX_HUMIDITY)
    {
        humidity = BME280_MAX_HUMIDITY;
        LOG_WARNING("BME280 read humidity is higher than the expected maximum");
    }

    return humidity;
}

bme280_error_code_t Bme280Init(bme280_device_t * const device, bme280_handler_t const * const handler, i2c_t const * const i2cDevice, uint8_t const i2cAddress)
{
    device->i2cDevice = NULL;
    device->i2cAddress = i2cAddress;
    device->handler = handler;

    bme280_error_code_t initResult = BME280_OK;

    uint8_t tryCount = 10;
    uint8_t chipId = 0;

    LOG_INFO("Started BME280 initialization");

    initResult = Bme280CheckNull(device);

    if (initResult == BME280_OK)
    {
        device->i2cDevice = i2cDevice;

        while (tryCount != 0)
        {
            initResult = Bme280GetRegisters(device, BME280_CHIP_ID_ADDRESS, &chipId, 1);

            if (initResult == BME280_OK && chipId == BME280_CHIP_ID)
            {
                LOG_INFO("Found BME280 with chip ID = 0x60");

                initResult = Bme280SoftReset(device);

                if (initResult == BME280_OK)
                {
                    initResult = Bm280GetCalibrationData(device);
                }

                break;
            }

            LOG_WARNING("Failed to find BME280. Trying again...");

            --tryCount;
            _delay_ms(1);
        }

        if (tryCount == 0)
        {
            initResult = BME280_DEVICE_NOT_FOUND;
        }
    }

    if (initResult == BME280_OK)
    {
        LOG_INFO("Got BME280 calibration data from sensor");
        LOG_INFO("Finished the BME280 initialization");
    }
    else
    {
        LOG_ERROR("Couldn't finish the BME280 initialization");
    }

    return initResult;
}
