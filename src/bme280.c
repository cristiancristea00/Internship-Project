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


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "bme280.h"

#include "config.h"
#include "vector.h"
#include "uart.h"
#include "i2c.h"

#include <util/delay.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Concatenate two bytes into a 16-bit value.
 **/
#define BME280_CONCAT_BYTES(MSB, LSB) (((uint16_t) (MSB) << 8) | (uint16_t) (LSB))

/**
 * @brief Sets the bits of a register.
 **/
#define BME280_SET_BITS(REGISTER_DATA, BITNAME, DATA) ((REGISTER_DATA & ~(BITNAME ## _MSK)) | ((DATA << BITNAME ## _POS) & BITNAME ## _MSK))

/**
 * @brief Gets the bits of a register.
 **/
#define BME280_GET_BITS(REGISTER_DATA, BITNAME) ((REGISTER_DATA & (BITNAME ## _MSK)) >> (BITNAME ## _POS))

// The BME280 chip identifier

#define BME280_CHIP_ID                     UINT8(0x60)

// Register addresses

#define BME280_CHIP_ID_ADDRESS             UINT8(0xD0)
#define BME280_RESET_ADDRESS               UINT8(0xE0)
#define BME280_TEMP_PRESS_CALIB_ADDRESS    UINT8(0x88)
#define BME280_HUMIDITY_CALIB_ADDRESS      UINT8(0xE1)
#define BME280_DATA_ADDRESS                UINT8(0xF7)
#define BME280_POWER_CONTROL_ADDRESS       UINT8(0xF4)
#define BME280_CONTROL_HUMIDITY_ADDRESS    UINT8(0xF2)
#define BME280_CONTROL_MEAS_ADDRESS        UINT8(0xF4)
#define BME280_CONFIG_ADDRESS              UINT8(0xF5)

// Values related to sizes

#define BME280_TEMP_PRESS_CALIB_LENGTH     UINT8(26)
#define BME280_HUMIDITY_CALIB_LENGTH       UINT8(7)
#define BME280_DATA_LENGTH                 UINT8(8)
#define BME280_CONFIG_REGISTERS_LENGTH     UINT8(4)

// Status info

#define BME280_STATUS_REGISTER_ADDRESS     UINT8(0xF3)
#define BME280_SOFT_RESET_COMMAND          UINT8(0xB6)
#define BME280_STATUS_UPDATE               UINT8(0x01)

// Value limits for measurements

#define BME280_MIN_TEMPERATURE             INT32(-4000)
#define BME280_MAX_TEMPERATURE             INT32(8500)
#define BME280_MIN_PRESSURE                UINT32(3000000)
#define BME280_MAX_PRESSURE                UINT32(11000000)
#define BME280_MAX_HUMIDITY                UINT32(102400)

// Bit masks and bit positions

#define BME280_CONTROL_TEMPERATURE_MSK     UINT8(0xE0)
#define BME280_CONTROL_TEMPERATURE_POS     UINT8(0x05)

#define BME280_CONTROL_PRESSURE_MSK        UINT8(0x1C)
#define BME280_CONTROL_PRESSURE_POS        UINT8(0x02)

#define BME280_CONTROL_HUMIDITY_MSK        UINT8(0x07)
#define BME280_CONTROL_HUMIDITY_POS        UINT8(0x00)

#define BME280_SENSOR_MODE_MSK             UINT8(0x03)
#define BME280_SENSOR_MODE_POS             UINT8(0x00)

#define BME280_FILTER_MSK                  UINT8(0x1C)
#define BME280_FILTER_POS                  UINT8(0x02)

#define BME280_STANDBY_MSK                 UINT8(0xE0)
#define BME280_STANDBY_POS                 UINT8(0x05)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef struct BME280_UNCOMPENSATED_DATA
{
    // Uncompensated data for the temperature sensor
    uint32_t temperature;

    // Uncompensated data for the pressure sensor
    uint32_t pressure;

    // Uncompensated data for the humidity sensor
    uint32_t humidity;
} bme280_uncompensated_data_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Checks if the device, its bus handler and I2C device are valid.
 *
 * @param[in] device BME280 device
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_CheckNull(bme280_device_t const * const device);

/**
 * @brief Reads a number of one byte registers from the device using I2C and
 *        stores the data in a buffer.
 *
 * @param[in]  i2c The I2C device to use
 * @param[in]  address The address of the device
 * @param[in]  registerAddress The address of the first register to read
 * @param[out] dataBuffer The buffer to store the data in
 * @param[in]  bufferLength The number of bytes to read
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_I2C_ReadRegisters(i2c_t const * const i2c, uint8_t const address, uint8_t const registerAddress, uint8_t * const dataBuffer, uint8_t const bufferLength);

/**
 * @brief Writes a number of one byte registers to the device using I2C from a
 *        buffer.
 *
 * @param[in] i2c The I2C device to use
 * @param[in] address The address of the device
 * @param[in] registerAddresses The addresses of the registers to write
 * @param[in] dataBuffer The buffer containing the data to write
 * @param[in] bufferLength The number of bytes to write
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_I2C_WriteRegisters(i2c_t const * const i2c, uint8_t const address, uint8_t const * const registerAddresses, uint8_t const * const dataBuffer, uint8_t const bufferLength);

/**
 * @brief Reads a number of one byte registers from the device and stores the
 *        data in a buffer.
 *
 * @param[in]  device BME280 device
 * @param[in]  registerAddress The address of the register to read
 * @param[out] dataBuffer The buffer to store the data in
 * @param[in]  bufferLength The number of bytes to read
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_GetRegisters(bme280_device_t const * const device, uint8_t const registerAddress, uint8_t * const dataBuffer, uint8_t const bufferLength);

/**
 * @brief Writes a number of one byte registers to the device from a buffer.
 *
 * @param[in] device BME280 device
 * @param[in] registerAddresses The addresses of the registers to write
 * @param[in] dataBuffer The buffer containing the data to write
 * @param[in] bufferLength The number of bytes to write
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_SetRegisters(bme280_device_t const * const device, uint8_t const * const registerAddresses, uint8_t const * const dataBuffer, uint8_t const bufferLength);

/**
 * @brief Soft resets the device to default settings and to sleep mode.
 *
 * @param[in] device BME280 device
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_SoftReset(bme280_device_t const * const device);

/**
 * @brief Parses the temperature and pressure calibration data from the device
 *        and stores it in the calibration data structure.
 *
 * @param[out] calibrationData The calibration data structure
 * @param[in]  rawData The raw calibration data
 **/
static void BME280_ParseTemperatureAndPressureCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData);

/**
 * @brief Parses the humidity calibration data from the device and stores it in
 *        the calibration data structure.
 *
 * @param[out] calibrationData The calibration data structure
 * @param[in]  rawData The raw calibration data
 **/
static void BME280_ParseHumidityCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData);

/**
 * @brief Reads the calibration data from the device.
 *
 * @param[in, out] device BME280 device
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_GetCalibrationData(bme280_device_t * const device);

/**
 * @brief Compensates the raw temperature data using the calibration parameters.
 *
 * @param[in] uncompensatedData The raw temperature data
 * @param[in] calibrationData The calibration data
 *
 * @return double The real temperature value
 **/
static int32_t BME280_CompensateTemperature(bme280_uncompensated_data_t const * const uncompensatedData, bme280_calibration_data_t * const calibrationData);

/**
 * @brief Compensates the raw pressure data using the calibration parameters.
 *
 * @param[in] uncompensatedData The raw pressure data
 * @param[in] calibrationData The calibration data
 *
 * @return double The real pressure value
 **/
static uint32_t BME280_CompensatePressure(bme280_uncompensated_data_t const * const uncompensatedData, bme280_calibration_data_t const * const calibrationData);

/**
 * @brief Compensates the raw humidity data using the calibration parameters.
 *
 * @param[in] uncompensatedData The raw humidity data
 * @param[in] calibrationData The calibration data
 *
 * @return double The real humidity value
 **/
static uint32_t BME280_CompensateHumidity(bme280_uncompensated_data_t const * const uncompensatedData, bme280_calibration_data_t const * const calibrationData);

/**
 * @brief Compensates the raw sensor data using the calibration parameters and
 *        stores the real sensor values in the sensor data structure.
 *
 * @param[in, out] device BME280 device
 * @param[in]      uncompensatedData The raw sensor data
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_CompensateData(bme280_device_t * const device, bme280_uncompensated_data_t const * const uncompensatedData);

/**
 * @brief Concatenates the register values to uncompensated data.
 *
 * @param[out] uncompensatedData The uncompensated data
 * @param[in]  data The register values
 **/
static void BME280_ParseSensorData(bme280_uncompensated_data_t * const uncompensatedData, uint8_t const * const data);

/**
 * @brief Sets the device temperature and pressure oversampling settings.
 *
 * @param[in, out] device BME280 device
 * @param[in]      settings The settings to set
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_SetOversamplingTemperaturePressure(bme280_device_t * const device, bme280_settings_t const * const settings);

/**
 * @brief Sets the device humidity oversampling settings.
 *
 * @param[in, out] device BME280 device
 * @param[in]      settings The settings to set
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_SetOversamplingHumidity(bme280_device_t * const device, bme280_settings_t const * const settings);

/**
 * @brief Sets the device oversampling settings.
 *
 * @param[in, out] device BME280 device
 * @param[in]      settings The settings to set
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_SetOversamplingSettings(bme280_device_t * const device, bme280_settings_t const * const settings);

/**
 * @brief Sets the device filter and standby settings.
 *
 * @param[in, out] device BME280 device
 * @param[in]      settings The settings to set
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_SetFilterStandbySettings(bme280_device_t * const device, bme280_settings_t const * const settings);

/**
 * @brief Writes the power mode to the device.
 *
 * @param[in, out] device BME280 device
 * @param[in]      powerMode The power mode to set
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_WritePowerMode(bme280_device_t * const device, bme280_power_mode_t const powerMode);

/**
 * @brief Sets the device power mode.
 *
 * @param[in, out] device BME280 device
 * @param[in]      settings The settings to set
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_SetSensorPowerMode(bme280_device_t * const device, bme280_settings_t const * const settings);

/**
 * @brief Sets the device settings.
 *
 * @param[in, out] device BME280 device
 * @param[in]      settings The settings to set
 *
 * @return bme280_error_code_t Error code
 **/
static bme280_error_code_t BME280_SetSensorSettings(bme280_device_t * const device, bme280_settings_t const * const settings);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

static bme280_error_code_t BME280_CheckNull(bme280_device_t const * const device)
{
    if (device == NULL || device->handler == NULL || device->i2cDevice == NULL)
    {
        LOG_ERROR("Found NULL pointer on BME280");
        return BME280_NULL_POINTER;
    }
    else
    {
        return BME280_OK;
    }
}

static bme280_error_code_t BME280_I2C_ReadRegisters(i2c_t const * const i2c, uint8_t const address, uint8_t const registerAddress, uint8_t * const dataBuffer, uint8_t const bufferLength)
{
    bme280_error_code_t readResult = BME280_OK;

    if (i2c->SendData(address, &registerAddress, 1) != I2C_OK)
    {
        readResult = BME280_COMMUNICATION_ERROR;
    }

    if (i2c->ReceiveData(address, dataBuffer, bufferLength) != I2C_OK)
    {
        readResult = BME280_COMMUNICATION_ERROR;
    }

    i2c->EndTransaction();

    return readResult;
}

static bme280_error_code_t BME280_I2C_WriteRegisters(i2c_t const * const i2c, uint8_t const address, uint8_t const * const registerAddresses, uint8_t const * const dataBuffer, uint8_t const bufferLength)
{
    bme280_error_code_t writeResult = BME280_OK;

    uint8_t const interleavdBufferLength = 2 * bufferLength;
    uint8_t interleavedDataBuffer[interleavdBufferLength];

    for (uint8_t registerIdx = 0; registerIdx < bufferLength; ++registerIdx)
    {
        interleavedDataBuffer[2 * registerIdx] = registerAddresses[registerIdx];
        interleavedDataBuffer[2 * registerIdx + 1] = dataBuffer[registerIdx];
    }

    if (i2c->SendData(address, &interleavedDataBuffer, interleavdBufferLength) != I2C_OK)
    {
        writeResult = BME280_COMMUNICATION_ERROR;
    }

    i2c->EndTransaction();

    return writeResult;
}

static bme280_error_code_t BME280_GetRegisters(bme280_device_t const * const device, uint8_t const registerAddress, uint8_t * const dataBuffer, uint8_t const bufferLength)
{
    bme280_error_code_t getResult = BME280_OK;

    getResult = BME280_CheckNull(device);

    if (getResult == BME280_OK && dataBuffer != NULL)
    {
        if (device->handler->I2C_Read(device->i2cDevice, device->i2cAddress, registerAddress, dataBuffer, bufferLength) != BME280_OK)
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

static bme280_error_code_t BME280_SetRegisters(bme280_device_t const * const device, uint8_t const * const registerAddresses, uint8_t const * const dataBuffer, uint8_t const bufferLength)
{
    bme280_error_code_t setResult = BME280_OK;

    setResult = BME280_CheckNull(device);

    if (setResult == BME280_OK && dataBuffer != NULL && registerAddresses != NULL)
    {
        if (bufferLength != 0)
        {
            if (device->handler->I2C_Write(device->i2cDevice, device->i2cAddress, registerAddresses, dataBuffer, bufferLength) != BME280_OK)
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

static bme280_error_code_t BME280_SoftReset(bme280_device_t const * const device)
{
    bme280_error_code_t resetResult = BME280_OK;

    resetResult = BME280_CheckNull(device);

    LOG_INFO("Started BME280 soft reset");

    if (resetResult == BME280_OK)
    {
        uint8_t const registerAdress = BME280_RESET_ADDRESS;
        uint8_t const resetCommand = BME280_SOFT_RESET_COMMAND;

        resetResult = BME280_SetRegisters(device, &registerAdress, &resetCommand, 1);

        if (resetResult == BME280_OK)
        {
            uint8_t tryCount = 10;
            uint8_t statusRegister = 0;

            LOG_INFO("Finished BME280 soft reset successfully");

            do
            {
                LOG_INFO("Waiting for the BME280 NVM copying...");
                _delay_ms(2);
                resetResult = BME280_GetRegisters(device, BME280_STATUS_REGISTER_ADDRESS, &statusRegister, 1);
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

static void BME280_ParseTemperatureAndPressureCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData)
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

    return;
}

static void BME280_ParseHumidityCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData)
{
    // Humidity

    calibrationData->humidityCoef2 = (int16_t) BME280_CONCAT_BYTES(rawData[1], rawData[0]);
    calibrationData->humidityCoef3 = (uint8_t) rawData[2];

    int16_t coefHumidity4_MSB      = (int16_t) ((int8_t) rawData[3] << 4);
    int16_t coefHumidity4_LSB      = (int16_t) (rawData[4] & 0x0F);
    calibrationData->humidityCoef4 = (int16_t) (coefHumidity4_MSB | coefHumidity4_LSB);

    int16_t coefHumidity5_MSB      = (int16_t) ((int8_t) rawData[5] << 4);
    int16_t coefHumidity5_LSB      = (int16_t) (rawData[4] >> 4);
    calibrationData->humidityCoef5 = (int16_t) (coefHumidity5_MSB | coefHumidity5_LSB);

    calibrationData->humidityCoef6 = (int8_t) rawData[6];

    return;
}

static bme280_error_code_t BME280_GetCalibrationData(bme280_device_t * const device)
{
    bme280_error_code_t calibrationResult = BME280_OK;

    uint8_t calibrationData[BME280_TEMP_PRESS_CALIB_LENGTH] = { 0 };

    calibrationResult = BME280_GetRegisters(device, BME280_TEMP_PRESS_CALIB_ADDRESS, calibrationData, BME280_TEMP_PRESS_CALIB_LENGTH);

    if (calibrationResult == BME280_OK)
    {
        BME280_ParseTemperatureAndPressureCalibration(&device->calibrationData, calibrationData);

        calibrationResult = BME280_GetRegisters(device, BME280_HUMIDITY_CALIB_ADDRESS, calibrationData, BME280_HUMIDITY_CALIB_LENGTH);

        if (calibrationResult == BME280_OK)
        {
            BME280_ParseHumidityCalibration(&device->calibrationData, calibrationData);
        }
    }

    if (calibrationResult == BME280_OK)
    {
        LOG_INFO("Got BME280 calibration data from sensor");
    }
    else
    {
        LOG_ERROR("Failed to get BME280 calibration data");
    }

    return calibrationResult;
}

static int32_t BME280_CompensateTemperature(bme280_uncompensated_data_t const * const uncompensatedData, bme280_calibration_data_t * const calibrationData)
{
    int32_t temperature = 0;

    int32_t temp1 = 0;
    int32_t temp2 = 0;

    temp1 = (int32_t) ((uncompensatedData->temperature >> 3) - ((int32_t) calibrationData->temperatureCoef1 << 1));
    temp1 = (temp1 * ((int32_t) calibrationData->temperatureCoef2)) >> 11;
    temp2 = (int32_t) ((uncompensatedData->temperature >> 4) - ((int32_t) calibrationData->temperatureCoef1));
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

static uint32_t BME280_CompensatePressure(bme280_uncompensated_data_t const * const uncompensatedData, bme280_calibration_data_t const * const calibrationData)
{
    uint32_t pressure = 0;

    int64_t temp1 = 0;
    int64_t temp2 = 0;
    int64_t temp3 = 0;

    temp1 = ((int64_t) calibrationData->temperatureTemporary) - 128000;
    temp2 = temp1 * temp1 * (int64_t) calibrationData->pressureCoef6;
    temp2 = temp2 + ((temp1 * (int64_t) calibrationData->pressureCoef5) << 17);
    temp2 = temp2 + (((int64_t) calibrationData->pressureCoef4) << 35);
    temp1 = ((temp1 * temp1 * (int64_t) calibrationData->pressureCoef3) >> 8) + (temp1 * ((int64_t) calibrationData->pressureCoef2) << 12);
    temp1 = ((((int64_t) 1) << 47) + temp1) * ((int64_t) calibrationData->pressureCoef1) >> 33;

    if (temp1 != 0)
    {
        temp3 = ((int64_t) 1 << 20) - uncompensatedData->pressure;
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

static uint32_t BME280_CompensateHumidity(bme280_uncompensated_data_t const * const uncompensatedData, bme280_calibration_data_t const * const calibrationData)
{
    uint32_t humidity = 0;

    int32_t temp1 = 0;
    int32_t temp2 = 0;
    int32_t temp3 = 0;
    int32_t temp4 = 0;
    int32_t temp5 = 0;

    temp1 = calibrationData->temperatureTemporary - ((int32_t) 76800);
    temp2 = (int32_t) (uncompensatedData->humidity << 14);
    temp3 = (int32_t) (((int32_t) calibrationData->humidityCoef4) << 20);
    temp4 = ((int32_t) calibrationData->humidityCoef5) * temp1;
    temp5 = ((temp2 - temp3 - temp4) + ((int32_t) 1 << 14)) >> 15;
    temp2 = (temp1 * ((int32_t) calibrationData->humidityCoef6)) >> 10;
    temp3 = (temp1 * ((int32_t) calibrationData->humidityCoef3)) >> 11;
    temp4 = ((temp2 * (temp3 + ((int32_t) 1 << 15))) >> 10) + ((int32_t) 1 << 21);
    temp2 = ((temp4 * ((int32_t) calibrationData->humidityCoef2)) + ((int32_t) 1 << 13)) >> 14;
    temp3 = temp5 * temp2;
    temp4 = ((temp3 >> 15) * (temp3 >> 15)) >> 7;
    temp5 = temp3 - ((temp4 * ((int32_t) calibrationData->humidityCoef1)) >> 4);
    temp5 = temp5 < 0 ? 0 : temp5;
    temp5 = temp5 > 419430400 ? 419430400 : temp5;
    humidity = (uint32_t) (temp5 >> 12);

    if (humidity > BME280_MAX_HUMIDITY)
    {
        humidity = BME280_MAX_HUMIDITY;
        LOG_WARNING("BME280 read humidity is higher than the expected maximum");
    }

    return humidity;
}

static bme280_error_code_t BME280_CompensateData(bme280_device_t * const device, bme280_uncompensated_data_t const * const uncompensatedData)
{
    bme280_error_code_t compensationResult = BME280_OK;

    compensationResult = BME280_CheckNull(device);

    if (uncompensatedData != NULL)
    {
        device->data.temperature = BME280_CompensateTemperature(uncompensatedData, &device->calibrationData);
        device->data.pressure = BME280_CompensatePressure(uncompensatedData, &device->calibrationData);
        device->data.humidity = BME280_CompensateHumidity(uncompensatedData, &device->calibrationData);

        LOG_INFO("Compensated BME280 data successfully");
    }
    else
    {
        compensationResult = BME280_NULL_POINTER;
        LOG_WARNING("Failed to compensate BME280 data");
    }

    return compensationResult;
}

static void BME280_ParseSensorData(bme280_uncompensated_data_t * const uncompensatedData, uint8_t const * const data)
{
    // Store the parsed register values for temperature data
    uint32_t temperatureMSB  = (uint32_t) data[3] << 12;
    uint32_t temperatureLSB  = (uint32_t) data[4] << 4;
    uint32_t temperatureXLSB = (uint32_t) data[5] >> 4;

    uncompensatedData->temperature = temperatureMSB | temperatureLSB | temperatureXLSB;

    // Store the parsed register values for pressure data
    uint32_t pressureMSB  = (uint32_t) data[0] << 12;
    uint32_t pressureLSB  = (uint32_t) data[1] << 4;
    uint32_t pressureXLSB = (uint32_t) data[2] >> 4;

    uncompensatedData->pressure = pressureMSB | pressureLSB | pressureXLSB;

    // Store the parsed register values for humidity data
    uint32_t humidityMSB = (uint32_t) data[6] << 8;
    uint32_t humidityLSB = (uint32_t) data[7];

    uncompensatedData->humidity = humidityMSB | humidityLSB;

    return;
}

static bme280_error_code_t BME280_SetOversamplingTemperaturePressure(bme280_device_t * const device, bme280_settings_t const * const settings)
{
    bme280_error_code_t oversamplingResult = BME280_OK;

    uint8_t measAddress = BME280_CONTROL_MEAS_ADDRESS;

    uint8_t registerData = 0;

    oversamplingResult = BME280_GetRegisters(device, measAddress, &registerData, 1);

    if (oversamplingResult == BME280_OK)
    {
        registerData = BME280_SET_BITS(registerData, BME280_CONTROL_TEMPERATURE, settings->temperatureOversampling);
        registerData = BME280_SET_BITS(registerData, BME280_CONTROL_PRESSURE, settings->pressureOversampling);

        oversamplingResult = BME280_SetRegisters(device, &measAddress, &registerData, 1);
    }

    if (oversamplingResult == BME280_OK)
    {
        LOG_INFO("Set oversampling settings for temperature and pressure successfully");
    }
    else
    {
        LOG_ERROR("Failed to set oversampling settings for temperature and pressure");
    }

    return oversamplingResult;
}

static bme280_error_code_t BME280_SetOversamplingHumidity(bme280_device_t * const device, bme280_settings_t const * const settings)
{
    bme280_error_code_t oversamplingResult = BME280_OK;

    uint8_t humidityAddress = BME280_CONTROL_HUMIDITY_ADDRESS;

    uint8_t registerData = settings->humidityOversampling & BME280_CONTROL_HUMIDITY_MSK;

    oversamplingResult = BME280_SetRegisters(device, &humidityAddress, &registerData, 1);

    if (oversamplingResult == BME280_OK)
    {
        LOG_INFO("Set oversampling settings for humidity successfully");
    }
    else
    {
        LOG_ERROR("Failed to set oversampling settings for humidity");
    }

    return oversamplingResult;
}

static bme280_error_code_t BME280_SetOversamplingSettings(bme280_device_t * const device, bme280_settings_t const * const settings)
{
    bme280_error_code_t oversamplingResult = BME280_OK;

    oversamplingResult = BME280_SetOversamplingHumidity(device, settings);

    oversamplingResult = BME280_SetOversamplingTemperaturePressure(device, settings);

    return oversamplingResult;
}

static bme280_error_code_t BME280_SetFilterStandbySettings(bme280_device_t * const device, bme280_settings_t const * const settings)
{
    bme280_error_code_t standbyFilterResult = BME280_OK;

    uint8_t configAddress = BME280_CONFIG_ADDRESS;

    uint8_t registerData = 0;

    standbyFilterResult = BME280_GetRegisters(device, configAddress, &registerData, 1);

    if (standbyFilterResult == BME280_OK)
    {
        registerData = BME280_SET_BITS(registerData, BME280_FILTER, settings->iirFilterCoefficients);
        registerData = BME280_SET_BITS(registerData, BME280_STANDBY, settings->standbyTime);

        standbyFilterResult = BME280_SetRegisters(device, &configAddress, &registerData, 1);
    }

    if (standbyFilterResult == BME280_OK)
    {
        LOG_INFO("Set filter and standby settings successfully");
    }
    else
    {
        LOG_ERROR("Failed to set filter and standby settings");
    }

    return standbyFilterResult;
}

static bme280_error_code_t BME280_WritePowerMode(bme280_device_t * const device, bme280_power_mode_t const powerMode)
{
    bme280_error_code_t powerResult = BME280_OK;

    uint8_t powerControlAddress = BME280_POWER_CONTROL_ADDRESS;

    uint8_t sensorModeValue = 0;

    powerResult = BME280_GetRegisters(device, powerControlAddress, &sensorModeValue, 1);

    if (powerResult == BME280_OK)
    {
        sensorModeValue = BME280_SET_BITS(sensorModeValue, BME280_SENSOR_MODE, powerMode);

        powerResult = BME280_SetRegisters(device, &powerControlAddress, &sensorModeValue, 1);
    }

    return powerResult;
}

static bme280_error_code_t BME280_SetSensorPowerMode(bme280_device_t * const device, bme280_settings_t const * const settings)
{
    bme280_error_code_t modeResult = BME280_OK;

    modeResult = BME280_CheckNull(device);

    if (modeResult == BME280_OK)
    {
        modeResult = BME280_WritePowerMode(device, settings->powerMode);
    }

    if (modeResult == BME280_OK)
    {
        LOG_INFO("Set power mode settings successfully");
    }
    else
    {
        LOG_ERROR("Failed to set power mode settings");
    }

    return modeResult;
}

static bme280_error_code_t BME280_SetSensorSettings(bme280_device_t * const device, bme280_settings_t const * const settings)
{
    bme280_error_code_t settingsResult = BME280_OK;

    settingsResult = BME280_CheckNull(device);

    if (settingsResult == BME280_OK)
    {
        if (settingsResult == BME280_OK)
        {
            settingsResult = BME280_SetOversamplingSettings(device, settings);
        }

        if (settingsResult == BME280_OK)
        {
            settingsResult = BME280_SetFilterStandbySettings(device, settings);
        }

        if (settingsResult == BME280_OK)
        {
            settingsResult = BME280_SetSensorPowerMode(device, settings);
        }
    }

    if (settingsResult == BME280_OK)
    {
        device->settings = *settings;
        LOG_INFO("Set BME280 settings successfully");
    }
    else
    {
        LOG_ERROR("Failed to set BME280 settings");
    }

    return settingsResult;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

bme280_error_code_t BME280_Init(bme280_device_t * const device, bme280_handler_t const * const handler, i2c_t const * const i2cDevice, uint8_t const i2cAddress, bme280_settings_t const * const settings)
{
    device->i2cDevice = i2cDevice;
    device->i2cAddress = i2cAddress;
    device->handler = handler;

    bme280_error_code_t initResult = BME280_OK;

    LOG_INFO("Started BME280 initialization");

    initResult = BME280_CheckNull(device);

    if (initResult == BME280_OK)
    {
        uint8_t tryCount = 10;
        uint8_t chipId = 0;

        while (tryCount != 0)
        {
            initResult = BME280_GetRegisters(device, BME280_CHIP_ID_ADDRESS, &chipId, 1);

            if (initResult == BME280_OK && chipId == BME280_CHIP_ID)
            {
                LOG_INFO("Found BME280 with chip ID = 0x60");

                initResult = BME280_SoftReset(device);

                if (initResult == BME280_OK)
                {
                    initResult = BME280_GetCalibrationData(device);
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
        initResult = BME280_SetSensorSettings(device, settings);
    }

    if (initResult == BME280_OK)
    {
        LOG_INFO("Finished the BME280 initialization");
    }
    else
    {
        LOG_ERROR("Couldn't finish the BME280 initialization");
    }

    _delay_ms(100);

    return initResult;
}

bme280_error_code_t BME280_GetSensorData(bme280_device_t * const device)
{
    bme280_error_code_t acquisitionResult = BME280_OK;

    acquisitionResult = BME280_CheckNull(device);

    if (acquisitionResult == BME280_OK)
    {
        uint8_t data[BME280_DATA_LENGTH] = { 0 };
        bme280_uncompensated_data_t uncompensatedData = { 0 };

        acquisitionResult = BME280_GetRegisters(device, BME280_DATA_ADDRESS, data, BME280_DATA_LENGTH);

        if (acquisitionResult == BME280_OK)
        {
            BME280_ParseSensorData(&uncompensatedData, data);

            LOG_INFO("Acquired raw sensor data from BME280");

            acquisitionResult = BME280_CompensateData(device, &uncompensatedData);
        }
    }

    if (acquisitionResult == BME280_OK)
    {
        LOG_INFO("Sensor acquisition from BME280 finished successfully");
    }
    else
    {
        LOG_WARNING("Sensor acquisition from BME280 failed");
    }

    return acquisitionResult;
}

int32_t BME280_GetTemperature(bme280_data_t const * const deviceData)
{
    return deviceData->temperature;
}

uint32_t BME280_GetPressure(bme280_data_t const * const deviceData)
{
    return deviceData->pressure;
}

uint32_t BME280_GetHuimidity(bme280_data_t const * const deviceData)
{
    return deviceData->humidity;
}

double BME280_GetDisplayTemperature(bme280_data_t const * const deviceData)
{
    return 0.01f * (double) BME280_GetTemperature(deviceData);
}

double BME280_GetDisplayPressure(bme280_data_t const * const deviceData)
{
    return 0.0001f * (double) BME280_GetPressure(deviceData);
}

double BME280_GetDisplayHumidity(bme280_data_t const * const deviceData)
{
    return (1.0f / 1024.0f) * (double) BME280_GetHuimidity(deviceData);
}

void BME280_StructInterpret(void * const data, vector_t * const vector)
{
    bme280_data_t * const sensorData = (bme280_data_t *) data;

    // Remove the checksum byte
    Vector_RemoveByte(vector);

    sensorData->temperature = (int32_t) Vector_RemoveDoubleWord(vector);
    sensorData->pressure    = (uint32_t) Vector_RemoveDoubleWord(vector);
    sensorData->humidity    = (uint32_t) Vector_RemoveDoubleWord(vector);

    return;
}

void BME280_SerializeSensorData(bme280_device_t const * const device, uint8_t * const buffer)
{
    vector_t serializedBuffer;
    Vector_Initialize(&serializedBuffer);

    Vector_AddDoubleWord(&serializedBuffer, (uint32_t) BME280_GetHuimidity(&device->data));
    Vector_AddDoubleWord(&serializedBuffer, (uint32_t) BME280_GetPressure(&device->data));
    Vector_AddDoubleWord(&serializedBuffer, (uint32_t) BME280_GetTemperature(&device->data));

    memcpy(buffer, serializedBuffer.internalBuffer, BME280_SERIALIZED_SIZE);

    return;
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

bme280_handler_t const BME280_I2C0_Handler = {
    .I2C_Read = BME280_I2C_ReadRegisters,
    .I2C_Write = BME280_I2C_WriteRegisters,
};
