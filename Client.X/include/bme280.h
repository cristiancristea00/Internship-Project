/**
 *  @file bme280.h
 *  @author Cristian Cristea - M70957
 *  @date July 25, 2022
 *
 *  @brief Header file for the BME280 module
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


#ifndef BME280_H
#define	BME280_H

#include "config.h"
#include "uart.h"
#include "i2c.h"

#include <util/delay.h>

#include <stddef.h>

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

// Integer macros

#define UINT8(X)                           ((uint8_t) (X))
#define UINT32(X)                          ((uint32_t) (X))
#define INT32(X)                           ((int32_t) (X))

// Principal and secondary I2C addresses of the chip

#define BME280_I2C_ADDRESS                 UINT8(0x76)
#define BME280_I2C_ADDRESS_SEC             UINT8(0x77)

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

typedef enum BME280_ERROR_CODE
{
    BME280_OK                  = 0x00,
    BME280_DEVICE_NOT_FOUND    = 0x01,
    BME280_NULL_POINTER        = 0x02,
    BME280_COMMUNICATION_ERROR = 0x03,
    BME280_INVALID_LENGTH      = 0x04,
    BME280_NVM_COPY_FAILED     = 0x05
} bme280_error_code_t;

typedef struct BME280_CALIBRATION_DATA
{
    // Calibration coefficients for the temperature sensor

    uint16_t temperatureCoef1;
    int16_t temperatureCoef2;
    int16_t temperatureCoef3;

    int32_t temperatureTemporary;

    // Calibration coefficients for the pressure sensor

    uint16_t pressureCoef1;
    int16_t pressureCoef2;
    int16_t pressureCoef3;
    int16_t pressureCoef4;
    int16_t pressureCoef5;
    int16_t pressureCoef6;
    int16_t pressureCoef7;
    int16_t pressureCoef8;
    int16_t pressureCoef9;

    // Calibration coefficients for the humidity sensor

    uint8_t humidityCoef1;
    int16_t humidityCoef2;
    uint8_t humidityCoef3;
    int16_t humidityCoef4;
    int16_t humidityCoef5;
    int8_t humidityCoef6;
} bme280_calibration_data_t;


typedef struct BME280_UNCOMPENSATED_DATA
{
    // Uncompensated data for the temperature sensor
    uint32_t temperature;

    // Uncompensated data for the pressure sensor
    uint32_t pressure;

    // Uncompensated data for the humidity sensor
    uint32_t humidity;
} bme280_uncompensated_data_t;


typedef struct BME280_DATA
{
    // Compensated data for the temperature sensor
    int32_t temperature;

    // Compensated data for the pressure sensor
    uint32_t  pressure;

    // Compensated data for the humidity sensor
    uint32_t humidity;
} bme280_data_t;

typedef enum BME280_OVERSAMPLING
{
    BME280_NO_OVERSAMPLING  = 0x00,
    BME280_OVERSAMPLING_1X  = 0x01,
    BME280_OVERSAMPLING_2X  = 0x02,
    BME280_OVERSAMPLING_4X  = 0x03,
    BME280_OVERSAMPLING_8X  = 0x04,
    BME280_OVERSAMPLING_16X = 0x05
} bme280_oversampling_t;

typedef enum BME280_IIR_FILTER
{
    BME280_IIR_FILTER_OFF = 0x00,
    BME280_IIR_FILTER_2   = 0x01,
    BME280_IIR_FILTER_4   = 0x02,
    BME280_IIR_FILTER_8   = 0x03,
    BME280_IIR_FILTER_16  = 0x04

} bme280_iir_filter_t;

typedef enum BME280_STANDY_TIME
{
    BME280_STANDBY_TIME_0_5_MS  = 0x00,
    BME280_STANDBY_TIME_62_5_MS = 0x01,
    BME280_STANDBY_TIME_125_MS  = 0x02,
    BME280_STANDBY_TIME_250_MS  = 0x03,
    BME280_STANDBY_TIME_500_MS  = 0x04,
    BME280_STANDBY_TIME_1000_MS = 0x05,
    BME280_STANDBY_TIME_10_MS   = 0x06,
    BME280_STANDBY_TIME_20_MS   = 0x07
} bme280_standby_time_t;

typedef enum BME280_POWER_MODE
{
    BME280_SLEEP_MODE  = 0x00,
    BME280_FORCED_MODE = 0x01,
    BME280_NORMAL_MODE = 0x03
} bme280_power_mode_t;

typedef struct BME280_SETTINGS
{
    // Temperature oversampling
    bme280_oversampling_t temperatureOversampling;

    // Pressure oversampling
    bme280_oversampling_t pressureOversampling;

    // Humidity oversampling
    bme280_oversampling_t humidityOversampling;

    // IIR filter coefficient
    bme280_iir_filter_t iirFilterCoefficients;

    // Standby time
    bme280_standby_time_t standbyTime;

    // Power mode
    bme280_power_mode_t powerMode;
} bme280_settings_t;

typedef bme280_error_code_t (* bme280_read_t) (i2c_t const * const, uint8_t const, uint8_t const, uint8_t * const, uint8_t const);
typedef bme280_error_code_t (* bme280_write_t) (i2c_t const * const, uint8_t const, uint8_t const * const, uint8_t const * const, uint8_t const);

typedef struct BME280_HANDLER
{
    bme280_read_t Read;
    bme280_write_t Write;
} bme280_handler_t;

typedef struct BME280_DEVICE
{
    // I2C address of the device
    uint8_t i2cAddress;

    // I2C device
    i2c_t const * i2cDevice;

    // Handler
    bme280_handler_t const * handler;

    // Sensor settings
    bme280_settings_t settings;

    // Calibration data;
    bme280_calibration_data_t calibrationData;

    // Data
    bme280_data_t data;

} bme280_device_t;

/**
 * @brief Checks if the device and its bus handler are valid.
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
static bme280_error_code_t BME280_ReadRegisters(i2c_t const * const i2c, uint8_t const address, uint8_t const registerAddress, uint8_t * const dataBuffer, uint8_t const bufferLength);

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
static bme280_error_code_t BME280_WriteRegisters(i2c_t const * const i2c, uint8_t const address, uint8_t const * const registerAddresses, uint8_t const * const dataBuffer, uint8_t const bufferLength);

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

/**
 * @brief Initializes the BME280 device.
 *
 * @param[in, out] device BME280 device
 * @param[in]      handler Read and write operations handler
 * @param[in]      i2cDevice I2C device
 * @param[in]      i2cAddress I2C device address
 * @param[in]      settings The settings to set
 *
 * @return bme280_error_code_t Error code
 **/
bme280_error_code_t BME280_Init(bme280_device_t * const device, bme280_handler_t const * const handler, i2c_t const * const i2cDevice, uint8_t const i2cAddress, bme280_settings_t const * const settings);

/**
 * @brief Reads the sensor data from the device and stores it in the sensor data
 *        structure.
 *
 * @param[in, out] device BME280 device
 *
 * @return bme280_error_code_t Error code
 **/
bme280_error_code_t BME280_GetSensorData(bme280_device_t * const device);

/**
 * @brief Returns the temperature from the last measurement.
 *
 * @param[in] device BME280 device
 *
 * @return int32_t The temperature
 **/
int32_t BME280_GetTemperature(bme280_device_t const * const device);

/**
 * @brief Returns the pressure from the last measurement.
 *
 * @param[in] device BME280 device
 *
 * @return uint32_t The pressure
 **/
uint32_t BME280_GetPressure(bme280_device_t const * const device);

/**
 * @brief Returns the humidity from the last measurement.
 *
 * @param[in] device BME280 device
 *
 * @return uint32_t The humidity
 **/
uint32_t BME280_GetHuimidity(bme280_device_t const * const device);

/**
 * @brief Returns the temperature from the last measurement in display format
 *        (as a floating point number).
 *
 * @param[in] device BME280 device
 *
 * @return double The temperature
 **/
double BME280_GetDisplayTemperature(bme280_device_t const * const device);

/**
 * @brief Returns the pressure from the last measurement in display format
 *        (as a floating point number).
 *
 * @param[in] device BME280 device
 *
 * @return double The pressure
 **/
double BME280_GetDisplayPressure(bme280_device_t const * const device);

/**
 * @brief Returns the humidity from the last measurement in display format
 *        (as a floating point number).
 *
 * @param[in] device BME280 device
 *
 * @return double The humidity
 **/
double BME280_GetDisplayHumidity(bme280_device_t const * const device);

#endif // BME280_H
