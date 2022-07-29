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
 * @brief
 * TODO - Add description
 **/
#define BME280_CONCAT_BYTES(MSB, LSB) (((uint16_t) (MSB) << 8) | (uint16_t) (LSB))

/**
 * @brief
 * TODO - Add description
 **/
#define BME280_SET_BITS(REGISTER_DATA, BITNAME, DATA) ((REGISTER_DATA & ~(BITNAME ## _bm)) | ((DATA << BITNAME ## _bp) & BITNAME ## _bm))

/**
 * @brief
 * TODO - Add description
 **/
#define BME280_GET_BITS(REGISTER_DATA, BITNAME) ((REGISTER_DATA & ~(BITNAME ## _bm)) | (data & BITNAME ## _bm))

// Principal and secondary I2C addresses of the chip
#define BME280_I2C_ADDRESS        0x76
#define BME280_I2C_ADDRESS_SEC    0x77

// The BME280 chip identifier
#define BME280_CHIP_ID    0x60

// Register addresses
#define BME280_CHIP_ID_ADDRESS             0xD0
#define BME280_RESET_ADDRESS               0xE0
#define BME280_TEMP_PRESS_CALIB_ADDRESS    0x88
#define BME280_HUMIDITY_CALIB_ADDRESS      0xE1
#define BME280_DATA_ADDRESS                0xF7

// Registers related to sizes
#define BME280_TEMP_PRESS_CALIB_LENGTH    26
#define BME280_HUMIDITY_CALIB_LENGTH      7
#define BME280_DATA_LENGTH                8

// Status
#define BME280_STATUS_REGISTER_ADDRESS    0xF3
#define BME280_SOFT_RESET_COMMAND         0xB6
#define BME280_STATUS_UPDATE              0x01

// Value limits for measurements
#define BME280_MIN_TEMPERATURE    -4000L
#define BME280_MAX_TEMPERATURE    8500L
#define BME280_MIN_PRESSURE       3000000UL
#define BME280_MAX_PRESSURE       11000000UL
#define BME280_MAX_HUMIDITY       102400UL


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
} bme280_settings_t;

typedef bme280_error_code_t (* bme280_read_t) (i2c_t * const, uint8_t const, uint8_t const, uint8_t * const, uint8_t const);
typedef bme280_error_code_t (* bme280_write_t) (i2c_t * const, uint8_t const, uint8_t const * const, uint8_t const * const, uint8_t const);

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

static bme280_error_code_t BME280_CheckNull(bme280_device_t const * const device);

static bme280_error_code_t BME280_ReadRegisters(i2c_t * const i2c, uint8_t const address, uint8_t const registerAddress, uint8_t * const data, uint8_t const length);

static bme280_error_code_t BME280_WriteRegisters(i2c_t * const i2c, uint8_t const address, uint8_t const * const registerAddresses, uint8_t const * const data, uint8_t const length);

static bme280_error_code_t BME280_GetRegisters(bme280_device_t * const device, uint8_t const registerAddress, uint8_t * const data, uint8_t const length);

static bme280_error_code_t BME280_SetRegisters(bme280_device_t * const device, uint8_t const * const registerAddresses, uint8_t const * const data, uint8_t const length);

static bme280_error_code_t BME280_SoftReset(bme280_device_t * const device);

static void BME280_ParseTemperatureAndPressureCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData);

static void BME280_ParseHumidityCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData);

static bme280_error_code_t BME280_GetCalibrationData(bme280_device_t * const device);

static int32_t BME280_CompensateTemperature(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData);

static uint32_t BME280_CompensatePressure(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData);

static uint32_t BME280_CompensateHumidity(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData);

static bme280_error_code_t BME280_CompensateData(bme280_device_t * const device, bme280_uncompensated_data_t * const uncompensatedData);

static void BME280_ParseSensorData(uint8_t const * const data, bme280_uncompensated_data_t * const uncompensatedData);

static bme280_error_code_t BME280_SetOversamplingTemperaturePressure(bme280_device_t * const device, bme280_settings_t const * const settings);

static bme280_error_code_t BME280_SetOversamplingHumidity(bme280_device_t * const device, bme280_settings_t const * const settings);

bme280_error_code_t BME280_SetOversamplingSettings(bme280_device_t * const device, bme280_settings_t const * const settings);

bme280_error_code_t BME280_GetSensorData(bme280_device_t * const device);

bme280_error_code_t BME280_Init(bme280_device_t * const device, bme280_handler_t const * const handler, i2c_t const * const handle, uint8_t const i2cAddress);

#endif // BME280_H
