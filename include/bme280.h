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

#define BME280_I2C_ADDRESS 0x76

#define BME280_CHIP_ID_ADDRESS 0xD0

#define BME280_CHIP_ID 0x60

#define BME280_RESET_ADDRESS 0xE0

#define BME280_SOFT_RESET_COMMAND 0xB6

#define BME280_TEMP_PRESS_CALIB_ADDRESS 0x88
#define BME280_HUMIDITY_CALIB_ADDRESS   0xE1

#define BME280_STATUS_UPDATE 0x01

#define BME280_STATUS_REGISTER_ADDRESS 0xF3

#define BME280_TEMP_PRESS_CALIB_LENGTH 26
#define BME280_HUMIDITY_CALIB_LENGTH   7


#define BME280_MIN_TEMPERATURE  -4000
#define BME280_MAX_TEMPERATURE   8500
#define BME280_MIN_PRESSURE      3000000
#define BME280_MAX_PRESSURE      11000000
#define BME280_MAX_HUMIDITY      102400

typedef enum BME280_ERROR_CODE
{
    BME280_OK                  = 0,
    BME280_DEVICE_NOT_FOUND    = 1,
    BME280_NULL_POINTER        = 2,
    BME280_COMMUNICATION_ERROR = 3,
    BME280_INVALID_LENGTH      = 4,
    BME280_NVM_COPY_FAILED     = 5
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


typedef struct BME280_SETTINGS
{
    // Temperature oversampling
    uint8_t temperatureOversampling;

    // Pressure oversampling
    uint8_t pressureOversampling;

    // Humidity oversampling
    uint8_t humidityOversampling;

    // IIR filter coefficient
    uint8_t iirFilterCoefficient;

    // Standby time
    uint8_t standbyTime;
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

static bme280_error_code_t Bme280CheckNull(bme280_device_t const * const device);

static bme280_error_code_t Bm280ReadRegisters(i2c_t * const i2c, uint8_t const address, uint8_t const registerAddress, uint8_t * const data, uint8_t const length);

static bme280_error_code_t Bm280WriteRegisters(i2c_t * const i2c, uint8_t const address, uint8_t const * const registerAddresses, uint8_t const * const data, uint8_t const length);

static bme280_error_code_t Bme280GetRegisters(bme280_device_t * const device, uint8_t const registerAddress, uint8_t * const data, uint8_t const length);

static bme280_error_code_t Bme280SetRegisters(bme280_device_t * const device, uint8_t const * const registerAddresses, uint8_t const * const data, uint8_t const length);

static bme280_error_code_t Bme280SoftReset(bme280_device_t * const device);

static void Bme280ParseTemperatureAndPressureCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData);

static void Bme280ParseHumidityCalibration(bme280_calibration_data_t * const calibrationData, uint8_t const * const rawData);

static bme280_error_code_t Bm280GetCalibrationData(bme280_device_t * const device);

static int32_t Bme280CompensateTemperature(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData);

static uint32_t Bme280CompensatePressure(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData);

static uint32_t Bme280CompensateHumidity(bme280_uncompensated_data_t * const uncompensatedData, bme280_calibration_data_t * const calibrationData);

static bme280_error_code_t BMe280CompensateData(bme280_device_t * const device, bme280_uncompensated_data_t * const uncompensatedData);

bme280_error_code_t Bme280Init(bme280_device_t * const device, bme280_handler_t const * const handler, i2c_t const * const handle, uint8_t const i2cAddress);

#endif // BME280_H
