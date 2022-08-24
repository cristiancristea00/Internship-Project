/**
 *  @file main.c
 *  @author Cristian Cristea - M70957
 *  @date 18 July 2022
 *
 *  @brief Main source file for the Base station
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


#include "config.h"
#include "vector.h"
#include "bme280.h"
#include "hc-05.h"
#include "uart.h"
#include "crc8.h"
#include "i2c.h"

#include <avr/io.h>
#include <util/delay.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


static void SensorRead(bme280_device_t * const device);

void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart_1.Initialize(460800);

    bme280_device_t weatherClick;

    bme280_settings_t settings = {
        .temperatureOversampling = BME280_OVERSAMPLING_16X,
        .pressureOversampling = BME280_OVERSAMPLING_16X,
        .humidityOversampling = BME280_OVERSAMPLING_16X,
        .iirFilterCoefficients = BME280_IIR_FILTER_8,
        .powerMode = BME280_NORMAL_MODE,
        .standbyTime = BME280_STANDBY_TIME_500_MS
    };

    BME280_Init(&weatherClick, &BME280_I2C0_Handler, &i2c_0, BME280_I2C_ADDRESS, &settings);

    hc05_device_t sensorStation;
    HC05_Initialize(&sensorStation, &uart_0);

    uint8_t serializedSensorData[BME280_SERIALIZED_SIZE] = { 0 };

    PauseMiliseconds(5000);

    while (true)
    {
        SensorRead(&weatherClick);
        BME280_SerializeSensorData(&weatherClick, &serializedSensorData);
        HC05_SendData(&sensorStation, &serializedSensorData, BME280_SERIALIZED_SIZE);
        PauseMiliseconds(5000);
    }
}

static void SensorRead(bme280_device_t * const device)
{
    bme280_error_code_t readResult = BME280_GetSensorData(device);

    if (readResult != BME280_OK)
    {
        return;
    }

    bme280_data_t sensorData = device->data;

    printf("Temperature: %0.2lf °C\n\r", BME280_GetDisplayTemperature(&sensorData));
    printf("Pressure: %0.2lf hPa\n\r", BME280_GetDisplayPressure(&sensorData));
    printf("Relative humidity: %0.2lf %c\n\r", BME280_GetDisplayHumidity(&sensorData), '%');

    return;
}
