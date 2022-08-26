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


#include "mics-6814.h"
#include "ads1015.h"
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


__attribute__((always_inline)) inline static void BusScan(void);
__attribute__((always_inline)) inline static void SensorRead(bme280_device_t * const weatherClick, mics6814_device_t * const airQualityClick);

void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart_1.Initialize(460800);
    i2c_0.Initialize(I2C_MODE_FAST_PLUS);

    BusScan();

    bme280_device_t weatherClick;
    bme280_settings_t settings = {
        .temperatureOversampling = BME280_OVERSAMPLING_16X,
        .pressureOversampling = BME280_OVERSAMPLING_16X,
        .humidityOversampling = BME280_OVERSAMPLING_16X,
        .iirFilterCoefficients = BME280_IIR_FILTER_16,
        .powerMode = BME280_NORMAL_MODE,
        .standbyTime = BME280_STANDBY_TIME_250_MS
    };
    BME280_Initialize(&weatherClick, &BME280_I2C0_Handler, &i2c_0, BME280_I2C_ADDRESS, &settings);

    ads1015_device_t airQualityClickADC;
    mics6814_device_t airQualityClick;
    MICS6814_Initialize(&airQualityClick, &airQualityClickADC, &i2c_0);

    hc05_device_t sensorStation;
    HC05_Initialize(&sensorStation, &uart_0);

    uint8_t serializedSensorData[BME280_SERIALIZED_SIZE] = { 0 };

    PauseMiliseconds(5000);

    while (true)
    {
        SensorRead(&weatherClick, &airQualityClick);
        BME280_SerializeSensorData(&weatherClick, &serializedSensorData);
        HC05_SendData(&sensorStation, &serializedSensorData, BME280_SERIALIZED_SIZE);
        PauseMiliseconds(5000);
    }
}

__attribute__((always_inline)) inline static void SensorRead(bme280_device_t * const weatherClick, mics6814_device_t * const airQualityClick)
{
    BME280_GetSensorData(weatherClick);
    bme280_data_t sensorData = weatherClick->data;

    printf("Temperature: %0.2lf °C\n\r", BME280_GetDisplayTemperature(&sensorData));
    printf("Pressure: %0.2lf hPa\n\r", BME280_GetDisplayPressure(&sensorData));
    printf("Relative humidity: %0.2lf %c\n\r", BME280_GetDisplayHumidity(&sensorData), '%');

    printf("Carbon monoxide (CO): %0.2lf ppm\n\r", MICS6814_GetCarbonMonoxide(airQualityClick));
    printf("Nitrogen dioxide (NO2): %0.2lf ppm\n\r", MICS6814_GetNitrogenDioxide(airQualityClick));
    printf("Ethanol (C2H5OH): %0.2lf ppm\n\r", MICS6814_GetEthanol(airQualityClick));
    printf("Hydrogen (H2): %0.2lf ppm\n\r", MICS6814_GetHydrogen(airQualityClick));
    printf("Ammonia (NH3): %0.2lf ppm\n\r", MICS6814_GetAmmonia(airQualityClick));
    printf("Methane (CH4): %0.2lf ppm\n\r", MICS6814_GetMethane(airQualityClick));
    printf("Propane (C3H8): %0.2lf ppm\n\r", MICS6814_GetPropane(airQualityClick));
    printf("Isobutane (C4H10): %0.2lf ppm\n\r", MICS6814_GetIsobutane(airQualityClick));

    return;
}

__attribute__((always_inline)) inline static void BusScan(void)
{
    printf("\n\rI2C Scan started from 0x%02X to 0x%02X:", I2C_ADRESS_MIN, I2C_ADRESS_MAX);

    for (uint8_t clientAddress = I2C_ADRESS_MIN; clientAddress <= I2C_ADRESS_MAX; ++clientAddress)
    {
        printf("\n\rScanning client address: 0x%02X", clientAddress);
        if (i2c_0.ClientAvailable(clientAddress))
        {
            uart_1.Print(" --> client ACKED");
        }
    }
    uart_1.Print("\n\rI2C Scan ended\n\r");

    return;
}