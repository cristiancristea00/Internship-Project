/**
 *  @file utils.c
 *  @author Cristian Cristea - M70957
 *  @date 30 August 2022
 *
 *  @brief Source file for utils functions for the Sensors station
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

#include "utils.h"

#include "config.h"
#include "uart.h"
#include "i2c.h"

#include <avr/io.h>

#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline void SystemInitialize(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart_1.Initialize(460800);
    uart_0.InitializeWithReceive(460800, HC05_ReceiveCallback);
    i2c_0.Initialize(I2C_FAST_MODE_PLUS);

    return;
}

__attribute__((always_inline)) inline void ModulesInitialize(bme280_device_t * const weatherClick, hc05_device_t * const bluetooth)
{
    bme280_settings_t settings = {
        .temperatureOversampling = BME280_OVERSAMPLING_16X,
        .pressureOversampling = BME280_OVERSAMPLING_16X,
        .humidityOversampling = BME280_OVERSAMPLING_16X,
        .iirFilterCoefficients = BME280_IIR_FILTER_8,
        .powerMode = BME280_NORMAL_MODE,
        .standbyTime = BME280_STANDBY_TIME_500_MS
    };
    BME280_Init(weatherClick, &BME280_I2C0_Handler, &i2c_0, BME280_I2C_ADDRESS, &settings);

    HC05_Initialize(bluetooth, &uart_0);

    return;
}

__attribute__((always_inline)) inline void DisplayData(bme280_data_t const * const sensorsData)
{
    printf("Temperature: %0.2lf °C\n\r", BME280_GetDisplayTemperature(sensorsData));
    printf("Pressure: %0.2lf hPa\n\r", BME280_GetDisplayPressure(sensorsData));
    printf("Relative humidity: %0.2lf %c\n\r", BME280_GetDisplayHumidity(sensorsData), '%');

    return;
}
