/**
 *  @file main.c
 *  @author Cristian Cristea - M70957
 *  @date 18 July 2022
 *
 *  @brief Main source file for the project
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
#include "uart.h"
#include "i2c.h"
#include "bme280.h"

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <util/delay.h>

#include <stdbool.h>

extern uart_t const uart_1;
extern i2c_t const i2c_0;
extern bme280_handler_t const defaultHandler;

void BusScan(void);
void SensorRead(bme280_device_t const * const device);

void main(void)
{
    PORTC.DIRSET = PIN0_bm;

    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc, PRESCALE_DISABLED);

    uart_1.Init(UART_BAUD_RATE(460800));

    i2c_0.Init(I2C_FAST_MODE_PLUS);

    _delay_ms(5000);

    bme280_device_t weatherClick;

    bme280_settings_t settings = {
        .temperatureOversampling = BME280_OVERSAMPLING_4X,
        .pressureOversampling = BME280_OVERSAMPLING_4X,
        .humidityOversampling = BME280_OVERSAMPLING_4X,
        .iirFilterCoefficients = BME280_IIR_FILTER_8,
        .powerMode = BME280_NORMAL_MODE,
        .standbyTime = BME280_STANDBY_TIME_0_5_MS
    };

    BME280_Init(&weatherClick, &defaultHandler, &i2c_0, BME280_I2C_ADDRESS, &settings);

    while (true)
    {
        SensorRead(&weatherClick);
        _delay_ms(5000);
    }
}

void SensorRead(bme280_device_t const * const device)
{
    bme280_error_code_t readResult = BME280_GetSensorData(device);

    if (readResult != BME280_OK)
    {
        return;
    }

    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf\n\r", device->data.temperature, device->data.pressure, device->data.humidity);

    return;
}

void BusScan(void)
{
    uart_1.Print("\n\rI2C Scan started from 0x00 to 0x7F");

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