/**
 *  @file main.c
 *  @author Cristian Cristea - M70957
 *  @date 18 July 2022
 *
 *  @brief Main source file for the Client program
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

#include <avr/cpufunc.h>
#include <util/delay.h>

#include <stdbool.h>


extern uart_t const uart_0;
extern uart_t const uart_1;
extern i2c_t const i2c_0;
extern bme280_handler_t const BME280_I2C0_Handler;

// DEBUG
extern vector_t HC05_BUFFER;

//static void BusScan(void);
//static void SensorRead(bme280_device_t * const device);

void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart_1.Initialize(460800);

    //    i2c_0.Initialize(I2C_FAST_MODE_PLUS);
    //
    //    BusScan();
    //
    //    bme280_device_t weatherClick;
    //
    //    bme280_settings_t settings = {
    //        .temperatureOversampling = BME280_OVERSAMPLING_16X,
    //        .pressureOversampling = BME280_OVERSAMPLING_16X,
    //        .humidityOversampling = BME280_OVERSAMPLING_16X,
    //        .iirFilterCoefficients = BME280_IIR_FILTER_8,
    //        .powerMode = BME280_NORMAL_MODE,
    //        .standbyTime = BME280_STANDBY_TIME_500_MS
    //    };
    //
    //    BME280_Init(&weatherClick, &BME280_I2C0_Handler, &i2c_0, BME280_I2C_ADDRESS, &settings);
    //
    hc05_device_t sensorStation;

    HC05_Initialize(&sensorStation, &uart_0);

    bme280_data_t temp;

    while (true)
    {
        HC05_ReceiveData(&sensorStation, &temp, BME280_StructInterpret);

        uart_1.Print("\n\rData: ");
        for (uint8_t i = 0; i < HC05_BUFFER.bufferSize; ++i)
        {
            printf("0x%02X ", HC05_BUFFER.internalBuffer[i]);
        }
        uart_1.Print("\n\r");
        _delay_ms(5000);
    }
}

//static void SensorRead(bme280_device_t * const device)
//{
//    bme280_error_code_t readResult = BME280_GetSensorData(device);
//
//    if (readResult != BME280_OK)
//    {
//        return;
//    }
//
//    printf("Temperature: %0.2lf °C\n\r", BME280_GetDisplayTemperature(device));
//    printf("Pressure: %0.2lf hPa\n\r", BME280_GetDisplayPressure(device));
//    printf("Relative humidity: %0.2lf%c\n\r", BME280_GetDisplayHumidity(device), '%');
//
//    return;
//}
//
//static void BusScan(void)
//{
//    uart_1.Print("I2C Scan started from 0x00 to 0x7F");
//
//    for (uint8_t clientAddress = I2C_ADRESS_MIN; clientAddress <= I2C_ADRESS_MAX; ++clientAddress)
//    {
//        printf("\n\rScanning client address: 0x%02X", clientAddress);
//        if (i2c_0.ClientAvailable(clientAddress))
//        {
//            uart_1.Print(" --> client ACKED");
//        }
//    }
//    uart_1.Print("\n\rI2C Scan ended\n\r");
//
//    return;
//}