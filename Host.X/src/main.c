/**
 *  @file main.c
 *  @author Cristian Cristea - M70957
 *  @date August 10, 2022
 *
 *  @brief Main source file for the Host program
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
#include "bme280.h"
#include "hc-05.h"
#include "uart.h"

#include <util/delay.h>

extern uart_t const uart_0;
extern uart_t const uart_1;

static void DisplayData(bme280_data_t * const sensorsData);

void main(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart_1.Initialize(460800);

    hc05_device_t baseStation;
    HC05_Initialize(&baseStation, &uart_0);

    bme280_data_t sensorsData;

    _delay_ms(5000);

    uart_1.Print("Ready to receive data!\n\r");

    while (true)
    {
        HC05_ReceiveData(&baseStation, &sensorsData, BME280_StructInterpret);
        DisplayData(&sensorsData);
        _delay_ms(10000);
    }
}

static void DisplayData(bme280_data_t * const sensorsData)
{
    printf("Temperature: %0.2lf °C\n\r", BME280_GetDisplayTemperature(sensorsData));
    printf("Pressure: %0.2lf hPa\n\r", BME280_GetDisplayPressure(sensorsData));
    printf("Relative humidity: %0.2lf%c\n\r", BME280_GetDisplayHumidity(sensorsData), '%');

    return;
}
