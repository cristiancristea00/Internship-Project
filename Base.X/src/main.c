/**
 *  @file main.c
 *  @author Cristian Cristea - M70957
 *  @date August 10, 2022
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
#include "utils.h"
#include "bme280.h"
#include "hc-05.h"
#include "oled.h"

void main(void)
{
    SystemInitialize();

    oled_device_t OLEDC;
    hc05_device_t bluetooth;

    ModulesInitialize(&OLEDC, &bluetooth);

    bme280_data_t sensorsData;

    while (true)
    {
        if (HC05_ReceiveData(&bluetooth, &sensorsData, BME280_StructInterpret) == HC05_OK)
        {
            DisplayData(&sensorsData, &OLEDC);
        }
    }
}