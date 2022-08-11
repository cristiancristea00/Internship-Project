/**
 *  @file hc-05.c
 *  @author Cristian Cristea - M70957
 *  @date 11 August 2022
 *
 *  @brief Source file for the HC-05 module
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


#include "hc-05.h"

hc05_error_code HC05_Initialize(hc05_device_t * const device, uart_t const * const uartDevice)
{
    LOG_INFO("Started HC-05 initialization");

    device->uartDevice = uartDevice;

    hc05_error_code initResult = HC05_CheckNull(device);

    Vector_Initialize(&device->buffer);

    if (initResult == HC05_OK)
    {
        LOG_INFO("Finished the HC-05 initialization");
    }
    else
    {
        LOG_ERROR("Couldn't finish the HC-05 initialization");
    }

    return initResult;
}

static hc05_error_code HC05_CheckNull(hc05_device_t const * const device)
{
    if (device == NULL || device->uartDevice == NULL)
    {
        LOG_ERROR("Found NULL pointer in HC-05 check");
        return HC05_NULL_POINTER;
    }
    else
    {
        return HC05_OK;
    }
}

static void HC05_SendData(hc05_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize)
{
    device->uartDevice->SendData(buffer, bufferSize);

    return;
}