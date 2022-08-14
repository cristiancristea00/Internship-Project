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
#include "uart.h"

// DEBUG: TODO static
vector_t HC05_BUFFER;

static hc05_status_t transsmitionStatus = HC05_IDLE;

hc05_error_code HC05_Initialize(hc05_device_t * const device, uart_t const * const uartDevice)
{
    LOG_INFO("Started HC-05 initialization");

    Vector_Initialize(&HC05_BUFFER);

    uartDevice->InitializeWithReceive(460800, NULL);
    device->uartDevice = uartDevice;
    device->uartDevice->RegisterCallback(HC05_ReceiveCallback);

    hc05_error_code initResult = HC05_CheckNull(device);

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

static hc05_error_code HC05_SendData(hc05_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize)
{
    hc05_error_code sendResult = HC05_OK;

    sendResult = HC05_CheckNull(device);

    if (sendResult == HC05_OK)
    {
        uint8_t tryCount = 10;
        hc05_response_t transsmitionResponse = HC05_EMPTY;

        while (tryCount != 0)
        {
            device->uartDevice->SendData(buffer, bufferSize);

            transsmitionResponse = HC05_WaitForConfirmation();

            if (transsmitionResponse == HC05_ACKED)
            {
                sendResult = HC05_OK;
                break;
            }
            else if (transsmitionResponse == HC05_NACKED)
            {
                LOG_WARNING("HC-05 transsmition partner NACKED. Retrying...");
            }
            else
            {
                LOG_WARNING("HC-05 transsmition partner is not responding. Retrying...");
            }

            --tryCount;
            _delay_ms(1);
        }
    }

    transsmitionStatus = HC05_IDLE;

    return sendResult;
}

static hc05_response_t HC05_WaitForConfirmation(void)
{
    while (transsmitionStatus != HC05_FINISHED)
    {
        TightLoopContents();
    }

    return (hc05_response_t) Vector_LastByte(&HC05_BUFFER);
}

static void HC05_ReceiveCallback(uint8_t const data)
{
    if (transsmitionStatus == HC05_IDLE)
    {
        transsmitionStatus = HC05_IN_PROGRESS;
        Vector_AddByte(&HC05_BUFFER, data);
    }
    else if (transsmitionStatus == HC05_IN_PROGRESS)
    {
        if (HC05_IsLastByte())
        {
            transsmitionStatus = HC05_FINISHED;
        }
        Vector_AddByte(&HC05_BUFFER, data);
    }

    return;
}

static bool HC05_IsLastByte(void)
{
    return (HC05_GetNumberOfBytesToReceive() - 1) == HC05_BUFFER.bufferSize;
}

static uint8_t HC05_GetNumberOfBytesToReceive(void)
{
    return Vector_FirstByte(&HC05_BUFFER);
}

hc05_error_code HC05_ReceiveData(hc05_device_t const * const device, void * const dataStructure, struct_interpret_t structInterpreter)
{
    hc05_error_code receiveResult = HC05_OK;

    receiveResult = HC05_CheckNull(device);

    if (receiveResult == HC05_OK)
    {
        while (transsmitionStatus != HC05_FINISHED)
        {
            TightLoopContents();
        }

        receiveResult = HC05_VerifyChecksum();

        if (receiveResult == HC05_OK)
        {
            structInterpreter(dataStructure, &HC05_BUFFER);

            HC05_SendAcknowledge();
        }
        else
        {
            HC05_SendNotAcknowledge();
        }

        Vector_Clear(&HC05_BUFFER);
    }

    transsmitionStatus = HC05_IDLE;

    return receiveResult;
}

static hc05_error_code HC05_VerifyChecksum(void)
{
    // TODO

    return HC05_OK;
}

static void HC05_SendAcknowledge(void)
{
    // TODO

    return;
}

static void HC05_SendNotAcknowledge(void)
{
    // TODO

    return;
}