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


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define UINT8(X)          ((uint8_t) (X))

#define ACK_NACK_SIZE     UINT8(3)
#define ACK_NACK_BYTES    UINT8(1)


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*
 * TODO
 */
static hc05_error_code_t HC05_CheckNull(hc05_device_t const * const device);

/*
 * TODO
 */
__attribute__((always_inline)) inline static void HC05_SendPreparedData(hc05_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize);

/*
 * TODO
 */
static hc05_response_t HC05_WaitForConfirmation(void);

/*
 * TODO
 */
static hc05_response_t HC05_GetResponseReceived(void);

/*
 * TODO
 */
static void HC05_ReceiveCallback(uint8_t const data);

/*
 * TODO
 */
__attribute__((always_inline)) inline static bool HC05_IsLastByte(void);

/*
 * TODO
 */
__attribute__((always_inline)) inline static uint8_t HC05_GetNumberOfBytesToReceive(void);

/*
 * TODO
 */
static hc05_error_code_t HC05_VerifyReceiveChecksum(void);

/*
 * TODO
 */
static hc05_error_code_t HC05_VerifyChecksum(uint8_t const * const data, uint8_t const dataLength, uint8_t const checksum);

/*
 * TODO
 */
static void HC05_SendAcknowledge(hc05_device_t const * const device);

/*
 * TODO
 */
static void HC05_SendNotAcknowledge(hc05_device_t const * const device);

/*
 * TODO
 */
__attribute__((always_inline)) inline static void HC05_SendResponse(hc05_device_t const * const device, hc05_response_t const response);

/*
 * TODO
 */
__attribute__((always_inline)) inline static uint8_t HC05_ComputeChecksum(uint8_t const * const data, uint8_t const dataLength);

/*
 * TODO
 */
__attribute__((always_inline)) inline static void HC05_EndTransmission(void);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

static vector_t HC05_BUFFER;

static hc05_status_t transsmitionStatus = HC05_IDLE;

static hc05_error_code_t HC05_CheckNull(hc05_device_t const * const device)
{
    if ((device == NULL) || (device->uartDevice == NULL))
    {
        LOG_ERROR("Found NULL pointer in HC-05 check");
        return HC05_NULL_POINTER;
    }
    else
    {
        return HC05_OK;
    }
}

__attribute__((always_inline)) inline static void HC05_SendPreparedData(hc05_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize)
{
    uint8_t const newLength = bufferSize + 2;

    // Please forgive me for the sin that I'm committing

    uint8_t * preparedData = (uint8_t *) calloc(newLength, sizeof (uint8_t));

    preparedData[0] = bufferSize;
    memcpy(preparedData + 1, buffer, bufferSize);
    preparedData[newLength - 1] = HC05_ComputeChecksum(preparedData, bufferSize);

    device->uartDevice->SendData(preparedData, newLength);

    // My soul is finally free

    free(preparedData);
    preparedData = NULL;

    return;
}

static hc05_response_t HC05_WaitForConfirmation(void)
{
    PauseMiliseconds(500);

    if (transsmitionStatus == HC05_FINISHED)
    {
        return HC05_GetResponseReceived();
    }
    else
    {
        return HC05_EMPTY;
    }
}

static hc05_response_t HC05_GetResponseReceived(void)
{
    uint8_t response[ACK_NACK_SIZE] = { 0 };

    response[2] = Vector_RemoveByte(&HC05_BUFFER);
    response[1] = Vector_RemoveByte(&HC05_BUFFER);
    response[0] = Vector_RemoveByte(&HC05_BUFFER);

    if (HC05_VerifyChecksum(response, ACK_NACK_BYTES, response[2]) == HC05_OK)
    {
        return (hc05_response_t) response[1];
    }
    else
    {
        return HC05_EMPTY;
    }
}

static void HC05_ReceiveCallback(uint8_t const data)
{
    if ((transsmitionStatus == HC05_IDLE) && (data != 0))
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

__attribute__((always_inline)) inline static bool HC05_IsLastByte(void)
{
    return HC05_GetNumberOfBytesToReceive() == HC05_BUFFER.bufferSize;
}

__attribute__((always_inline)) inline static uint8_t HC05_GetNumberOfBytesToReceive(void)
{
    return Vector_FirstByte(&HC05_BUFFER) + UINT8(1);
}

static hc05_error_code_t HC05_VerifyReceiveChecksum(void)
{
    uint8_t const * const data = HC05_BUFFER.internalBuffer;
    uint8_t const dataLength = Vector_FirstByte(&HC05_BUFFER);
    uint8_t const checksum = Vector_LastByte(&HC05_BUFFER);

    return HC05_VerifyChecksum(data, dataLength, checksum);
}

static hc05_error_code_t HC05_VerifyChecksum(uint8_t const * const data, uint8_t const dataLength, uint8_t const checksum)
{
    uint8_t const computedChecksum = HC05_ComputeChecksum(data, dataLength);

    if (checksum == computedChecksum)
    {
        return HC05_OK;
    }
    else
    {
        return HC05_FAILED_CHECKSUM;
    }
}

static void HC05_SendAcknowledge(hc05_device_t const * const device)
{
    HC05_SendResponse(device, HC05_ACKED);

    return;
}

static void HC05_SendNotAcknowledge(hc05_device_t const * const device)
{
    HC05_SendResponse(device, HC05_NACKED);

    return;
}

__attribute__((always_inline)) inline static void HC05_SendResponse(hc05_device_t const * const device, hc05_response_t const response)
{
    uint8_t packet[ACK_NACK_SIZE] = { 0 };

    packet[0] = ACK_NACK_BYTES;
    packet[1] = UINT8(response);
    packet[2] = HC05_ComputeChecksum(packet, ACK_NACK_BYTES);

    device->uartDevice->SendData(packet, ACK_NACK_SIZE);

    return;
}

__attribute__((always_inline)) inline static uint8_t HC05_ComputeChecksum(uint8_t const * const data, uint8_t const dataLength)
{
    return CRC8_Compute(data, dataLength + 1);
}

__attribute__((always_inline)) inline static void HC05_EndTransmission(void)
{
    transsmitionStatus = HC05_IDLE;
    Vector_Clear(&HC05_BUFFER);

    return;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

hc05_error_code_t HC05_Initialize(hc05_device_t * const device, uart_t const * const uartDevice)
{
    LOG_INFO("Started HC-05 initialization");

    Vector_Initialize(&HC05_BUFFER);

    uartDevice->InitializeWithReceive(460800, HC05_ReceiveCallback);
    device->uartDevice = uartDevice;

    hc05_error_code_t initResult = HC05_CheckNull(device);

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

hc05_error_code_t HC05_SendData(hc05_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize)
{
    hc05_error_code_t sendResult = HC05_OK;

    uint8_t tryCount = 10;

    sendResult = HC05_CheckNull(device);

    if (sendResult == HC05_OK)
    {
        LOG_INFO("Started HC-05 send transmission");

        hc05_response_t transsmitionResponse = HC05_EMPTY;

        while (tryCount != UINT8(0))
        {
            HC05_SendPreparedData(device, buffer, bufferSize);

            transsmitionResponse = HC05_WaitForConfirmation();

            if (transsmitionResponse == HC05_ACKED)
            {
                sendResult = HC05_OK;
                break;
            }
            else if (transsmitionResponse == HC05_NACKED)
            {
                LOG_WARNING("HC-05 send transmission partner NACKED. Retrying...");
            }
            else
            {
                LOG_WARNING("HC-05 send transmission partner is not responding. Retrying...");
            }

            --tryCount;
            _delay_ms(10);
        }
    }

    if (tryCount == UINT8(0))
    {
        LOG_ERROR("HC-05 send transmission failed");
    }
    else
    {
        LOG_INFO("Finished HC-05 send transmission");
    }

    HC05_EndTransmission();

    return sendResult;
}

hc05_error_code_t HC05_ReceiveData(hc05_device_t const * const device, void * const dataStructure, struct_interpret_t structInterpreter)
{
    hc05_error_code_t receiveResult = HC05_OK;

    receiveResult = HC05_CheckNull(device);

    if (receiveResult == HC05_OK)
    {
        LOG_INFO("Started HC-05 receive transmission");

        while (transsmitionStatus != HC05_FINISHED)
        {
            TightLoopContents();
        }

        receiveResult = HC05_VerifyReceiveChecksum();

        if (receiveResult == HC05_OK)
        {
            structInterpreter(dataStructure, &HC05_BUFFER);

            HC05_SendAcknowledge(device);

            LOG_INFO("Finished HC-05 receive transmission");
        }
        else
        {
            HC05_SendNotAcknowledge(device);

            LOG_ERROR("HC-05 receive transmission failed");
        }
    }

    HC05_EndTransmission();

    return receiveResult;
}