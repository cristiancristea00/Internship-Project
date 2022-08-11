/**
 *  @file hc-05.h
 *  @author Cristian Cristea - M70957
 *  @date 11 August 2022
 *
 *  @brief Header file for the HC-05 module
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


#ifndef HC05_H
#define	HC05_H

#include "config.h"
#include "uart.h"

#include <stddef.h>


typedef enum HC05_ERROR_CODE
{
    HC05_OK           = 0x00,
    HC05_NULL_POINTER = 0x01
} hc05_error_code;

typedef struct HC05_DEVICE
{
    // UART device
    uart_t const * uartDevice;
} hc05_device_t;


/* TODO */
hc05_error_code HC05_Initialize(hc05_device_t * const device, uart_t const * const uartDevice);

/* TODO */
static hc05_error_code HC05_CheckNull(hc05_device_t const * const device);

/* TODO */
static void HC05_SendData(hc05_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize);

#endif // HC05_H

