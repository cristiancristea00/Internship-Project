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


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "config.h"
#include "vector.h"
#include "uart.h"
#include "crc8.h"

#include <util/delay.h>

#include <stddef.h>
#include <stdbool.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum HC05_ERROR_CODE
{
    HC05_OK           = 0x00,
    HC05_NULL_POINTER = 0x01,
    HC05_SEND_FAILED  = 0x02
} hc05_error_code_t;

typedef enum HC05_STATUS
{
    HC05_IDLE        = 0x00,
    HC05_IN_PROGRESS = 0x01,
    HC05_FINISHED    = 0x02,
} hc05_status_t;

typedef enum HC05_RESPONSE
{
    HC05_EMPTY  = 0x00,
    HC05_ACKED  = 0x01,
    HC05_NACKED = 0x02
} hc05_response_t;

typedef struct HC05_DEVICE
{
    // UART device
    uart_t const * uartDevice;
} hc05_device_t;

typedef void (* struct_interpret_t) (void * const, vector_t * const);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*
 * TODO
 */
hc05_error_code_t HC05_Initialize(hc05_device_t * const device, uart_t const * const uartDevice);

/*
 * TODO
 */
hc05_error_code_t HC05_ReceiveData(hc05_device_t const * const device, void * const dataStructure, struct_interpret_t structInterpreter);

/*
 * TODO
 */
hc05_error_code_t HC05_SendData(hc05_device_t const * const device, uint8_t const * const buffer, uint8_t const bufferSize);

#endif // HC05_H
