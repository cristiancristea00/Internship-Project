/**
 *  @file uart.h
 *  @author Cristian Cristea - M70957
 *  @date July 20, 2022
 *
 *  @brief Header file for the UART module
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


#ifndef UART_H
#define	UART_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "config.h"

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef void (* uart_callback_t) (uint8_t const);
typedef void (* uart_initialize_t) (uint32_t const);
typedef void (* uart_initialize_receive_t) (uint32_t const, uart_callback_t const);
typedef void (* uart_send_byte_t) (uint8_t const);
typedef void (* uart_send_data_t) (uint8_t const * const, uint8_t const);
typedef void (* uart_print_char_t) (char const);
typedef void (* uart_print_t) (char const * const);
typedef void (* uart_register_callback_t) (uart_callback_t const);

typedef struct UART
{
    uart_initialize_t Initialize;
    uart_initialize_receive_t InitializeWithReceive;
    uart_send_byte_t SendByte;
    uart_send_data_t SendData;
    uart_print_char_t PrintChar;
    uart_print_t Print;
    uart_register_callback_t RegisterCallback;
} uart_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

extern uart_t const uart_0;

extern uart_t const uart_1;

#endif // UART_H
