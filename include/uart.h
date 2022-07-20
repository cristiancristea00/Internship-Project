/**
 *  @file uart.h
 *  @author Cristian Cristea - M70957
 *  @date July 20, 2022
 *
 *  @brief TODO: Short summary
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

#include "config.h"

#include <avr/io.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>


#define UART_BAUD_RATE(x) ((uint16_t) ((4UL * F_CPU) / x ## UL))


typedef void (* uart_callback_t) (uint8_t);


void Uart1Init(uint16_t const baudRate);

void Uart1RegisterCallback(uart_callback_t const callback);

void Uart1Print(char const * string);

static void Uart1SendByte(uint8_t const dataByte);

static bool Uart1TxBusy(void);

#ifdef UART_PRINTF

static int8_t Uart1PrintChar(char const character, FILE * const stream);

#endif // UART_PRINTF

#endif // UART_H

