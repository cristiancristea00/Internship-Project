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

#include "config.h"

#include <avr/io.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>

/**
 * @brief Macro to convert UART baud rate to the value to be used in the BAUD
 *        register of the USART.
 *
 **/
#define UART_BAUD_RATE(x) ((uint16_t) ((4UL * F_CPU) / x ## UL))

typedef void (* uart_init_t) (uint16_t const);
typedef void (* uart_print_t) (char const *);
typedef void (* uart_print_char_t) (char const);

typedef struct UART
{
    uart_init_t Init;
    uart_print_t Print;
    uart_print_char_t PrintChar;
} uart_t;

/**
 * @brief Initialize the UART module by setting the baud rate and enabling the
 *        transmitter and receiver. The UART module is configured for 8-bit with
 *        no parity and 1 stop bit. Also, the Receive Complete Interrupt is
 *        enabled.
 *
 * @param baudRate The baud rate register value
 **/
static void UART1_Init(uint16_t const baudRate);

/**
 * @brief Sends a null-terminated string over UART.
 *
 * @param string The null-terminated string to be sent
 **/
static void UART1_Print(char const * string);

/**
 * @brief Send a single character over UART.
 *
 * @param character The character to be sent
 */
static void UART1_PrintChar(char const character);

/**
 * @brief Sends a byte over UART.
 *
 * @param dataByte The byte to be sent
 **/
static void UART1_SendByte(uint8_t const dataByte);

/**
 * @brief Checks if the UART module is busy sending data.
 *
 * @return true The UART module is busy.
 * @return false The UART module is ready.
 **/
static bool UART1_TXBusy(void);

#ifdef UART_PRINTF

/**
 * @brief Wrapper around the @ref Uart1PrintChar function to make it compatible
 *        with the C stream interface.
 *
 * @param character The character to be sent
 * @param stream The stream used to send the character
 * @return int8_t Always returns 0
 **/
static int8_t UART1_SendChar(char const character, FILE * const stream);

#endif // UART_PRINTF

#endif // UART_H
