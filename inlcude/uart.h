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

#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/io.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>

/**
 * @brief Macro to convert UART baud rate to the value to be used in the BAUD
 *        register of the USART.
 *
 **/
#define UART_BAUD_RATE(x) ((uint16_t) ((4UL * F_CPU) / (x)))

typedef enum UART_RECEIVE
{
    UART_RECEIVE_DISABLED = 0x00,
    UART_RECEIVE_ENABLED = 0x01
} uart_receive_t;


typedef void (* uart_callback_t) (uint8_t const);

typedef void (* uart_initialize_t) (uint32_t const, uart_receive_t const);
typedef void (* uart_send_byte_t) (uint8_t const);
typedef void (* uart_send_data_t) (uint8_t const * const, uint8_t const);
typedef void (* uart_print_char_t) (char const);
typedef void (* uart_print_t) (char const * const);
typedef void (* uart_register_callback_t) (uart_callback_t const);

typedef struct UART
{
    uart_initialize_t Initialize;
    uart_send_byte_t SendByte;
    uart_send_data_t SendData;
    uart_print_char_t PrintChar;
    uart_print_t Print;
    uart_register_callback_t RegisterCallback;
} uart_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            UART0 Declarations                              //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize the UART0 module by setting the baud rate and enabling the
 *        transmitter and receiver. The UART1 module is configured for 8-bit
 *        with no parity and 1 stop bit. Also, the Receive Complete Interrupt is
 *        enabled.
 *
 * @param[in] baudRate The baud rate register value
 **/
static void UART0_Initialize(uint32_t const baudRate, uart_receive_t const enableReceive);

/**
 * @brief Sends a null-terminated string over UART0.
 *
 * @param[in] string The null-terminated string to be sent
 **/
static void UART0_Print(char const * const string);

/**
 * @brief Send a single character over UART0.
 *
 * @param[in] character The character to be sent
 */
static void UART0_PrintChar(char const character);

/**
 * @brief Sends a number of bytes over UART0.
 *
 * @param[in] buffer The buffer containing the bytes to be sent
 * @param[in] bufferSize The number of bytes to be sent
 **/
static void UART0_SendData(uint8_t const * const buffer, uint8_t const bufferSize);

/**
 * @brief Sends a byte over UART0.
 *
 * @param[in] dataByte The byte to be sent
 **/
static void UART0_SendByte(uint8_t const dataByte);

/**
 * @brief Checks if the UART0 module is busy sending data.
 *
 * @return true The UART1 module is busy.
 * @return false The UART1 module is ready.
 **/
static bool UART0_TXBusy(void);

/**
 * @brief Registers a callback function to be called when a byte is received
 *        over UART0.
 *
 * @param[in] callback The callback function to be registered
 **/
static void UART0_RegisterCallback(uart_callback_t const callback);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            UART1 Declarations                              //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize the UART1 module by setting the baud rate and enabling the
 *        transmitter and receiver. The UART1 module is configured for 8-bit
 *        with no parity and 1 stop bit. Also, the Receive Complete Interrupt is
 *        enabled.
 *
 * @param[in] baudRate The baud rate register value
 **/
static void UART1_Initialize(uint32_t const baudRate, uart_receive_t const enableReceive);

/**
 * @brief Sends a null-terminated string over UART1.
 *
 * @param[in] string The null-terminated string to be sent
 **/
static void UART1_Print(char const * const string);

/**
 * @brief Send a single character over UART1.
 *
 * @param[in] character The character to be sent
 */
static void UART1_PrintChar(char const character);

/**
 * @brief Sends a number of bytes over UART1.
 *
 * @param[in] buffer The buffer containing the bytes to be sent
 * @param[in] bufferSize The number of bytes to be sent
 **/
static void UART1_SendData(uint8_t const * const buffer, uint8_t const bufferSize);

/**
 * @brief Sends a byte over UART1.
 *
 * @param[in] dataByte The byte to be sent
 **/
static void UART1_SendByte(uint8_t const dataByte);

/**
 * @brief Checks if the UART1 module is busy sending data.
 *
 * @return true The UART1 module is busy.
 * @return false The UART1 module is ready.
 **/
static bool UART1_TXBusy(void);

/**
 * @brief Registers a callback function to be called when a byte is received
 *        over UART1.
 *
 * @param[in] callback The callback function to be registered
 **/
static void UART1_RegisterCallback(uart_callback_t const callback);

#ifdef UART_PRINTF

/**
 * @brief Wrapper around the @ref UART1_PrintChar function to make it compatible
 *        with the C stream interface.
 *
 * @param[in] character The character to be sent
 * @param[in] stream The stream used to send the character
 *
 * @return int8_t Always returns 0
 **/
static int8_t UART1_SendChar(char const character, FILE * const stream);

#endif // UART_PRINTF

#endif // UART_H
