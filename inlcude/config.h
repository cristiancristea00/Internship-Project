/**
 *  @file config.h
 *  @author Cristian Cristea - M70957
 *  @date July 20, 2022
 *
 *  @brief Configuration file for the project that contains definitions
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


#ifndef CONFIG_H
#define	CONFIG_H

#include "uart.h"

#include <avr/io.h>

#include <stdarg.h>

#define F_CPU 24000000UL           // The CPU frequency. To be set manuallly.

#define UART_PRINTF                // Uncomment to enable printf functionality on UART.
#define LOGGING                    // Uncomment to enable logging

#define PRESCALE_ENABLED  true     // Enable the CPU prescaler
#define PRESCALE_DISABLED false    // Disable the CPU prescaler

#ifdef LOGGING

#include <stdio.h>

/**
 * @brief TODO
 *
 **/
void PrintForLogging(char const * const message);

#define LOG_DEBUG_PRINTF(STRING, ...)    (printf("[DEBUG]: " STRING "\n\r", ##__VA_ARGS__))
#define LOG_INFO_PRINTF(STRING, ...)     (printf("[INFO]: " STRING "\n\r", ##__VA_ARGS__))
#define LOG_WARNING_PRINTF(STRING, ...)  (printf("[WARNING]: " STRING "\n\r", ##__VA_ARGS__))
#define LOG_ERROR_PRINTF(STRING, ...)    (printf("[ERROR]: " STRING "\n\r", ##__VA_ARGS__))

#define LOG_DEBUG(STRING)                (PrintForLogging("[DEBUG]: " STRING "\n\r"))
#define LOG_INFO(STRING)                 (PrintForLogging("[INFO]: " STRING "\n\r"))
#define LOG_WARNING(STRING)              (PrintForLogging("[WARNING]: " STRING "\n\r"))
#define LOG_ERROR(STRING)                (PrintForLogging("[ERROR]: " STRING "\n\r"))

#else

#define LOG_DEBUG_PRINTF
#define LOG_INFO_PRINTF
#define LOG_WARNING_PRINTF
#define LOG_ERROR_PRINTF

#define LOG_DEBUG
#define LOG_INFO
#define LOG_WARNING
#define LOG_ERROR

#endif // LOGGING

/**
 * @brief Sets the CPU's clock frequency and the prescaler factor.
 *
 * @param[in] frequency The desired frequency of the CPU's clock
 * @param[in] prescaler The prescaler factor
 **/
void SetClockFrequencyWithPrescaler(uint8_t const frequency, uint8_t const prescaler);

/**
 * @brief Sets the CPU's clock frequency and optionally the prescaler factor.
 *
 * @param[in] frequency The desired frequency of the CPU's clock
 **/
void SetClockFrequency(uint8_t const frequency);

/**
 * @brief Empty function that should be used in loops for better readability.
 *
 **/
void TightLoopContents(void);

#endif // CONFIG_H
