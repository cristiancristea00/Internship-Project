/**
 *  @file uart.c
 *  @author Cristian Cristea - M70957
 *  @date July 20, 2022
 *
 *  @brief Source file for the UART module
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


#include "uart.h"

/**
 * @brief  Module for UART1
 *
 **/
uart_t const uart_1 = {
    .Initialize = UART1_Initialize,
    .Print = UART1_Print,
    .PrintChar = UART1_PrintChar,
};

#ifdef UART_PRINTF

static int8_t UART1_SendChar(char const character, FILE * const stream)
{
    UART1_PrintChar(character);

    return 0;
}

FILE uart1Stream = FDEV_SETUP_STREAM(UART1_PrintChar, NULL, _FDEV_SETUP_WRITE);

#endif // UART_PRINTF

static void UART1_Initialize(uint16_t const baudRate)
{
#ifdef UART_PRINTF

    stdout = &uart1Stream;

#endif // UART_PRINTF

    USART1.BAUD = baudRate;

    USART1.CTRLA = USART_RXCIE_bm;
    USART1.CTRLB = USART_TXEN_bm | USART_RXEN_bm;

    return;
}

static void UART1_Print(char const * string)
{
    char character = '\0';

    while (true)
    {
        character = *string++;

        if (character == '\0')
        {
            break;
        }

        UART1_SendByte((uint8_t) character);
    }

    return;
}

static void UART1_PrintChar(char const character)
{
    UART1_SendByte((char) character);

    return;
}

static inline void UART1_SendByte(uint8_t const dataByte)
{
    while (UART1_TXBusy());

    USART1.TXDATAL = dataByte;

    return;
}

static inline bool UART1_TXBusy(void)
{
    return !(USART1.STATUS & USART_DREIF_bm);
}
