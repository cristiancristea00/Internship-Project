/**
 *  @file uart.c
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


#include "uart.h"

static uart_callback_t uartCallback = NULL;

#ifdef UART_PRINTF

static int8_t Uart1PrintChar(char const character, FILE * const stream)
{
    Uart1SendByte((uint8_t) character);

    return 0;
}

FILE uart1Stream = FDEV_SETUP_STREAM(Uart1PrintChar, NULL, _FDEV_SETUP_WRITE);

#endif // UART_PRINTF

void Uart1Init(uint16_t const baudRate)
{
#ifdef UART_PRINTF

    stdout = &uart1Stream;

#endif // UART_PRINTF

    uartCallback = NULL;

    USART1.BAUD = baudRate;

    USART1.CTRLA = USART_RXCIE_bm;
    USART1.CTRLB = USART_TXEN_bm | USART_RXEN_bm;

    return;
}

void Uart1RegisterCallback(uart_callback_t const callback)
{
    uartCallback = callback;

    return;
}

void Uart1Print(char const * string)
{
    char character = '\0';

    while (1)
    {
        character = *string++;

        if (character == '\0')
        {
            break;
        }

        Uart1SendByte((uint8_t) character);
    }

    return;
}

static inline void Uart1SendByte(uint8_t const dataByte)
{
    while (Uart1TxBusy());

    USART1.TXDATAL = dataByte;

    return;
}

static inline bool Uart1TxBusy(void)
{
    return !(USART1.STATUS & USART_DREIF_bm);
}