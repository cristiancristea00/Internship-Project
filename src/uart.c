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


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Macro to convert UART baud rate to the value to be used in the BAUD
 *        register of the USART.
 *
 **/
#define UART_BAUD_RATE(x) ((uint16_t) ((4UL * F_CPU) / (x)))


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             *
 *                                    UART0                                    *
 *                                                                             *
 *******************************************************************************/

/**
 * @brief Initialize the UART0 module by setting the baud rate and enabling the
 *        transmitter and receiver. The UART0 module is configured for 8-bit
 *        with no parity and 1 stop bit. The Receive Complete Interrupt and
 *        Global interrupts are enabled.
 *
 * @param[in] baudRate The baud rate
 * @param[in] receiveCallback The callback function to be called when a byte is
 *                            received
 **/
__attribute__((always_inline)) inline static void UART0_InitializeWithReceive(uint32_t const baudRate, uart_callback_t const receiveCallback);

/**
 * @brief Initialize the UART0 module by setting the baud rate and enabling the
 *        transmitter. The UART0 module is configured for 8-bit with no parity
 *        and 1 stop bit.
 *
 * @param[in] baudRate The baud rate
 **/
__attribute__((always_inline)) inline static void UART0_Initialize(uint32_t const baudRate);

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
__attribute__((always_inline)) inline static void UART0_PrintChar(char const character);

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
__attribute__((always_inline)) inline static bool UART0_TXBusy(void);

/**
 * @brief Registers a callback function to be called when a byte is received
 *        over UART0.
 *
 * @param[in] callback The callback function to be registered
 **/
__attribute__((always_inline)) inline static void UART0_RegisterCallback(uart_callback_t const callback);

/*******************************************************************************
 *                                                                             *
 *                                    UART1                                    *
 *                                                                             *
 *******************************************************************************/

/**
 * @brief Initialize the UART1 module by setting the baud rate and enabling the
 *        transmitter and receiver. The UART1 module is configured for 8-bit
 *        with no parity and 1 stop bit. The Receive Complete Interrupt is
 *        enabled.
 *
 * @param[in] baudRate The baud rate
 * @param[in] receiveCallback The callback function to be called when a byte is
 *                        received
 **/
__attribute__((always_inline)) inline static void UART1_InitializeWithReceive(uint32_t const baudRate, uart_callback_t const receiveCallback);

/**
 * @brief Initialize the UART1 module by setting the baud rate and enabling the
 *        transmitter. The UART1 module is configured for 8-bit with no parity
 *        and 1 stop bit.
 *
 * @param[in] baudRate The baud rate
 **/
__attribute__((always_inline)) inline static void UART1_Initialize(uint32_t const baudRate);

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
__attribute__((always_inline)) inline static void UART1_PrintChar(char const character);

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
__attribute__((always_inline)) inline static bool UART1_TXBusy(void);

/**
 * @brief Registers a callback function to be called when a byte is received
 *        over UART1.
 *
 * @param[in] callback The callback function to be registered
 **/
__attribute__((always_inline)) inline static void UART1_RegisterCallback(uart_callback_t const callback);

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
__attribute__((always_inline)) inline static int8_t UART1_SendChar(char const character, FILE * const stream);

#endif // UART_PRINTF


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             *
 *                                    UART0                                    *
 *                                                                             *
 *******************************************************************************/

static uart_callback_t uart_0_callback = NULL;

__attribute__((always_inline)) inline static void UART0_InitializeWithReceive(uint32_t const baudRate, uart_callback_t const receiveCallback)
{
    UART0_RegisterCallback(receiveCallback);

    UART0_Initialize(baudRate);

    PORTA.DIRCLR = PIN1_bm;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        USART0.CTRLA = USART_RXCIE_bm;
    }

    USART0.CTRLB |= USART_RXEN_bm;

    sei();

    return;
}

__attribute__((always_inline)) inline static void UART0_Initialize(uint32_t const baudRate)
{
    PORTA.DIRSET = PIN0_bm;

    PORTA.OUTSET = PIN0_bm;

    USART0.BAUD = UART_BAUD_RATE(baudRate);

    USART0.CTRLB = USART_TXEN_bm;

    return;
}

static void UART0_Print(char const * const string)
{
    char character = '\0';
    char const * currentStringPosition = string;

    while (true)
    {
        character = *currentStringPosition++;

        if (character == '\0')
        {
            break;
        }

        UART0_PrintChar(character);
    }

    return;
}

__attribute__((always_inline)) inline static void UART0_PrintChar(char const character)
{
    UART0_SendByte((char) character);

    return;
}

static void UART0_SendData(uint8_t const * const buffer, uint8_t const bufferSize)
{
    for (uint8_t bufferPosition = 0; bufferPosition < bufferSize; ++bufferPosition)
    {
        UART0_SendByte(buffer[bufferPosition]);
    }

    return;
}

static void UART0_SendByte(uint8_t const dataByte)
{
    while (UART0_TXBusy())
    {
        TightLoopContents();
    }

    USART0.TXDATAL = dataByte;

    return;
}

__attribute__((always_inline)) inline static inline bool UART0_TXBusy(void)
{
    return !(USART0.STATUS & USART_DREIF_bm);
}

__attribute__((always_inline)) inline static void UART0_RegisterCallback(uart_callback_t const callback)
{
    uart_0_callback = callback;

    return;
}


/*******************************************************************************
 *                                                                             *
 *                                    UART1                                    *
 *                                                                             *
 *******************************************************************************/

static uart_callback_t uart_1_callback = NULL;

__attribute__((always_inline)) inline static void UART1_InitializeWithReceive(uint32_t const baudRate, uart_callback_t const receiveCallback)
{
    UART1_RegisterCallback(receiveCallback);

    UART1_Initialize(baudRate);

    PORTC.DIRCLR = PIN1_bm;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        USART1.CTRLA = USART_RXCIE_bm;
    }

    USART1.CTRLB |= USART_RXEN_bm;

    return;
}

#ifdef UART_PRINTF

__attribute__((always_inline)) inline static int8_t UART1_SendChar(char const character, __attribute__((unused)) FILE * const stream)
{
    UART1_PrintChar(character);

    return 0;
}

static FILE uart_1_stream = FDEV_SETUP_STREAM(UART1_SendChar, NULL, _FDEV_SETUP_WRITE);

#endif // UART_PRINTF

__attribute__((always_inline)) inline static void UART1_Initialize(uint32_t const baudRate)
{
#ifdef UART_PRINTF

    stdout = &uart_1_stream;

#endif // UART_PRINTF

    PORTC.DIRSET = PIN0_bm;

    PORTC.OUTSET = PIN0_bm;

    USART1.BAUD = UART_BAUD_RATE(baudRate);

    USART1.CTRLB = USART_TXEN_bm;

    return;
}

static void UART1_Print(char const * const string)
{
    char character = '\0';
    char const * currentStringPosition = string;

    while (true)
    {
        character = *currentStringPosition++;

        if (character == '\0')
        {
            break;
        }

        UART1_PrintChar(character);
    }

    return;
}

__attribute__((always_inline)) inline static void UART1_PrintChar(char const character)
{
    UART1_SendByte((char) character);

    return;
}

static void UART1_SendData(uint8_t const * const buffer, uint8_t const bufferSize)
{
    for (uint8_t bufferPosition = 0; bufferPosition < bufferSize; ++bufferPosition)
    {
        UART1_SendByte(buffer[bufferPosition]);
    }

    return;
}

static void UART1_SendByte(uint8_t const dataByte)
{
    while (UART1_TXBusy())
    {
        TightLoopContents();
    }

    USART1.TXDATAL = dataByte;

    return;
}

__attribute__((always_inline)) inline static inline bool UART1_TXBusy(void)
{
    return !(USART1.STATUS & USART_DREIF_bm);
}

__attribute__((always_inline)) inline static void UART1_RegisterCallback(uart_callback_t const callback)
{
    uart_1_callback = callback;

    return;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                     Interrupt Service Routines (ISRs)                      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             *
 *                                    UART0                                    *
 *                                                                             *
 *******************************************************************************/

ISR(USART0_RXC_vect)
{
    uint8_t byte = USART0.RXDATAL;

    if (uart_0_callback != NULL)
    {
        uart_0_callback(byte);
    }
}

/*******************************************************************************
 *                                                                             *
 *                                    UART1                                    *
 *                                                                             *
 *******************************************************************************/

ISR(USART1_RXC_vect)
{
    uint8_t byte = USART1.RXDATAL;

    if (uart_1_callback != NULL)
    {
        uart_1_callback(byte);
    }
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             *
 *                                    UART0                                    *
 *                                                                             *
 *******************************************************************************/

/**
 * @brief  Module for UART0
 **/
uart_t const uart_0 = {
    .Initialize = UART0_Initialize,
    .InitializeWithReceive = UART0_InitializeWithReceive,
    .SendByte = UART0_SendByte,
    .SendData = UART0_SendData,
    .PrintChar = UART0_PrintChar,
    .Print = UART0_Print,
    .RegisterCallback = UART0_RegisterCallback
};

/*******************************************************************************
 *                                                                             *
 *                                    UART1                                    *
 *                                                                             *
 *******************************************************************************/

/**
 * @brief  Module for UART1
 **/
uart_t const uart_1 = {
    .Initialize = UART1_Initialize,
    .InitializeWithReceive = UART1_InitializeWithReceive,
    .SendByte = UART1_SendByte,
    .SendData = UART1_SendData,
    .PrintChar = UART1_PrintChar,
    .Print = UART1_Print,
    .RegisterCallback = UART1_RegisterCallback
};
