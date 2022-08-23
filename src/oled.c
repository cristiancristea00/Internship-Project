/**
 *  @file oled.c
 *  @author Cristian Cristea - M70957
 *  @date August 18, 2022
 *
 *  @brief Source file for the OLED module
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


#include "oled.h"


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define OLED_SET(SIGNAL)                OLED_SET_PORT_PIN(SIGNAL ## _PORT, SIGNAL ## _PIN)
#define OLED_CLR(SIGNAL)                OLED_CLR_PORT_PIN(SIGNAL ## _PORT, SIGNAL ## _PIN)
#define OLED_OUT(SIGNAL)                OLED_OUT_PORT_PIN(SIGNAL ## _PORT, SIGNAL ## _PIN)

#define OLED_SET_PORT_PIN(PORT, PIN)    (PORT.OUTSET = (1 << (PIN)))
#define OLED_CLR_PORT_PIN(PORT, PIN)    (PORT.OUTCLR = (1 << (PIN)))
#define OLED_OUT_PORT_PIN(PORT, PIN)    (PORT.DIRSET = (1 << (PIN)))

#define OLED_MIN_ADDRESS_BOUND          0
#define OLED_MAX_ADDRESS_BOUND          96



////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum OLED_COMMAND
{
    OLED_SET_COLUMN_ADDRESS               = 0x15,
    OLED_SET_ROW_ADDRESS                  = 0x75,
    OLED_WRITE_RAM                        = 0x5C,
    OLED_READ_RAM                         = 0x5D,
    OLED_SET_REMAP_DUAL_COM_LINE_MODE     = 0xA0,
    OLED_SET_DISPLAY_START_LINE           = 0xA1,
    OLED_SET_DISPLAY_OFFSET               = 0xA2,
    OLED_SET_DISPLAY_MODE_OFF_BLACK       = 0xA4,
    OLED_SET_DISPLAY_MODE_OFF_GS60        = 0xA5,
    OLED_SET_DISPLAY_MODE_ON              = 0xA6,
    OLED_SET_DISPLAY_MODE_INVERSE         = 0xA7,
    OLED_SET_FUNCTION_SELECTION           = 0xAB,
    OLED_SET_SLEEP_MODE_ON                = 0xAE,
    OLED_SET_SLEEP_MODE_OFF               = 0xAF,
    OLED_SET_PHASE_LENGTH                 = 0xB1,
    OLED_DISPLAY_ENHANCEMENT              = 0xB2,
    OLED_SET_FRONT_CLOCK_DIVIDER_OSC_FREQ = 0xB3,
    OLED_SET_GPIO                         = 0xB5,
    OLED_SET_SECOND_PRECHARGE_PERIOD      = 0xB6,
    OLED_GRAY_SCALE_PULSE_WIDTH_LUT       = 0xB8,
    OLED_USE_LINEAR_LUT                   = 0xB9,
    OLED_SET_PRECHARGE_VOLTAGE            = 0xBB,
    OLED_SET_VCOMH_VOLTAGE                = 0xBE,
    OLED_SET_CONTRAST_CURRENT             = 0xC1,
    OLED_MASTER_CONTRAST_CURRENT_CONTROL  = 0xC7,
    OLED_SET_MUX_RATIO                    = 0xCA,
    OLED_SET_COMMAND_LOCK                 = 0xFD
} oled_command_t;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * TODO
 */
static oled_error_code_t OLED_CheckNull(oled_device_t const * const device);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SendWord(oled_device_t const * const device, uint16_t const word);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_InitializePins(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetDataMode(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetCommandMode(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetReadMode(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetWriteMode(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetResetPin(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_ClearResetPin(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetEnablePin(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_ClearEnablePin(void);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_StartTransaction(oled_device_t const * const device);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_EndTransaction(oled_device_t const * const device);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetAddressBoundsWithOffset(oled_device_t const * const device, oled_command_t const command, uint8_t const min, uint8_t const max, uint8_t const offset);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetDisplayOptions(oled_device_t const * const device);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_EnableSleepMode(oled_device_t const * const device);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_DisableSleepMode(oled_device_t const * const device);

/**
 * TODO
 */
static oled_error_code_t OLED_SendCommand(oled_device_t const * const device, oled_command_t const command, uint8_t const * const payload, uint8_t const payloadSize);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

static oled_error_code_t OLED_CheckNull(oled_device_t const * const device)
{
    if ((device == NULL) || (device->spiDevice == NULL))
    {
        LOG_ERROR("Found NULL pointer in OLED check");
        return OLED_NULL_POINTER;
    }
    else
    {
        return OLED_OK;
    }
}

__attribute__((always_inline)) inline static void OLED_SendWord(oled_device_t const * const device, uint16_t const word)
{
    uint8_t const dataBuffer[2] = { (word >> 8), (word & 0x00FF) };

    device->spiDevice->SendData(dataBuffer, 2);
}

__attribute__((always_inline)) inline static void OLED_InitializePins(void)
{
    OLED_OUT(OLED_DATA_COMMAND);
    OLED_CLR(OLED_DATA_COMMAND);

    OLED_OUT(OLED_READ_WRITE);
    OLED_CLR(OLED_READ_WRITE);

    OLED_OUT(OLED_RESET);
    OLED_CLR(OLED_RESET);

    OLED_OUT(OLED_ENABLE);
    OLED_CLR(OLED_ENABLE);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetDataMode(void)
{
    OLED_SET(OLED_DATA_COMMAND);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetCommandMode(void)
{
    OLED_CLR(OLED_DATA_COMMAND);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetReadMode(void)
{
    OLED_SET(OLED_READ_WRITE);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetWriteMode(void)
{
    OLED_CLR(OLED_READ_WRITE);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetResetPin(void)
{
    OLED_SET(OLED_RESET);

    return;
}

__attribute__((always_inline)) inline static void OLED_ClearResetPin(void)
{
    OLED_CLR(OLED_RESET);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetEnablePin(void)
{
    OLED_SET(OLED_ENABLE);

    return;
}

__attribute__((always_inline)) inline static void OLED_ClearEnablePin(void)
{
    OLED_CLR(OLED_ENABLE);

    return;
}

__attribute__((always_inline)) inline static void OLED_StartTransaction(oled_device_t const * const device)
{
    device->spiDevice->ClientSelect(OLED_CHIP_SELECT);

    return;
}

__attribute__((always_inline)) inline static void OLED_EndTransaction(oled_device_t const * const device)
{
    device->spiDevice->ClientDeselect(OLED_CHIP_SELECT);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetAddressBoundsWithOffset(oled_device_t const * const device, oled_command_t const command, uint8_t const min, uint8_t const max, uint8_t const offset)
{
    uint8_t payload[2];

    payload[0] = offset + (min > OLED_MAX_ADDRESS_BOUND ? OLED_MAX_ADDRESS_BOUND : min);
    payload[1] = offset + (max > OLED_MAX_ADDRESS_BOUND ? OLED_MAX_ADDRESS_BOUND : max);

    OLED_SendCommand(device, command, payload, 2);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetDisplayOptions(oled_device_t const * const device)
{
    uint8_t payload = 0;

    /**
     * @brief This option enables the following:
     *        • Horizontal address increment;
     *        • Column address 127 is mapped to SEG0;
     *        • Default colour sequence: A -> B -> C;
     *        • Scan from COM[N - 1] to COM0 where N - multiplex ration;
     *        • Enable COM split odd even;
     *        • 65k colours.
     **/
    payload = 0x32;

    OLED_SendCommand(device, OLED_SET_REMAP_DUAL_COM_LINE_MODE, &payload, 1);

    /**
     * @brief Set vertical scroll by RAM to 32.
     **/
    payload = 0x20;

    OLED_SendCommand(device, OLED_SET_DISPLAY_START_LINE, &payload, 1);

    return;
}

__attribute__((always_inline)) inline static void OLED_EnableSleepMode(oled_device_t const * const device)
{
    OLED_SendCommand(device, OLED_SET_SLEEP_MODE_ON, NULL, 0);

    return;
}

__attribute__((always_inline)) inline static void OLED_DisableSleepMode(oled_device_t const * const device)
{
    OLED_SendCommand(device, OLED_SET_SLEEP_MODE_OFF, NULL, 0);

    return;
}

static oled_error_code_t OLED_SendCommand(oled_device_t const * const device, oled_command_t const command, uint8_t const * const payload, uint8_t const payloadSize)
{
    oled_error_code_t sendCommandResult = OLED_OK;

    sendCommandResult = OLED_CheckNull(device);

    OLED_StartTransaction(device);
    OLED_SetCommandMode();

    uint8_t const commandToSend = (uint8_t) command;
    device->spiDevice->SendData(&commandToSend, 1);

    if (payloadSize > 0)
    {
        if (payload != NULL)
        {
            OLED_SetDataMode();
            device->spiDevice->SendData(payload, payloadSize);
            OLED_SetCommandMode();
        }
        else
        {
            sendCommandResult = OLED_NULL_POINTER;
        }
    }

    OLED_EndTransaction(device);

    return sendCommandResult;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline void OLED_Initialize(oled_device_t * const device, spi_t const * const spiDevice)
{
    OLED_InitializePins();

    spiDevice->Initialize();
    device->spiDevice = spiDevice;

    OLED_ClearEnablePin();
    OLED_SetResetPin();
    OLED_SetWriteMode();
    OLED_ClearResetPin();
    OLED_SetResetPin();
    OLED_SetEnablePin();

    OLED_DisableSleepMode(device);
    OLED_SetColumnAddressBounds(device, OLED_MIN_ADDRESS_BOUND, OLED_MAX_ADDRESS_BOUND);
    OLED_SetRowAddressBounds(device, OLED_MIN_ADDRESS_BOUND, OLED_MAX_ADDRESS_BOUND);
    OLED_SetDisplayOptions(device);

    return;
}

__attribute__((always_inline)) inline void OLED_StartWritingDisplay(oled_device_t const * const device)
{
    OLED_SendCommand(device, OLED_WRITE_RAM, NULL, 0);

    OLED_StartTransaction(device);
    OLED_SetDataMode();

    return;
}

__attribute__((always_inline)) inline void OLED_StopWritingDisplay(oled_device_t const * const device)
{
    OLED_EndTransaction(device);
    OLED_SetCommandMode();

    return;
}

uint16_t OLED_ParseRGBToInteger(oled_colour_t const rgb)
{
    uint8_t const red = rgb.red >> 3;
    uint8_t const green = rgb.green >> 2;
    uint8_t const blue = rgb.blue >> 3;

    uint16_t const mostSignificantByte = ((uint16_t) (red << 3) | (green >> 3));
    uint16_t const leastSignificantByte = (green << 5) | blue;

    return (mostSignificantByte << 8) | leastSignificantByte;
}

oled_colour_t OLED_ParseIntegerToRGB(uint16_t const rawData)
{
    uint8_t const mostSignificantByte = rawData >> 8;
    uint8_t const leastSignificantByte = rawData & 0x00FF;

    oled_colour_t parsedColour = {
        .red = mostSignificantByte >> 3,
        .green = ((mostSignificantByte & 0x07) << 3) | (leastSignificantByte >> 5),
        .blue = leastSignificantByte & 0x1F
    };

    parsedColour.red <<= 3;
    parsedColour.green <<= 2;
    parsedColour.blue <<= 3;

    return parsedColour;
}

__attribute__((always_inline)) inline void OLED_SendColor(oled_device_t const * const device, oled_colour_t const colour)
{
    OLED_SendWord(device, OLED_ParseRGBToInteger(colour));

    return;
}

__attribute__((always_inline)) inline void OLED_SetRowAddressBounds(oled_device_t const * const device, uint8_t const min, uint8_t const max)
{
    OLED_SetAddressBoundsWithOffset(device, OLED_SET_ROW_ADDRESS, min, max, 0);

    return;
}

__attribute__((always_inline)) inline void OLED_SetColumnAddressBounds(oled_device_t const * const device, uint8_t const min, uint8_t const max)
{
    OLED_SetAddressBoundsWithOffset(device, OLED_SET_COLUMN_ADDRESS, min, max, 16);

    return;
}

void OLED_SetBackground(oled_device_t const * const device, oled_colour_t const colour)
{
    OLED_SetColumnAddressBounds(device, OLED_MIN_ADDRESS_BOUND, OLED_MAX_ADDRESS_BOUND);
    OLED_SetRowAddressBounds(device, OLED_MIN_ADDRESS_BOUND, OLED_MAX_ADDRESS_BOUND);

    OLED_StartWritingDisplay(device);

    for (uint8_t x = 0; x <= OLED_MAX_ADDRESS_BOUND; ++x)
    {
        for (uint8_t y = 0; y <= OLED_MAX_ADDRESS_BOUND; ++y)
        {
            OLED_SendColor(device, colour);
        }
    }

    OLED_StopWritingDisplay(device);

    return;
}