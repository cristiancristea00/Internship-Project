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

#define OLED_SET_PORT_PIN(PORT, PIN)    (PORT.OUTSET = (1 << (PIN)))
#define OLED_CLR_PORT_PIN(PORT, PIN)    (PORT.OUTCLR = (1 << (PIN)))

#define OLED_MAX_ADDRESS_BOUND 95


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum OLED_COMMAND
{
    OLED_CMD_SET_COLUMN_ADDRESS               = 0x15,
    OLED_CMD_SET_ROW_ADDRESS                  = 0x75,
    OLED_CMD_WRITE_RAM                        = 0x5C,
    OLED_CMD_READ_RAM                         = 0x5D,
    OLED_CMD_SET_REMAP_DUAL_COM_LINE_MODE     = 0xA0,
    OLED_CMD_SET_DISPLAY_START_LINE           = 0xA1,
    OLED_CMD_SET_DISPLAY_OFFSET               = 0xA2,
    OLED_CMD_SET_DISPLAY_MODE_OFF_BLACK       = 0xA4,
    OLED_CMD_SET_DISPLAY_MODE_OFF_GS60        = 0xA5,
    OLED_CMD_SET_DISPLAY_MODE_ON              = 0xA6,
    OLED_CMD_SET_DISPLAY_MODE_INVERSE         = 0xA7,
    OLED_CMD_SET_FUNCTION_SELECTION           = 0xAB,
    OLED_CMD_SET_SLEEP_MODE_ON                = 0xAE,
    OLED_CMD_SET_SLEEP_MODE_OFF               = 0xAF,
    OLED_CMD_SET_PHASE_LENGTH                 = 0xB1,
    OLED_CMD_DISPLAY_ENHANCEMENT              = 0xB2,
    OLED_CMD_SET_FRONT_CLOCK_DIVIDER_OSC_FREQ = 0xB3,
    OLED_CMD_SET_GPIO                         = 0xB5,
    OLED_CMD_SET_SECOND_PRECHARGE_PERIOD      = 0xB6,
    OLED_CMD_GRAY_SCALE_PULSE_WIDTH_LUT       = 0xB8,
    OLED_CMD_USE_LINEAR_LUT                   = 0xB9,
    OLED_CMD_SET_PRECHARGE_VOLTAGE            = 0xBB,
    OLED_CMD_SET_VCOMH_VOLTAGE                = 0xBE,
    OLED_CMD_SET_CONTRAST_CURRENT             = 0xC1,
    OLED_CMD_MASTER_CONTRAST_CURRENT_CONTROL  = 0xC7,
    OLED_CMD_SET_MUX_RATIO                    = 0xCA,
    OLED_CMD_SET_COMMAND_LOCK                 = 0xFD
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
__attribute__((always_inline)) inline static void OLED_SetDataMode(void);


/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetCommandMode(void);

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
__attribute__((always_inline)) inline static void OLED_SetRowAddressBounds(oled_device_t const * const device, uint8_t const min, uint8_t const max);

/**
 * TODO
 */
__attribute__((always_inline)) inline static void OLED_SetColumnAddressBounds(oled_device_t const * const device, uint8_t const min, uint8_t const max);

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

__attribute__((always_inline)) inline static void OLED_SetDataMode(void)
{
    OLED_SET(OLED_DATA_CMD);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetCommandMode(void)
{
    OLED_CLR(OLED_DATA_CMD);

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

__attribute__((always_inline)) inline static void OLED_SetRowAddressBounds(oled_device_t const * const device, uint8_t const min, uint8_t const max)
{
    uint8_t const payload[2] = {
        (min > OLED_MAX_ADDRESS_BOUND ? OLED_MAX_ADDRESS_BOUND : min),
        (max > OLED_MAX_ADDRESS_BOUND ? OLED_MAX_ADDRESS_BOUND : max)
    };

    OLED_SendCommand(device, OLED_CMD_SET_ROW_ADDRESS, payload, 2);

    return;
}

__attribute__((always_inline)) inline static void OLED_SetColumnAddressBounds(oled_device_t const * const device, uint8_t const min, uint8_t const max)
{
    // TODO
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