/**
 *  @file oled.h
 *  @author Cristian Cristea - M70957
 *  @date August 18, 2022
 *
 *  @brief Header file for the OLED module
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


#ifndef OLED_H
#define	OLED_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "config.h"
#include "spi.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define OLED_CHIP_SELECT          SPI_CS1

#define OLED_DATA_COMMAND_PORT    PORTD
#define OLED_DATA_COMMAND_PIN     PIN0

#define OLED_RESET_PORT           PORTD
#define OLED_RESET_PIN            PIN7

#define OLED_ENABLE_PORT          PORTD
#define OLED_ENABLE_PIN           PIN6

#define OLED_READ_WRITE_PORT      PORTD
#define OLED_READ_WRITE_PIN       PIN3

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum OLED_ERROR_CODE
{
    OLED_OK              = 0x00,
    OLED_NULL_POINTER    = 0x01,
} oled_error_code_t;

typedef struct OLED_DEVICE
{
    // SPI device
    spi_t const * spiDevice;
} oled_device_t;

typedef struct OLED_COLOUR
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} oled_colour_t;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * TODO
 */
void OLED_Initialize(oled_device_t * const device, spi_t const * const spiDevice);


/**
 * TODO
 */
void OLED_SetBackground(oled_device_t const * const device, oled_colour_t const colour);
#endif // OLED_H

