/**
 *  @file utils.c
 *  @author Cristian Cristea - M70957
 *  @date 30 August 2022
 *
 *  @brief Source file for utils functions for the Sensors station
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


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "utils.h"

#include "oled-draw.h"
#include "uart.h"

#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define DISPLAY_SPACING    5
#define MAX_ROW_DISPLAY    20


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Writes on the OLED the temperature, pressure and humidity initial text.
 *
 * @param[in] oled The OLEDC module to use
 **/
__attribute__((always_inline)) inline static void DisplaySetup(oled_device_t const * const oled);

/**
 * @brief Clears the OLED current line of text.
 *
 * @param[in] oled The OLED module to use
 * @param[in] start The start position of the line to clear
 **/
__attribute__((always_inline)) inline static void ClearLineOfText(oled_device_t const * const oled, oled_point_t const start);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

static oled_colour_t const black = { 0x00, 0x00, 0x00 };
static oled_colour_t const white = { 0xFF, 0xFF, 0xFF };

__attribute__((always_inline)) inline void DisplaySetup(oled_device_t const * const oled)
{
    oled_shape_parameters_t textParameters;
    oled_shape_t text;

    textParameters.string.scale_x = 1;
    textParameters.string.scale_y = 1;

    oled_point_t textStart;

    textStart.x = 0;
    textStart.y = 0;
    textParameters.string.start = textStart;
    textParameters.string.data = (uint8_t *) "Temperature:";
    OLED_SetShape(&text, OLED_SHAPE_STRING, &textParameters, white);
    text.Draw(oled, &text);

    textStart.x = 0;
    textStart.y = 2 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING);
    textParameters.string.start = textStart;
    textParameters.string.data = (uint8_t *) "Pressure:";
    OLED_SetShape(&text, OLED_SHAPE_STRING, &textParameters, white);
    text.Draw(oled, &text);

    textStart.x = 0;
    textStart.y = 4 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING);
    textParameters.string.start = textStart;
    textParameters.string.data = (uint8_t *) "Humidity:";
    OLED_SetShape(&text, OLED_SHAPE_STRING, &textParameters, white);
    text.Draw(oled, &text);

    return;
}

__attribute__((always_inline)) inline void ClearLineOfText(oled_device_t const * const oled, oled_point_t const start)
{
    oled_shape_parameters_t clearBoxParameters;
    oled_shape_t clearBox;

    oled_point_t const end = { start.x + MAX_ROW_DISPLAY * OLED_DRAW_FONT_WIDTH, start.y + OLED_DRAW_FONT_HEIGHT };

    clearBoxParameters.filled_rectangle.start = start;
    clearBoxParameters.filled_rectangle.end   = end;

    OLED_SetShape(&clearBox, OLED_SHAPE_FILLED_RECTANGLE, &clearBoxParameters, black);
    clearBox.Draw(oled, &clearBox);

    return;
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline void SystemInitialize(void)
{
    SetClockFrequency(CLKCTRL_FRQSEL_24M_gc);

    uart_1.Initialize(460800);
    uart_0.InitializeWithReceive(460800, HC05_ReceiveCallback);
    spi_0.Initialize();

    return;
}

__attribute__((always_inline)) inline void ModulesInitialize(oled_device_t * const oled , hc05_device_t * const bluetooth)
{
    OLED_Initialize(oled, &spi_0);
    OLED_SetBackground(oled, black);
    DisplaySetup(oled);

    HC05_Initialize(bluetooth, &uart_0);

    return;
}

__attribute__((always_inline)) inline void DisplayData(bme280_data_t const * const sensorsData, oled_device_t const * const oled)
{
    char temperatureBuffer[MAX_ROW_DISPLAY];
    double const temperature = BME280_GetDisplayTemperature(sensorsData);
    sprintf(temperatureBuffer, "%0.2lf degrees C", temperature);

    char pressureBuffer[MAX_ROW_DISPLAY];
    double const pressure = BME280_GetDisplayPressure(sensorsData);
    sprintf(pressureBuffer, "%0.2lf hPa", pressure);

    char humidityBuffer[MAX_ROW_DISPLAY];
    double const humidity = BME280_GetDisplayHumidity(sensorsData);
    sprintf(humidityBuffer, "%0.2lf %c", humidity, '%');


    oled_point_t temperatureTextStart = {
        .x = 0,
        .y = 1 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING)
    };
    oled_point_t pressureTextStart = {
        .x = 0,
        .y = 3 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING)
    };
    oled_point_t humidityTextStart = {
        .x = 0,
        .y = 5 * (OLED_DRAW_FONT_HEIGHT + DISPLAY_SPACING)
    };

    oled_shape_parameters_t temperaturetTextParameters;
    temperaturetTextParameters.string.scale_x = 1;
    temperaturetTextParameters.string.scale_y = 1;
    temperaturetTextParameters.string.start = temperatureTextStart;
    temperaturetTextParameters.string.data = (uint8_t *) temperatureBuffer;
    oled_shape_t temperatureText;
    OLED_SetShape(&temperatureText, OLED_SHAPE_STRING, &temperaturetTextParameters, white);

    oled_shape_parameters_t pressureTextParameters;
    pressureTextParameters.string.scale_x = 1;
    pressureTextParameters.string.scale_y = 1;
    pressureTextParameters.string.start = pressureTextStart;
    pressureTextParameters.string.data = (uint8_t *) pressureBuffer;
    oled_shape_t pressureText;
    OLED_SetShape(&pressureText, OLED_SHAPE_STRING, &pressureTextParameters, white);

    oled_shape_parameters_t humidityTextParameters;
    humidityTextParameters.string.scale_x = 1;
    humidityTextParameters.string.scale_y = 1;
    humidityTextParameters.string.start = humidityTextStart;
    humidityTextParameters.string.data = (uint8_t *) humidityBuffer;
    oled_shape_t humidityText;
    OLED_SetShape(&humidityText, OLED_SHAPE_STRING, &humidityTextParameters, white);

    ClearLineOfText(oled, temperatureTextStart);
    temperatureText.Draw(oled, &temperatureText);

    ClearLineOfText(oled, pressureTextStart);
    pressureText.Draw(oled, &pressureText);

    ClearLineOfText(oled, humidityTextStart);
    humidityText.Draw(oled, &humidityText);

    printf("Temperature: %0.2lf °C\n\r", temperature);
    printf("Pressure: %0.2lf hPa\n\r", pressure);
    printf("Relative humidity: %0.2lf %c\n\r", humidity, '%');

    return;
}
