/**
 *  @file oled-draw.c
 *  @author Cristian Cristea - M70957
 *  @date August 19, 2022
 *
 *  @brief Source file for the OLED drawing module
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


#include "oled-draw.h"


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define OLED_DRAW_MAX_DIMENSION    95


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * TODO
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Point(oled_device_t const * const device, oled_point_t const point, oled_colour_t const colour);

/**
 * TODO
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Line(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, uint8_t const width, oled_colour_t const colour);

/**
 * TODO
 **/
__attribute__((always_inline)) inline void OLED_DRAW_Rectangle(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, uint8_t const width, oled_colour_t const colour);

/**
 * TODO
 **/
__attribute__((always_inline)) inline void OLED_DRAW_FilledRectangle(oled_device_t const * const device, oled_point_t const startPoint, oled_point_t const endPoint, oled_colour_t const colour);

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline void OLED_DRAW_Point(oled_device_t const * const device, oled_point_t const point, oled_colour_t const colour)
{
    if (point.x > OLED_DRAW_MAX_DIMENSION || point.y > OLED_DRAW_MAX_DIMENSION)
    {
        return;
    }

    OLED_SetColumnAddressBounds(device, point.x, point.x);
    OLED_SetRowAddressBounds(device, point.y, point.y);

    OLED_StartWritingDisplay(device);
    OLED_SendColor(device, colour);
    OLED_StopWritingDisplay(device);

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_Line(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, uint8_t const width, oled_colour_t const colour)
{
    // TODO
}

__attribute__((always_inline)) inline void OLED_DRAW_Rectangle(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, uint8_t const width, oled_colour_t const colour)
{
    oled_point_t const topLeft     = { start.x, start.y };
    oled_point_t const topRight    = { end.x, start.y };
    oled_point_t const bottomLeft  = { start.x, end.y };
    oled_point_t const bottomRight = { end.x, end.y };

    OLED_DRAW_Line(device, topLeft, topRight, width, colour);
    OLED_DRAW_Line(device, topRight, bottomRight, width, colour);
    OLED_DRAW_Line(device, bottomRight, bottomLeft, width, colour);
    OLED_DRAW_Line(device, bottomLeft, topLeft, width, colour);

    return;
}

__attribute__((always_inline)) inline void OLED_DRAW_FilledRectangle(oled_device_t const * const device, oled_point_t const start, oled_point_t const end, oled_colour_t const colour)
{
    OLED_SetColumnAddressBounds(device, start.x, end.x);
    OLED_SetRowAddressBounds(device, start.y, end.y);

    OLED_StartWritingDisplay(device);

    for (uint8_t x = start.x; x <= end.x; ++x)
    {
        for (uint8_t y = start.y; y <= end.y; ++y)
        {
            OLED_SendColor(device, colour);
        }
    }

    OLED_StopWritingDisplay(device);

    return;
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void OLED_SetShape(oled_shape_t * const shape, oled_shape_type_t const type, oled_shape_parameters_t const * const parameters, oled_colour_t const colour)
{
    shape->type = type;
    shape->colour = OLED_ParseRGBToInteger(colour);

    switch (type)
    {
        case OLED_SHAPE_POINT:
            shape->parameters.point = parameters->point;
            shape->Draw = OLED_DrawPoint;
            break;
        case OLED_SHAPE_LINE:
            shape->parameters.line = parameters->line;
            shape->Draw = OLED_DrawLine;
            break;
        case OLED_SHAPE_RECTANGLE:
            shape->parameters.rectangle = parameters->rectangle;
            shape->Draw = OLED_DrawRectangle;
            break;
        case OLED_SHAPE_FILLED_RECTANGLE:
            shape->parameters.filled_rectangle = parameters->filled_rectangle;
            shape->Draw = OLED_DrawFilledRectangle;
            break;
        default:
            LOG_ERROR("Invalid shape in shape creation");
            break;
    }

    return;
}

void OLED_DrawPoint(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Point(device, shape->parameters.point.point, OLED_ParseIntegerToRGB(shape->colour));

    return;
}

void OLED_DrawLine(oled_device_t const * const device, oled_shape_t const * const shape)
{
    // TODO
}

void OLED_DrawRectangle(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_Rectangle(device, shape->parameters.rectangle.start, shape->parameters.rectangle.end, shape->parameters.rectangle.width, OLED_ParseIntegerToRGB(shape->colour));
}

void OLED_DrawFilledRectangle(oled_device_t const * const device, oled_shape_t const * const shape)
{
    OLED_DRAW_FilledRectangle(device, shape->parameters.filled_rectangle.start, shape->parameters.filled_rectangle.end, OLED_ParseIntegerToRGB(shape->colour));

    return;
}