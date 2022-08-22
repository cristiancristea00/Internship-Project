/**
 *  @file oled-draw.h
 *  @author Cristian Cristea - M70957
 *  @date August 19, 2022
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


#ifndef OLED_DRAW_H
#define	OLED_DRAW_H

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "oled.h"

#include <stdlib.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum OLED_SHAPE_TYPE
{
    OLED_SHAPE_POINT            = 0x00,
    OLED_SHAPE_LINE             = 0x01,
    OLED_SHAPE_RECTANGLE        = 0x02,
    OLED_SHAPE_FILLED_RECTANGLE = 0x03,
	OLED_SHAPE_DISC             = 0x04,
	OLED_SHAPE_CIRCLE           = 0x05,
 	OLED_SHAPE_BITMAP           = 0x06,
	OLED_SHAPE_CHARACTER        = 0x07,
	OLED_SHAPE_STRING           = 0x08,
} oled_shape_type_t;

typedef struct OLED_POINT
{
    uint8_t x;
    uint8_t y;
} oled_point_t;

typedef union OLED_SHAPE_PARAMETERS
{
    struct {
        oled_point_t point;
    } point;
    
    struct {
        oled_point_t start;
        oled_point_t end;
        uint8_t width;
    } line;
    
    struct {
        oled_point_t start;
        oled_point_t end;
        uint8_t width;
    } rectangle;
    
    struct {
        oled_point_t start;
        oled_point_t end;
    } filled_rectangle;
    
    struct {
        oled_point_t center;
        uint8_t radius;
        uint8_t width;
    } circle;
    
    struct {
        oled_point_t center;
        uint8_t radius;
    } disc;
    
    struct {
        oled_point_t start;
        uint8_t size_x;
        uint8_t size_y;
        oled_colour_t const * data;
    } bitmap;
    
    struct {
        oled_point_t start;
        uint8_t scale_x;
        uint8_t scale_y;
        uint8_t character;
    } character;
    
    struct {
        oled_point_t start;
        uint8_t scale_x;
        uint8_t scale_y;
        uint8_t const * data;
    } string;
} oled_shape_parameters_t;

typedef struct OLED_SHAPE
{
    uint16_t colour;
    oled_shape_type_t type;
    oled_shape_parameters_t parameters;
    void (* Draw) (oled_device_t const * const, struct OLED_SHAPE *);
} oled_shape_t;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * TODO
 */
void OLED_SetShape(oled_shape_t * const shape, oled_shape_type_t const type, oled_shape_parameters_t const * const parameters, oled_colour_t const colour);


#endif // OLED_DRAW_H

