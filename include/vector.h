/**
 *  @file vector.h
 *  @author Cristian Cristea - M70957
 *  @date 11 August 2022
 *
 *  @brief Header file for the Vector module
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


#ifndef VECTOR_H
#define	VECTOR_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "config.h"

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define VECTOR_MAX_BUFFER_SIZE    64


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef struct VECTOR
{
    // Internal buffer
    uint8_t internalBuffer[VECTOR_MAX_BUFFER_SIZE];

    // Current buffer size
    uint8_t bufferSize;
} vector_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Sets the internal buffer to zeros.
 *
 * @param[in, out] vector The vector to initialized
 **/
void Vector_Initialize(vector_t * const vector);

/**
 * @brief Clears the internal buffer.
 *
 * @param vector The vector to be cleared
 **/
void Vector_Clear(vector_t * const vector);

/**
 * @brief Adds a byte to the vector.
 *
 * @param[in, out] vector The vector to be added to
 * @param[in] byteToAdd The byte to be added
 **/
void Vector_AddByte(vector_t * const vector, uint8_t const byteToAdd);

/**
 * @brief Adds a word (2 bytes) to the vector.
 *
 * @param[in, out] vector The vector to be added to
 * @param[in] wordToAdd The word to be added
 **/
void Vector_AddWord(vector_t * const vector, uint16_t const wordToAdd);

/**
 * @brief Adds a double word (4 bytes) to the vector.
 *
 * @param[in, out] vector The vector to be added to
 * @param[in] doubleWordToAdd The double word to be added
 **/
void Vector_AddDoubleWord(vector_t * const vector, uint32_t const doubleWordToAdd);

/**
 * @brief Adds a quad word (8 bytes) to the vector.
 *
 * @param[in, out] vector The vector to be added to
 * @param[in] quadWordToAdd The quad word to be added
 **/
void Vector_AddQuadWord(vector_t * const vector, uint64_t const quadWordToAdd);

/**
 * @brief Removes a byte from the vector and returns it.
 *
 * @param[in, out] vector The vector to be removed from
 *
 * @return uint8_t The byte removed from the vector
 **/
uint8_t Vector_RemoveByte(vector_t * const vector);

/**
 * @brief Removes a word (2 bytes) from the vector and returns it.
 *
 * @param[in, out] vector The vector to be removed from
 *
 * @return uint16_t The word removed from the vector
 **/
uint16_t Vector_RemoveWord(vector_t * const vector);

/**
 * @brief Removes a double word (4 bytes) from the vector and returns it.
 *
 * @param[in, out] vector The vector to be removed from
 *
 * @return uint32_t The double word removed from the vector
 **/
uint32_t Vector_RemoveDoubleWord(vector_t * const vector);

/**
 * @brief Removes a quad word (8 bytes) from the vector and returns it.
 *
 * @param[in, out] vector The vector to be removed from
 *
 * @return uint64_t The quad word removed from the vector
 **/
uint64_t Vector_RemoveQuadWord(vector_t * const vector);

/* 
 * TODO 
 */
uint8_t Vector_FirstByte(vector_t const * const vector);

/* 
 * TODO 
 */
uint8_t Vector_LastByte(vector_t const * const vector);

// TODO: First function for the rest
// TODO: Last function for the rest

#endif // VECTOR_H
