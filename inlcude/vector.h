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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAX_SIZE 16

#define VECTOR_BYTES_BYTES          1
#define VECTOR_WORD_BYTES           2
#define VECTOR_DOUBLE_WORD_BYTES    4
#define VECTOR_QUAD_WORD_BYTES      8

typedef struct VECTOR
{
    // Internal buffer
    uint8_t internalBuffer[MAX_SIZE];

    // Current buffer size
    uint8_t bufferSize;
} vector_t;

typedef union WORD
{
    uint16_t value;
    struct
    {
        uint8_t byte0;
        uint8_t byte1;
    } bytes;
} word_t;

typedef union DOUBLE_WORD
{
    uint32_t value;
    struct
    {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
    } bytes;
} double_word_t;

typedef union QUAD_WORD
{
    uint64_t value;
    struct
    {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
    } bytes;
} quad_word_t;

#endif // VECTOR_H

/* TODO */
void Vector_Initialize(vector_t * const vector);

/* TODO */
static bool Vector_IsSpaceAvailable(vector_t * const vector, uint8_t const numberBytes);

/* TODO */
void Vector_AddByte(vector_t * const vector, uint8_t const byteToAdd);

/* TODO */
void Vector_AddWord(vector_t * const vector, uint16_t const wordToAdd);

/* TODO */
void Vector_AddDoubleWord(vector_t * const vector, uint32_t const doubleWordToAdd);

/* TODO */
void Vector_AddQuadWord(vector_t * const vector, uint64_t const quadWordToAdd);

/* TODO */
uint8_t Vector_RemoveByte(vector_t * const vector);

/* TODO */
uint16_t Vector_RemoveWord(vector_t * const vector);

/* TODO */
uint32_t Vector_RemoveDoubleWord(vector_t * const vector);

/* TODO */
uint64_t Vector_RemoveQuadWord(vector_t * const vector);