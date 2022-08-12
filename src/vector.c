/**
 *  @file vector.c
 *  @author Cristian Cristea - M70957
 *  @date 11 August 2022
 *
 *  @brief Source file for the Vector module
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


#include "vector.h"

void Vector_Initialize(vector_t * const vector)
{
    memset(vector->internalBuffer, 0, MAX_BUFFER_SIZE);

    vector->bufferSize = 0;

    return;
}

static bool Vector_IsSpaceAvailable(vector_t * const vector, uint8_t const numberBytes)
{
    if ((vector->bufferSize + numberBytes) <= MAX_BUFFER_SIZE)
    {
        return true;
    }
    else
    {
        LOG_ERROR_PRINTF("Failed to add %d bytes to vector", numberBytes);
        return false;
    }
}

void Vector_Clear(vector_t * const vector)
{
    vector->bufferSize = 0;

    return;
}

void Vector_AddByte(vector_t * const vector, uint8_t const byteToAdd)
{
    if (!Vector_IsSpaceAvailable(vector, VECTOR_BYTES_BYTES))
    {
        return;
    }

    vector->internalBuffer[vector->bufferSize] = byteToAdd;

    vector->bufferSize += VECTOR_BYTES_BYTES;

    return;
}

void Vector_AddWord(vector_t * const vector, uint16_t const wordToAdd)
{
    if (!Vector_IsSpaceAvailable(vector, VECTOR_WORD_BYTES))
    {
        return;
    }

    word_t const word = { .value = wordToAdd };

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const bufferSize = vector->bufferSize;

    buffer[bufferSize + 0] = word.bytes.byte0;
    buffer[bufferSize + 1] = word.bytes.byte1;

    vector->bufferSize += VECTOR_WORD_BYTES;

    return;
}

void Vector_AddDoubleWord(vector_t * const vector, uint32_t const doubleWordToAdd)
{
    if (!Vector_IsSpaceAvailable(vector, VECTOR_DOUBLE_WORD_BYTES))
    {
        return;
    }

    double_word_t doubleWord = { .value = doubleWordToAdd };

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const bufferSize = vector->bufferSize;

    buffer[bufferSize + 0] = doubleWord.bytes.byte0;
    buffer[bufferSize + 1] = doubleWord.bytes.byte1;
    buffer[bufferSize + 2] = doubleWord.bytes.byte2;
    buffer[bufferSize + 3] = doubleWord.bytes.byte3;

    vector->bufferSize += VECTOR_DOUBLE_WORD_BYTES;

    return;
}

void Vector_AddQuadWord(vector_t * const vector, uint64_t const quadWordToAdd)
{
    if (!Vector_IsSpaceAvailable(vector, VECTOR_QUAD_WORD_BYTES))
    {
        return;
    }

    quad_word_t quadWord = { .value = quadWordToAdd };

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const bufferSize = vector->bufferSize;

    buffer[bufferSize + 0] = quadWord.bytes.byte0;
    buffer[bufferSize + 1] = quadWord.bytes.byte1;
    buffer[bufferSize + 2] = quadWord.bytes.byte2;
    buffer[bufferSize + 3] = quadWord.bytes.byte3;
    buffer[bufferSize + 4] = quadWord.bytes.byte4;
    buffer[bufferSize + 5] = quadWord.bytes.byte5;
    buffer[bufferSize + 6] = quadWord.bytes.byte6;
    buffer[bufferSize + 7] = quadWord.bytes.byte7;

    vector->bufferSize += VECTOR_QUAD_WORD_BYTES;

    return;
}

uint8_t Vector_RemoveByte(vector_t * const vector)
{
    vector->bufferSize -= VECTOR_BYTES_BYTES;

    return vector->internalBuffer[vector->bufferSize];
}

uint16_t Vector_RemoveWord(vector_t * const vector)
{
    vector->bufferSize -= VECTOR_WORD_BYTES;

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const bufferSize = vector->bufferSize;

    word_t const word = {
        .bytes.byte0 = buffer[bufferSize + 0],
        .bytes.byte1 = buffer[bufferSize + 1]
    };

    return word.value;
}

uint32_t Vector_RemoveDoubleWord(vector_t * const vector)
{
    vector->bufferSize -= VECTOR_DOUBLE_WORD_BYTES;

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const bufferSize = vector->bufferSize;

    double_word_t const doubleWord = {
        .bytes.byte0 = buffer[bufferSize + 0],
        .bytes.byte1 = buffer[bufferSize + 1],
        .bytes.byte2 = buffer[bufferSize + 2],
        .bytes.byte3 = buffer[bufferSize + 3],
    };

    return doubleWord.value;
}

uint64_t Vector_RemoveQuadWord(vector_t * const vector)
{
    vector->bufferSize -= VECTOR_QUAD_WORD_BYTES;

    uint8_t * const buffer = vector->internalBuffer;
    uint8_t const bufferSize = vector->bufferSize;

    quad_word_t const quadWord = {
        .bytes.byte0 = buffer[bufferSize + 0],
        .bytes.byte1 = buffer[bufferSize + 1],
        .bytes.byte2 = buffer[bufferSize + 2],
        .bytes.byte3 = buffer[bufferSize + 3],
        .bytes.byte4 = buffer[bufferSize + 4],
        .bytes.byte5 = buffer[bufferSize + 5],
        .bytes.byte6 = buffer[bufferSize + 6],
        .bytes.byte7 = buffer[bufferSize + 7],
    };

    return quadWord.value;
}

uint8_t Vector_FirstByte(vector_t const * const vector)
{
    return vector->internalBuffer[0];
}

uint8_t Vector_LastByte(vector_t const * const vector)
{
    return vector->internalBuffer[vector->bufferSize - 1];
}

// TODO: First function for the rest
// TODO: Last function for the rest