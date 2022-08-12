/**
 *  @file crc8.h
 *  @author Cristian Cristea - M70957
 *  @date 12 August 2022
 *
 *  @brief Header file for the CRC8 module
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


#ifndef CRC8_H
#define	CRC8_H

#include <stdint.h>

#define CRC8_LOOKUP_SIZE      256

#define CRC8_INITIAL_VALUE    0xFF
#define CRC8_XOR_VALUE        0xFF

/**
 * @brief Computes the CRC-8 of a given data buffer using the CRC-8-AUTOSAR
 *        version. The CRC-8-AUTOSAR algorithm has the following properties:
 *        - The polynomial is x^8 + x^5 + x^3 + x^2 + x + 1 (i.e. 0x2F)
 *        - The initial value is 0xFF
 *        - The final XOR value is 0xFF
 *
 * @param data The data buffer to compute the CRC-8 of
 * @param dataLength The length of the data buffer
 *
 * @return uint8_t The CRC-8-AUTOSAR value of the data buffer
 **/
uint8_t CRC8_Compute(uint8_t const * const data, uint8_t const dataLength);

#endif // CRC8_H
