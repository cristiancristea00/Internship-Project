/**
 *  @file spi.h
 *  @author Cristian Cristea - M70957
 *  @date 17 August 2022
 *
 *  @brief Header file for the SPI module
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


#ifndef SPI_H
#define	SPI_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "config.h"

#include <avr/io.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum SPI_ERROR_CODE
{
    SPI_OK                  = 0x00,
    SPI_NULL_POINTER        = 0x01
} spi_error_code_t;

typedef enum SPI_CHIP_SELECT
{
    SPI_CS1 = 0x00,
    SPI_CS2 = 0x01,
    SPI_CS3 = 0x02
} spi_chip_select_t;

typedef void (* spi_inititialize_t) (void);
typedef spi_error_code_t (* spi_send_data_t) (uint8_t const * const, uint8_t const);
typedef spi_error_code_t (* spi_receive_data_t) (uint8_t * const, uint8_t const);
typedef spi_error_code_t (* spi_exchange_data_t) (uint8_t * const, uint8_t const);
typedef void  (* spi_client_t) (spi_chip_select_t const);

/**
 * @brief Object struct for the SPI module
 **/
typedef struct SPI
{
    spi_inititialize_t Initialize;
    spi_send_data_t SendData;
    spi_receive_data_t ReceiveData;
    spi_exchange_data_t ExchangeData;
    spi_client_t ClientSelect;
    spi_client_t ClientDeselect;
} spi_t;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Modules                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

extern spi_t const spi_0;

#endif // SPI_H

