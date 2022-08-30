/**
 *  @file utils.h
 *  @author Cristian Cristea - M70957
 *  @date 30 August 2022
 *
 *  @brief Header file for utils functions for the Sensors station
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


#ifndef UTILS_H
#define	UTILS_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "oled-draw.h"
#include "bme280.h"
#include "hc-05.h"
#include "oled.h"


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the system's clock and peripherals (UART and SPI).
 **/
void SystemInitialize(void);

/**
 * @brief Initializes the system's moudules: OLEDC and HC-05.
 *
 * @param[in, out] oled The OLEDC module to initialize
 * @param[in, out] bluetooth The HC-05 module to initialize
 **/
void ModulesInitialize(oled_device_t * const oled , hc05_device_t * const bluetooth);

/**
 * @brief Displays the data on the console and OLED screen.
 *
 * @param[in] sensorsData The data to display
 * @param[in] oled The OLEDC module to use for the display
 **/
void DisplayData(bme280_data_t const * const sensorsData, oled_device_t const * const oled);

#endif // UTILS_H
