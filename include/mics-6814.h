/**
 *  @file mics-6814.h
 *  @author Cristian Cristea - M70957
 *  @date 24 August 2022
 *
 *  @brief Header file for the MiCS-6814 module
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


#ifndef MICS_6814_H
#define	MICS_6814_H


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "ads1015.h"

#include "i2c.h"


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum MICS6814_ERROR_CODE
{
    MICS6814_OK                 = 0x00,
    MICS6814_NULL_POINTER       = 0x01,
    MICS6814_FAIL_CONFIGURATION = 0x02
} mics6814_error_code_t;

typedef struct MICS6814_DEVICE
{
    // Analog to Digital Converter
    ads1015_device_t * adc;
} mics6814_device_t;


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 Public API                                 //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the MiCS-6814 module.
 *
 * @param[in, out] device MiCS-6814 device
 * @param[in]      adcDevice Analog to Digital Converter device
 * @param[in]      i2cDevice I2C device
 *
 * @return mics6814_error_code_t Error code
 **/
mics6814_error_code_t MICS6814_Initialize(mics6814_device_t * const device, ads1015_device_t * const adcDevice, i2c_t const * const i2cDevice);

/**
 * @brief Reads the Carbon Monoxide (CO) concentration from the MiCS-6814 module
 *        and returns the value in ppm.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return double CO concentration in ppm
 **/
double MICS6814_GetCarbonMonoxide(mics6814_device_t * const device);

/**
 * @brief Reads the Nitrogen Dioxide (NO2) concentration from the MiCS-6814
 *        module and returns the value in ppm.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return double NO2 concentration in ppm
 **/
double MICS6814_GetNitrogenDioxide(mics6814_device_t * const device);

/**
 * @brief Reads the Ethanol (C2H5OH) concentration from the MiCS-6814 module and
 *        returns the value in ppm.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return double C2H5OH concentration in ppm
 **/
double MICS6814_GetEthanol(mics6814_device_t * const device);

/**
 * @brief Reads the Hydrogen (H2) concentration from the MiCS-6814 module and
 *        returns the value in ppm.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return double H2 concentration in ppm
 **/
double MICS6814_GetHydrogen(mics6814_device_t * const device);

/**
 * @brief Reads the Ammonia (NH3) concentration from the MiCS-6814 module and
 *        returns the value in ppm.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return double NH3 concentration in ppm
 **/
double MICS6814_GetAmmonia(mics6814_device_t * const device);

/**
 * @brief Reads the Methane (CH4) concentration from the MiCS-6814 module and
 *        returns the value in ppm.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return double CH4 concentration in ppm
 **/
double MICS6814_GetMethane(mics6814_device_t * const device);

/**
 * @brief Reads the Propane (C3H8) concentration from the MiCS-6814 module and
 *        returns the value in ppm.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return double C3H8 concentration in ppm
 **/
double MICS6814_GetPropane(mics6814_device_t * const device);

/**
 * @brief Reads the Isobutane (C4H10) concentration from the MiCS-6814 module
 *        and returns the value in ppm.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return double C4H10 concentration in ppm
 **/
double MICS6814_GetIsobutane(mics6814_device_t * const device);

#endif // MICS_6814_H
