/**
 *  @file mics-6814.c
 *  @author Cristian Cristea - M70957
 *  @date 24 August 2022
 *
 *  @brief Source file for the MiCS-6814 module
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

#include "mics-6814.h"

#include "ads1015.h"
#include "config.h"

#include <util/delay.h>

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define MICS6814_CHANNEL_NO2    ADS1015_AIN0_GND
#define MICS6814_CHANNEL_NH3    ADS1015_AIN1_GND
#define MICS6814_CHANNEL_CO     ADS1015_AIN2_GND


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                            Private (static) API                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Checks if the device and its ADC device are valid.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return mics6814_error_code_t Error code
 **/
__attribute__((always_inline)) inline static mics6814_error_code_t MICS6814_CheckNull(mics6814_device_t const * const device);

/**
 * @brief TODO
 *
 * @param device
 *
 * @return mics6814_error_code_t
 */
__attribute__((always_inline)) inline static mics6814_error_code_t MICS6814_ConfigureADC(mics6814_device_t * const device, i2c_t const * const i2cDevice);


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Private (static) definitions                        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

__attribute__((always_inline)) inline static mics6814_error_code_t MICS6814_CheckNull(mics6814_device_t const * const device)
{
    if (device == NULL || device->adc == NULL)
    {
        LOG_ERROR("Found NULL pointer on MiCS-6814");
        return MICS6814_NULL_POINTER;
    }
    else
    {
        return MICS6814_OK;
    }
}

__attribute__((always_inline)) inline static mics6814_error_code_t MICS6814_ConfigureADC(mics6814_device_t * const device, i2c_t const * const i2cDevice)
{
    uint16_t const adcConfiguration = ADS1015_PGA_2_048V | ADS1015_CONTINUOUS_MODE | ADS1015_DATA_RATE_128_SPS | ADS1015_COMPARATOR_DISABLE;

    if (ADS1015_Initialize(&device->adc, &ADS1015_I2C0_Handler, i2cDevice, ADS1015_I2C_ADDRESS, adcConfiguration) == ADS1015_OK)
    {
        return MICS6814_OK;
    }
    else
    {
        return MICS6814_FAIL_CONFIGURATION;
    }
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

mics6814_error_code_t MICS6814_Initialize(mics6814_device_t * const device, ads1015_device_t * const adcDevice, i2c_t const * const i2cDevice)
{
    device->adc = adcDevice;

    mics6814_error_code_t initResult = MICS6814_OK;

    LOG_INFO("Started MiCS-8614 initialization");

    initResult = MICS6814_CheckNull(device);

    if (initResult == MICS6814_OK)
    {
        initResult = MICS6814_ConfigureADC(device, i2cDevice);
    }

    if (initResult == MICS6814_OK)
    {
        LOG_INFO("Finished the MiCS-8614 initialization");
    }
    else
    {
        LOG_ERROR("Couldn't finish the MiCS-8614 initialization");
    }

    return initResult;
}

void MICS6814_SetInputCarbonMonoxide(mics6814_device_t const * const device)
{
    uint16_t configuration = 0;
    ADS1015_GetConfiguration(&device->adc, &configuration);
    configuration &= ~(MICS6814_CHANNEL_NO2 | MICS6814_CHANNEL_NH3);
    configuration |= MICS6814_CHANNEL_CO;

    // For whatever reason this doesn't work with a single function call
    ADS1015_SetConfiguration(&device->adc, configuration);
    ADS1015_SetConfiguration(&device->adc, configuration);

    PauseMiliseconds(100);

    return;
}

void MICS6814_SetInputNitrogenDioxide(mics6814_device_t const * const device)
{
    uint16_t configuration = 0;
    ADS1015_GetConfiguration(&device->adc, &configuration);
    configuration &= ~(MICS6814_CHANNEL_CO | MICS6814_CHANNEL_NH3);
    configuration |= MICS6814_CHANNEL_NO2;

    // For whatever reason this doesn't work with a single function call
    ADS1015_SetConfiguration(&device->adc, configuration);
    ADS1015_SetConfiguration(&device->adc, configuration);

    PauseMiliseconds(1000);

    return;
}

void MICS6814_SetInputAmmonia(mics6814_device_t const * const device)
{
    uint16_t configuration = 0;
    ADS1015_GetConfiguration(&device->adc, &configuration);
    configuration &= ~(MICS6814_CHANNEL_CO | MICS6814_CHANNEL_NO2);
    configuration |= MICS6814_CHANNEL_NH3;

    // For whatever reason this doesn't work with a single function call
    ADS1015_SetConfiguration(&device->adc, configuration);
    ADS1015_SetConfiguration(&device->adc, configuration);

    PauseMiliseconds(1000);

    return;
}

uint16_t MICS6814_ReadValue(mics6814_device_t const * const device)
{
    uint16_t value = 0;
    if (ADS1015_GetConversion(&device->adc, &value) == ADS1015_OK)
    {
        return value;
    }
    else
    {
        LOG_WARNING("Couldn't read MiCS-6814 value");
        return 0;
    }
}