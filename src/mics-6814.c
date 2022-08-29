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
#include <math.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Macros and defines                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define MICS6814_BASE_RESISTANCE_RED    200.0f  // TODO: Find a fucking correct value
#define MICS6814_BASE_RESISTANCE_OX     10.0f   // TODO: The same problem
#define MICS6814_BASE_RESISTANCE_NH3    100.0f  // TODO: Once again I am asking for mercy


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                        Typedefs, enums and structs                         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

typedef enum MICS6814_CHANNEL
{
    MICS6814_CHANNEL_OX_SENSOR  = ADS1015_AIN0_GND,
    MICS6814_CHANNEL_NH3_SENSOR = ADS1015_AIN1_GND,
    MICS6814_CHANNEL_RED_SENSOR = ADS1015_AIN2_GND
} mics6814_channel_t;

typedef enum MICS6814_GAS
{
    MICS6814_CARBON_MONOXIDE  = 0x00,
    MICS6814_NITROGEN_DIOXIDE = 0x01,
    MICS6814_ETHANOL          = 0x02,
    MICS6814_HYDROGEN         = 0x03,
    MICS6814_AMMONIA          = 0x04,
    MICS6814_METHANE          = 0x05,
    MICS6814_PROPANE          = 0x06,
    MICS6814_ISOBUTANE        = 0x07
} mics6814_gas_t;


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
 * @brief Initializes the ADC device.
 *
 * @param[in, out] device MiCS-6814 device
 * @param[in]      i2cDevice I2C device
 *
 * @return mics6814_error_code_t Error code
 **/
__attribute__((always_inline)) inline static mics6814_error_code_t MICS6814_ConfigureADC(mics6814_device_t * const device, i2c_t const * const i2cDevice);

/**
 * @brief Sets the MiCS-6814's ADC input to the RED channel.
 *
 * @param[in] device MiCS-6814 device
 **/
static void MICS6814_SetADCInputToRED(mics6814_device_t const * const device);

/**
 * @brief Sets the MiCS-6814's ADC input to the OX channel.
 *
 * @param[in] device MiCS-6814 device
 **/
static void MICS6814_SetADCInputToOX(mics6814_device_t const * const device);

/**
 * @brief Sets the MiCS-6814's ADC input to the NH3 channel.
 *
 * @param[in] device MiCS-6814 device
 **/
static void MICS6814_SetADCInputToNH3(mics6814_device_t const * const device);

/**
 * @brief Reads the MiCS-6814's current ADC input.
 *
 * @param[in] device MiCS-6814 device
 *
 * @return uint16_t ADC value
 **/
static uint16_t MICS6814_ReadValue(mics6814_device_t const * const device);

/**
 * @brief Computes the resistance ratio according to the base resistance and the
 *        current read value.
 *
 * @param[in] device MiCS-6814 device
 * @param[in] channel Channel to be read
 *
 * @return double Computed ratio
 **/
static double MICS6814_GetResistanceRatio(mics6814_device_t const * const device, mics6814_channel_t const channel);

/**
 * @brief Measures the speciefied gas concentration in ppm.
 *
 * @param[in] device MiCS-6814 device
 * @param[in] gas Gas to be measured
 *
 * @return double Gas concentration in ppm
 **/
static double MICS6814_Measure(mics6814_device_t const * const device, mics6814_gas_t const gas);

/**
 * @brief Computes the Carbon Monoxide (CO) concentration in ppm.
 *
 * @param[in] ratio_RED Resistance ratio of the RED channel
 *
 * @return double CO concentration in ppm
 **/
__attribute__((always_inline)) inline static double MICS6814_ComputeCarbonMonoxide(double const ratio_RED);

/**
 * @brief Computes the Nitrogen Dioxide (NO2) concentration in ppm.
 *
 * @param[in] ratio_OX Resistance ratio of the OX channel
 *
 * @return double NO2 concentration in ppm
 **/
__attribute__((always_inline)) inline static double MICS6814_ComputeNitrogenDioxide(double const ratio_OX);

/**
 * @brief Computes the Ethanol (C2H5OH) concentration in ppm.
 *
 * @param[in] ratio_RED Resistance ratio of the RED channel
 * @param[in] ratio_NH3 Resistance ratio of the NH3 channel
 *
 * @return double C2H5OH concentration in ppm
 **/
__attribute__((always_inline)) inline static double MICS6814_ComputeEthanol(double const ratio_RED, double const ratio_NH3);

/**
 * @brief Computes the Hydrogen (H2) concentration in ppm.
 *
 * @param[in] ratio_RED Resistance ratio of the RED channel
 * @param[in] ratio_NH3 Resistance ratio of the NH3 channel
 *
 * @return double H2 concentration in ppm
 **/
__attribute__((always_inline)) inline static double MICS6814_ComputeHydrogen(double const ratio_RED, double const ratio_NH3);

/**
 * @brief Computes the Ammonia (NH3) concentration in ppm.
 *
 * @param[in] ratio_RED Resistance ratio of the RED channel
 * @param[in] ratio_NH3 Resistance ratio of the NH3 channel
 *
 * @return double NH3 concentration in ppm
 **/
__attribute__((always_inline)) inline static double MICS6814_ComputeAmmonia(double const ratio_RED, double const ratio_NH3);

/**
 * @brief Computes the Methane (CH4) concentration in ppm.
 *
 * @param[in] ratio_RED Resistance ratio of the RED channel
 *
 * @return double CH4 concentration in ppm
 **/
__attribute__((always_inline)) inline static double MICS6814_ComputeMethane(double const ratio_RED);

/**
 * @brief Computes the Propane (C3H8) concentration in ppm.
 *
 * @param[in] ratio_RED Resistance ratio of the RED channel
 * @param[in] ratio_NH3 Resistance ratio of the NH3 channel
 *
 * @return double C3H8 concentration in ppm
 **/
__attribute__((always_inline)) inline static double MICS6814_ComputePropane(double const ratio_RED, double const ratio_NH3);

/**
 * @brief Computes the Isobutane (C4H10) concentration in ppm.
 *
 * @param[in] ratio_RED Resistance ratio of the RED channel
 * @param[in] ratio_NH3 Resistance ratio of the NH3 channel
 *
 * @return double C4H10 concentration in ppm
 **/
__attribute__((always_inline)) inline static double MICS6814_ComputeIsobutane(double const ratio_RED, double const ratio_NH3);


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
    uint16_t const adcConfiguration = ADS1015_PGA_2_048V | ADS1015_CONTINUOUS_MODE | ADS1015_DATA_RATE_250_SPS | ADS1015_COMPARATOR_DISABLE;

    if (ADS1015_Initialize(&device->adc, &ADS1015_I2C0_Handler, i2cDevice, ADS1015_I2C_ADDRESS, adcConfiguration) == ADS1015_OK)
    {
        return MICS6814_OK;
    }
    else
    {
        return MICS6814_FAIL_CONFIGURATION;
    }
}

static void MICS6814_SetADCInputToRED(mics6814_device_t const * const device)
{
    uint16_t configuration = 0;
    ADS1015_GetConfiguration(&device->adc, &configuration);
    configuration &= ~(MICS6814_CHANNEL_OX_SENSOR | MICS6814_CHANNEL_NH3_SENSOR);
    configuration |= MICS6814_CHANNEL_RED_SENSOR;

    // For whatever reason this doesn't work with a single function call
    ADS1015_SetConfiguration(&device->adc, configuration);
    ADS1015_SetConfiguration(&device->adc, configuration);

    PauseMicroseconds(ADS1015_DATA_RATE_250_SPS_DELAY);

    return;
}

static void MICS6814_SetADCInputToOX(mics6814_device_t const * const device)
{
    uint16_t configuration = 0;
    ADS1015_GetConfiguration(&device->adc, &configuration);
    configuration &= ~(MICS6814_CHANNEL_RED_SENSOR | MICS6814_CHANNEL_NH3_SENSOR);
    configuration |= MICS6814_CHANNEL_OX_SENSOR;

    // For whatever reason this doesn't work with a single function call
    ADS1015_SetConfiguration(&device->adc, configuration);
    ADS1015_SetConfiguration(&device->adc, configuration);

    PauseMicroseconds(ADS1015_DATA_RATE_250_SPS_DELAY);

    return;
}

static void MICS6814_SetADCInputToNH3(mics6814_device_t const * const device)
{
    uint16_t configuration = 0;
    ADS1015_GetConfiguration(&device->adc, &configuration);
    configuration &= ~(MICS6814_CHANNEL_RED_SENSOR | MICS6814_CHANNEL_OX_SENSOR);
    configuration |= MICS6814_CHANNEL_NH3_SENSOR;

    // For whatever reason this doesn't work with a single function call
    ADS1015_SetConfiguration(&device->adc, configuration);
    ADS1015_SetConfiguration(&device->adc, configuration);

    PauseMicroseconds(ADS1015_DATA_RATE_250_SPS_DELAY);

    return;
}

static uint16_t MICS6814_ReadValue(mics6814_device_t const * const device)
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

static double MICS6814_GetResistanceRatio(mics6814_device_t const * const device, mics6814_channel_t const channel)
{
    uint16_t readValue = 0;

    switch (channel)
    {
        case MICS6814_CHANNEL_RED_SENSOR:
            MICS6814_SetADCInputToRED(device);
            readValue = MICS6814_ReadValue(device);
            readValue = ((double) (1000UL * readValue)) / ((double) (2047 - readValue));
            return readValue / MICS6814_BASE_RESISTANCE_RED;
        case MICS6814_CHANNEL_OX_SENSOR:
            MICS6814_SetADCInputToOX(device);
            readValue = MICS6814_ReadValue(device);
            readValue = ((double) (15UL * readValue)) / ((double) (2047 - readValue));
            return readValue / MICS6814_BASE_RESISTANCE_OX;
        case MICS6814_CHANNEL_NH3_SENSOR:
            MICS6814_SetADCInputToNH3(device);
            readValue = MICS6814_ReadValue(device);
            readValue = ((double) (1000UL * readValue)) / ((double) (2047 - readValue));
            return readValue / MICS6814_BASE_RESISTANCE_NH3;
        default:
            LOG_ERROR("Invalid channel type in MiCS-6814 selection");
            return 0;
    }
}

static double MICS6814_Measure(mics6814_device_t const * const device, mics6814_gas_t const gas)
{
    double const ratio_RED = MICS6814_GetResistanceRatio(device, MICS6814_CHANNEL_RED_SENSOR);
    double const ratio_OX  = MICS6814_GetResistanceRatio(device, MICS6814_CHANNEL_OX_SENSOR);
    double const ratio_NH3 = MICS6814_GetResistanceRatio(device, MICS6814_CHANNEL_NH3_SENSOR);

    switch (gas)
    {
        case MICS6814_CARBON_MONOXIDE:
            return MICS6814_ComputeCarbonMonoxide(ratio_RED);
        case MICS6814_NITROGEN_DIOXIDE:
            return MICS6814_ComputeNitrogenDioxide(ratio_OX);
        case MICS6814_ETHANOL:
            return MICS6814_ComputeEthanol(ratio_RED, ratio_NH3);
        case MICS6814_HYDROGEN:
            return MICS6814_ComputeHydrogen(ratio_RED, ratio_NH3);
        case MICS6814_AMMONIA:
            return MICS6814_ComputeAmmonia(ratio_RED, ratio_NH3);
        case MICS6814_METHANE:
            return MICS6814_ComputeMethane(ratio_RED);
        case MICS6814_PROPANE:
            return MICS6814_ComputePropane(ratio_RED, ratio_NH3);
        case MICS6814_ISOBUTANE:
            return MICS6814_ComputeIsobutane(ratio_RED, ratio_NH3);
        default:
            LOG_ERROR("Invalid gas type in MiCS-6814 selection");
            return 0;
    }
}

__attribute__((always_inline)) inline static double MICS6814_ComputeCarbonMonoxide(double const ratio_RED)
{
    double result = 0;

    if (ratio_RED > 4  || ratio_RED < 0.01)
    {
        result = 0;
    }
    else
    {
        result = pow(ratio_RED, -1.177) * 4.4638;

        if (result < 1 || result > 1000)
        {
            result = 0;
        }
    }

    return result;
}

__attribute__((always_inline)) inline static double MICS6814_ComputeNitrogenDioxide(double const ratio_OX)
{
    double result = 0;

    if (ratio_OX > 40 || ratio_OX < 0.06)
    {
        result = 0;
    }
    else
    {
        result = pow(ratio_OX, 0.9979) * 0.1516;

        if (result < 0.01 || result > 7)
        {
            result = 0;
        }
    }

    return result;
}

__attribute__((always_inline)) inline static double MICS6814_ComputeEthanol(double const ratio_RED, double const ratio_NH3)
{
    double result = 0;

    if (ratio_RED > 1.05 || ratio_RED < 0.03)
    {
        if (ratio_NH3 < 0.8 && ratio_NH3 > 0.06)
        {
            result = pow(ratio_NH3, -2.781) * 0.2068;

            if (result < 1 || result > 250)
            {
                result = 0;
            }
        }
        else
        {
            result = 0;
        }
    }
    else
    {
        result = pow(ratio_RED, -1.58) * 1.363;

        if (result < 1.5 || result > 250)
        {
            result = 0;
        }
    }

    return result;
}

__attribute__((always_inline)) inline static double MICS6814_ComputeHydrogen(double const ratio_RED, double const ratio_NH3)
{
    double result = 0;

    if (ratio_RED > 1 || ratio_RED < 0.035)
    {
        if (ratio_NH3 < 2 && ratio_NH3 > 0.3)
        {
            result = pow(ratio_NH3, -2.948) * 8.0074;

            if (result < 1 || result > 200)
            {
                result = 0;
            }
        }
        else
        {
            result = 0;
        }
    }
    else
    {
        result = pow(ratio_RED, -1.781) * 0.828;

        if (result < 1 || result > 200)
        {
            result = 0;
        }
    }

    return result;
}

__attribute__((always_inline)) inline static double MICS6814_ComputeAmmonia(double const ratio_RED, double const ratio_NH3)
{
    double result = 0;

    if (ratio_NH3 > 0.9 || ratio_NH3 < 0.05)
    {
        if (ratio_RED < 1 && ratio_RED > 0.3)
        {
            result = pow(ratio_RED, -4.33) * 0.974;

            if (result < 1 || result > 160)
            {
                result = 0;
            }
        }
        else
        {
            result = 0;
        }
    }
    else
    {
        result = pow(ratio_NH3, -1.903) * 0.6151;

        if (result < 1 || result > 160)
        {
            result = 0;
        }
    }

    return result;
}

__attribute__((always_inline)) inline static double MICS6814_ComputeMethane(double const ratio_RED)
{
    double result = 0;

    if (ratio_RED > 0.8  || ratio_RED < 0.45)
    {
        result = 0;
    }
    else
    {
        result = pow(ratio_RED, -4.093) * 837.38;

        if (result < 3000 || result > 10500)
        {
            result = 0;
        }
    }

    return result;
}

__attribute__((always_inline)) inline static double MICS6814_ComputePropane(double const ratio_RED, double const ratio_NH3)
{
    double result = 0;

    if (ratio_NH3 > 0.9 || ratio_NH3 < 0.2)
    {
        if (ratio_RED < 0.18 && ratio_RED > 0.05)
        {
            result = pow(ratio_RED, -1.316) * 323.64;

            if (result < 3000 || result > 10500)
            {
                result = 0;
            }
        }
        else
        {
            result = 0;
        }
    }
    else
    {
        result = pow(ratio_NH3, -2.492) * 569.56;

        if (result < 1000 || result > 30000)
        {
            result = 0;
        }
    }

    return result;
}

__attribute__((always_inline)) inline static double MICS6814_ComputeIsobutane(double const ratio_RED, double const ratio_NH3)
{
    double result = 0;

    if (ratio_NH3 > 0.8 || ratio_NH3 < 0.1)
    {
        if (ratio_RED < 0.07 && ratio_RED > 0.05)
        {
            result = 44680 - (556000 * ratio_RED);

            if (result < 3000 || result > 10500)
            {
                result = 0;
            }
        }
        else
        {
            result = 0;
        }
    }
    else
    {
        result = pow(ratio_NH3, -1.888) * 503.2;

        if (result < 1000 || result > 30000)
        {
            result = 0;
        }
    }

    return result;
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

__attribute__((always_inline)) inline double MICS6814_GetCarbonMonoxide(mics6814_device_t * const device)
{
    return MICS6814_Measure(device, MICS6814_CARBON_MONOXIDE);
}

__attribute__((always_inline)) inline double MICS6814_GetNitrogenDioxide(mics6814_device_t * const device)
{
    return MICS6814_Measure(device, MICS6814_NITROGEN_DIOXIDE);
}

__attribute__((always_inline)) inline double MICS6814_GetEthanol(mics6814_device_t * const device)
{
    return MICS6814_Measure(device, MICS6814_ETHANOL);
}

__attribute__((always_inline)) inline double MICS6814_GetHydrogen(mics6814_device_t * const device)
{
    return MICS6814_Measure(device, MICS6814_HYDROGEN);
}

__attribute__((always_inline)) inline double MICS6814_GetAmmonia(mics6814_device_t * const device)
{
    return MICS6814_Measure(device, MICS6814_AMMONIA);
}

__attribute__((always_inline)) inline double MICS6814_GetMethane(mics6814_device_t * const device)
{
    return MICS6814_Measure(device, MICS6814_METHANE);
}

__attribute__((always_inline)) inline double MICS6814_GetPropane(mics6814_device_t * const device)
{
    return MICS6814_Measure(device, MICS6814_PROPANE);
}

__attribute__((always_inline)) inline double MICS6814_GetIsobutane(mics6814_device_t * const device)
{
    return MICS6814_Measure(device, MICS6814_ISOBUTANE);
}
