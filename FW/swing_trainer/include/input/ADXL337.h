/**
 * @file ADXL337.h
 * @brief Header file for the ADXL337 accelerometer interface.
 *
 * This file provides the structure definition and function declarations
 * to interface with the ADXL337 accelerometer sensor using analog inputs.
 */

#ifndef ADXL337_H
#define ADXL337_H

#include "../defines.h"
#include "analogsensor.h"

/**
 * @struct ac337_t
 * @brief Represents an ADXL337 accelerometer instance.
 *
 * This structure encapsulates the three sensor axes (x, y, z) as well as
 * scaling factors to convert raw ADC values into meaningful acceleration data.
 * The 
 */
typedef struct ac337_t ac337_t;

/**
 * @brief Creates and initializes an ADXL337 accelerometer instance.
 *
 * Allocates memory for the accelerometer structure and its associated
 * axis sensors (x, y, z). Initializes each axis sensor with its respective
 * parameters. The accelerometer has [-3.0, 3.0] g range for each axis
 *
 * @param pin_x Analog pin for the X-axis.
 * @param pin_y Analog pin for the Y-axis.
 * @param pin_z Analog pin for the Z-axis.
 * @param v_supply Voltage supply to the accelerometer (e.g., 3.3V or 5V).
 * @param range Full measurement range of the accelerometer in g (e.g., 6 for ±3g).
 * @param bits Resolution of the ADC in bits (e.g., 10 for 10-bit ADC).
 * @return Pointer to the initialized `ac337_t` structure, or `NULL` on failure.
 */
ac337_t *ac337_new(uint8_t pin_x, uint8_t pin_y, uint8_t pin_z, data_t v_supply, uint8_t bits);

/**
 * @brief Frees the memory allocated for an ADXL337 instance.
 *
 * Deallocates memory for the accelerometer structure and its associated
 * axis sensors. Ensures proper cleanup of resources.
 *
 * @param s Pointer to the `ac337_t` instance to free.
 */
void ac337_free(ac337_t *s);

/**
 * @brief Reads the raw acceleration value for the X-axis.
 *
 * Converts the raw ADC value to acceleration using the scaling factor.
 *
 * @param s Pointer to the `ac337_t` instance.
 * @return The raw acceleration value for the X-axis in g.
 */
data_t ac337_read_x(ac337_t const *s);

/**
 * @brief Reads the filtered acceleration value for the X-axis.
 *
 * Computes a filtered acceleration value using a moving average or other
 * filtering technique.
 *
 * @param s Pointer to the `ac337_t` instance.
 * @return The filtered acceleration value for the X-axis in g.
 */
data_t ac337_filtered_read_x(ac337_t const *s);

/**
 * @brief Reads the raw acceleration value for the Y-axis.
 *
 * Converts the raw ADC value to acceleration using the scaling factor.
 *
 * @param s Pointer to the `ac337_t` instance.
 * @return The raw acceleration value for the Y-axis in g.
 */
data_t ac337_read_y(ac337_t const *s);

/**
 * @brief Reads the filtered acceleration value for the Y-axis.
 *
 * Computes a filtered acceleration value using a moving average or other
 * filtering technique.
 *
 * @param s Pointer to the `ac337_t` instance.
 * @return The filtered acceleration value for the Y-axis in g.
 */
data_t ac337_filtered_read_y(ac337_t const *s);

/**
 * @brief Reads the raw acceleration value for the Z-axis.
 *
 * Converts the raw ADC value to acceleration using the scaling factor.
 *
 * @param s Pointer to the `ac337_t` instance.
 * @return The raw acceleration value for the Z-axis in g.
 */
data_t ac337_read_z(ac337_t const *s);

/**
 * @brief Reads the filtered acceleration value for the Z-axis.
 *
 * Computes a filtered acceleration value using a moving average or other
 * filtering technique.
 *
 * @param s Pointer to the `ac337_t` instance.
 * @return The filtered acceleration value for the Z-axis in g.
 */
data_t ac337_filtered_read_z(ac337_t const *s);

#endif // ADXL337_H
