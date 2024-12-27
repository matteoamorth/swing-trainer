#ifndef ANALOGSENSOR_H
#define ANALOGSENSOR_H

#include "../defines.h"
typedef float data_t;

/**
 * @brief Represents a generic analog sensor instance.
 * 
 * It can be used with any type of passive analog sensor.
 */
typedef struct analogin_t analogin_t;

/**
 * @brief Creates a new generic analog sensor instance.
 * 
 * This function allocates memory for a generic sensor structure
 * and initializes its fields based on the provided parameters.
 * 
 * @param _pin The analog pin number where the sensor is connected.
 * @param v_supply The supply voltage (in volts) used by the sensor.
 * @param bits The ADC resolution in bits (e.g., 10 for a 10-bit ADC).
 * @param offset The offset of measurements.
 * 
 * @return A pointer to the newly created sensor structure, or NULL if memory allocation fails.
 */
analogin_t *sensor_new(uint8_t _pin, uint8_t v_supply, uint8_t bits, data_t offset);
 
/**
 * @brief Frees the memory allocated for a generic sensor instance.
 * 
 * This function deallocates the memory used by the analogin_t structure.
 * 
 * @param s A pointer to the analogin_t structure to be freed.
 */
void sensor_free(analogin_t *s);

/**
 * @brief Reads the  value from the sensor.
 * 
 * This function calculates the value based on the sensor parameters.
 * 
 * @param s A pointer to the analogin_t structure representing the sensor.
 *          
 * @return The value measurement (type: `data_t`), or an undefined value if the sensor data is invalid.
 */
data_t sensor_read(analogin_t const *s);
data_t sensor_read_filtered(analogin_t const *s);

#endif