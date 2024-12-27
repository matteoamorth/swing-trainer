#ifndef IR_GP2Y0A41_H
#define IR_GP2Y0A41_H

#include "../defines.h"

//  GP2Y0A41SK0F proximity sensor 

#define IR_AVERAGE_READ 10

/**
 * @brief Represents a `IR_GP2Y0A41` sensor instance.
 * 
 * The `GP2Y0A41SK0F` is a distance measuring sensor unit , 
 * composed of an integrated combination of PSD (position sensitive detector), 
 * IR-LED (infrared emitting diode) and signal processing circuit.
 */
typedef struct IR_GP2Y0A41_t IR_GP2Y0A41_t;
/**
 * @brief Creates a new IR_GP2Y0A41 sensor instance.
 * 
 * This function allocates memory for an IR_GP2Y0A41 sensor structure
 * and initializes its fields based on the provided parameters.
 * 
 * @param _pin The analog pin number where the sensor is connected.
 * @param v_supply The supply voltage (in volts) used by the sensor.
 * @param bits The ADC resolution in bits (e.g., 10 for a 10-bit ADC).
 * @return A pointer to the newly created IR_GP2Y0A41_t structure, or NULL if memory allocation fails.
 */
IR_GP2Y0A41_t *IR_sens_new(uint8_t _pin, uint8_t v_supply, uint8_t bits);

/**
 * @brief Frees the memory allocated for an IR_GP2Y0A41 sensor instance.
 * 
 * This function deallocates the memory used by the IR_GP2Y0A41_t structure.
 * 
 * @param s A pointer to the IR_GP2Y0A41_t structure to be freed.
 */
void IR_sens_free(IR_GP2Y0A41_t *s);

/**
 * @brief Reads the raw distance value from the IR sensor.
 * 
 * This function calculates the distance based on the sensor parameters.
 * 
 * @param s A pointer to the IR_GP2Y0A41_t structure representing the sensor.
 *          
 * @return The distance measurement (type: `data_t`), or an undefined value if the sensor data is invalid.
 */
data_t IR_read(IR_GP2Y0A41_t const *s);

/**
 * @brief Reads a filtered (averaged) distance value from the IR sensor.
 * 
 * This function performs multiple readings from the sensor and returns
 * the average value to reduce noise in the measurements. 
 * Adjust IR_AVERAGE_READ to enlarge or reduce measures number.
 * 
 * @param s A pointer to the IR_GP2Y0A41_t structure representing the sensor.
 *  
 * @return The distance measurement.
 */
data_t IR_read_filtered(IR_GP2Y0A41_t const *s);


#endif
