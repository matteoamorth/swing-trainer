/**
 * @file digitalin.h
 * @brief Library for managing digital input pins with optional reverse logic.
 * 
 * This library provides a simple interface to handle digital input pins on a microcontroller.
 */

#ifndef DIGITALIN_H
#define DIGITALIN_H

#include "../defines.h"

/**
 * @struct digitalin_t
 * @brief Represents a digital input pin with additional configuration options.
 * 
 * This structure encapsulates the configuration and state for a digital input pin.
 * 
 * @param pin
 * The pin number associated with the digital input.
 * 
 * @param t
 * The debounce time in milliseconds to stabilize the input signal.
 * 
 * @param reverse
 * Whether the input logic is reversed (active LOW instead of active HIGH).
 */
typedef struct digitalin_t digitalin_t;

/**
 * @brief Creates and initializes a new digital input instance.
 * 
 * Allocates memory for a digital input object and configures the pin mode, debounce time, 
 * and logic settings.
 * 
 * @param _pin The pin number to configure as a digital input.
 * @param debounce_time The debounce time in milliseconds to stabilize the input signal.
 * @param internal_pullup Whether to enable the internal pull-up resistor.
 *                        - `true` enables the internal pull-up.
 *                        - `false` configures the pin as standard input.
 * @param reverse_logic Whether to reverse the logic:
 *                      - `true` for active LOW logic.
 *                      - `false` for standard active HIGH logic.
 * @return A pointer to the allocated and initialized `digitalin_t` structure, 
 *         or `NULL` if memory allocation fails.
 */
digitalin_t* digital_new(uint8_t _pin, uint8_t debounce_time, bool internal_pullup, bool reverse_logic);

/**
 * @brief Frees memory allocated for a digital input instance.
 * 
 * Releases memory allocated for a `digitalin_t` instance.
 * 
 * @param i A pointer to the `digitalin_t` instance to free.
 */
void digital_free(digitalin_t *i);

/**
 * @brief Reads the current state of a digital input pin.
 * 
 * Reads the logic level of the associated pin, taking into account the reverse logic setting.
 * 
 * @param i A pointer to the `digitalin_t` instance representing the pin.
 * @return The current state of the pin:
 *         - `true` if the pin is HIGH (or LOW if reverse logic is enabled).
 *         - `false` if the pin is LOW (or HIGH if reverse logic is enabled).
 */
bool digital_read(digitalin_t *i);

#endif 
