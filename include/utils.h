#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

/**
 * @brief Converts the raw analog value from the potentiometer to an angle.
 * 
 * @param `rawValue` : The raw analog value from the potentiometer (0-4095).
 * @return `uint16_t` : The converted angle value (0-180 degrees).
 */
uint16_t convertToAngle(uint16_t rawValue);

#endif // UTILS_H