/** 
 * @file colorSensor.hpp
 * @brief Header file for the color sensor module
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-09-09
 */


#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================
enum colors {
    BLUE,
    GREEN,
    ARENA,
    READING
};

// ===================================== Function Prototypes ==========================
bool colorSensor_init();
uint8_t colorSensor_read();
uint8_t colorSensor_getBase();

#endif // COLOR_SENSOR_H