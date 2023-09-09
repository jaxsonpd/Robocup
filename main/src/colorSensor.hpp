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


// ===================================== Function Prototypes ==========================
bool colorSensor_init();
bool colorSensor_overBase();
void colorSensor_setBase();

#endif // COLOR_SENSOR_H