/** 
 * @file sensors.h
 * @brief Sensor control functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-13
 */


#ifndef SENSORS_H
#define SENSORS_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include "USConfig.h"

// ===================================== Types/Constants ==============================

// IR trangulating sensor struct
typedef struct {
    uint8_t pin; // the pin the sensor is connected to
    uint8_t type; // 0: , 1:, 2: 20-150cm
} irTri_sensor_t;

// ===================================== Function Prototypes ==========================
bool sensors_init(void);
uint16_t sensors_getIRTriDistance(irTri_sensor_t sensor);
void sensors_pingUS(void);
void sensors_getUSDistance(uint16_t distances[US_NUM]);

#endif // SENSORS_H
