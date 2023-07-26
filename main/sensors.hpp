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

#include "src/USConfig.hpp"
#include "robotInformation.hpp"

// ===================================== Types/Constants ==============================

// ===================================== Function Prototypes ==========================
bool sensors_init(void);
void sensors_updateInfo(RobotInfo_t* robotInfo);
void sensors_update(void);

#endif // SENSORS_H
