/** 
 * @file motors.h
 * @brief Motor control functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-06-03
 */


#ifndef MOTOR_H
#define MOTOR_H


// ===================================== Includes =====================================
#include <Arduino.h>

#include "robotInfo.h"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
enum motor { // Motor selection enum
    MOTOR_1,
    MOTOR_2
};

// ===================================== Function Prototypes ==========================
bool motors_setup();
bool motors_setSpeed(uint8_t selectedMotor, int8_t speed);
void motors_updateInfo(RobotInfo_t *robotInfo);

#endif // MOTOR_H
