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

#include "robotInformation.hpp"

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
bool motors_followHeading(RobotInfo_t *robotInfo, int16_t headingSetpoint, int16_t speed);
bool motors_formShape(RobotInfo_t *robotInfo, uint32_t sideLenght, int16_t rotationAngle);

#endif // MOTOR_H
