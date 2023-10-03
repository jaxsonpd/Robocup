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

bool motors_followHeading(RobotInfo_t *robotInfo, int16_t headingSetpoint, int16_t speed);
void motors_setLeft(int16_t speed);
void motors_setRight(int16_t speed);
bool motors_rotate(RobotInfo_t* robotInfo, int16_t targetHeading, uint16_t rotationSpeed);
void motors_deInit(RobotInfo_t* robotInfo);
void motors_clearErrors(void);
void motors_update(RobotInfo_t* robotInfo);

#endif // MOTOR_H
