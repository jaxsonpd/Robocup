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
void motors_updateInfo(RobotInfo_t *robotInfo);
bool motors_followHeading(RobotInfo_t *robotInfo, int16_t headingSetpoint, int16_t speed);
bool motors_formShape(RobotInfo_t *robotInfo, uint32_t sideLenght, int16_t rotationAngle);
<<<<<<< main/motors.hpp
bool motors_deInit(void);
=======
void motors_deinit(RobotInfo_t* robotInfo);
void motors_clearErrors(void);

>>>>>>> main/motors.hpp
#endif // MOTOR_H
