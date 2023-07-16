/** 
 * @file motors.cpp
 * @brief Motor control functions for the robot cup project
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @date 2023-06-03
 */


// ===================================== Includes =====================================
#include <Arduino.h>
#include <Servo.h>

#include "motors.h"
#include "robotInfo.h"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
// ** DC Motor parameters **
#define MOTOR_1_PIN 0
#define MOTOR_2_PIN 1

#define MOTOR_SPEED_MAX 100
#define MOTOR_SPEED_MIN -100

Servo M1, M2; // Define the servo objects for each motor

// ===================================== Globals ======================================
int8_t motor1Speed = 0; // Speed of motor 1 (-100 to 100)
int8_t motor2Speed = 0; // Speed of motor 2 (-100 to 100)

// ===================================== Function Definitions =========================
/**
 * @brief Setup the motors for use
 * 
 * @return success (0) or failure (1)
 */
bool motors_setup(void) {
    // Attach the motors to each servo object
    M1.attach(MOTOR_1_PIN);
    M2.attach(MOTOR_2_PIN);

    return 0;
}

/**
 * @brief Set the speed of a motor
 * 
 * @param motor The motor to set the speed of
 * @param speed The speed to set the motor to
 * 
 * @return success (0) or failure (1)
 */
bool motors_setSpeed(uint8_t selectedMotor, int8_t speed) {
    int16_t proccessedSpeed = 0; // The speed to set the motor to

    // Check to see if the speed is in range
    if (speed > MOTOR_SPEED_MAX || speed < MOTOR_SPEED_MIN) {
        return 1;
    }

    // Invert motor 2 speed
    proccessedSpeed = (selectedMotor == MOTOR_2) ? -speed : speed;

    // Convert the speed to 1050-1950
    proccessedSpeed = map(proccessedSpeed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX, 1050, 1950);

    // Set the speed of the motor
    if (selectedMotor == MOTOR_1) {
        M1.writeMicroseconds(proccessedSpeed);
        motor1Speed = speed;
    } else if (selectedMotor == MOTOR_2) {
        M2.writeMicroseconds(proccessedSpeed);
        motor2Speed = speed;
    } else {
        return 1;
    }

    return 0;
}


/** 
 * @brief Update the motor speeds in the robotInfo struct
 * @param robotInfo The robotInfo struct to update
 * 
 */
void motors_updateInfo(RobotInfo_t *robotInfo) {
    // Update the motor speeds
    robotInfo->leftMotorSpeed = motor1Speed;
    robotInfo->rightMotorSpeed = motor2Speed;
}




