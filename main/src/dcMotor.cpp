/** 
 * @file dcMotor.cpp
 * @brief DC motor class implementation
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-08-20
 */


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include <Arduino.h>
#include <Servo.h>

#include "dcMotor.hpp"
// ===================================== Types/Constants ==============================


// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
/** 
 * @brief Initialise the DC motor
 * @param pin The pin the motor is connected to
 * @param direction The direction the motor is facing (0 = forward, 1 = backward)
 * 
 * @return 1 if the motor was successfully initialised, 0 otherwise
 */
bool dcMotor::init(uint8_t pin, bool direction) {
    // Assign the pin to the class variable
    motorPin = pin;
    motorDirection = direction;

    // Attach the servo object to the pin
    motor.attach(pin);

    return 1;
}

/**
 * @brief Set the speed of the motor
 * @param speed The speed to set the motor to (-100 to 100)
 * 
 * @return 1 if the speed was successfully set, 0 otherwise
 */
bool dcMotor::setSpeed(int16_t speed) {
    int16_t proccessedSpeed = 0; // The speed to set the motor to
    bool inBound = true;

    // Check to see if the speed is in range
    if (speed > MOTOR_SPEED_MAX || speed < MOTOR_SPEED_MIN) {
        inBound = false;
        speed = (speed > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : speed;
        speed = (speed < MOTOR_SPEED_MIN) ? MOTOR_SPEED_MIN : speed;
    }

    // Invert motor 2 speed
    proccessedSpeed = (!motorDirection) ? -speed : speed;

    // Convert the speed to 1050-1950
    proccessedSpeed = map(proccessedSpeed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX, 1050, 1950);

    // Set the speed of the motor
    motor.writeMicroseconds(proccessedSpeed);
    motorSpeed = speed;

    return inBound;
}


/**
 * @brief return the speed of the motor
 * 
 * @return The speed of the motor (-100 to 100)
 */
int16_t dcMotor::getSpeed(void) {
    return motorSpeed;
}


/**
 * @brief De-initialise the motor
 * 
 * @return 1 if the motor was successfully de-initialised, 0 otherwise
 */
bool dcMotor::deInit(void) {
    motor.detach();
    return 1;
}