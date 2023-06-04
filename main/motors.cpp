/** 
 * @file motors.cpp
 * @brief Motor control functions for the robot cup project
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @date 2023-06-03
 */


// ===================================== Includes =====================================
#include "arduino.h"

#include "motors.h"

#include <Servo.h>
#include <stdint.h>


// ===================================== Constants ====================================
#define MOTOR_1_PIN 0
#define MOTOR_2_PIN 1

#define MOTOR_SPEED_MAX 100
#define MOTOR_SPEED_MIN -100

Servo M1, M2; // Define the servo objects for each motor

// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
/**
 * @brief Setup the motors for use
 * 
 * @return success (0) or failure (1)
 */
bool motors_setup() {
    // Attach the motors to each servo object
    M1.attach(MOTOR_1_PIN);
    M2.attach(MOTOR_2_PIN);

    return 0;
}


/**
 * @brief Ramp the motors up to the speed setting
 * 
 * @param motor The motor to ramp up
 * @param startSpeed The speed to start ramping from
 * @param EndSpeed The speed to ramp up to
 * @param rampTime The time between 1% increases to take to ramp up
 * 
 * @return the current motor speed
 */
int16_t motors_ramp(uint8_t motor, int16_t startSpeed, int16_t EndSpeed, int32_t rampSpeed) {
    // Get the starting time
    static int32_t startTime = millis();
    static int16_t speed = startSpeed;
    static int16_t prevousStartSpeed = startSpeed;

    // If start speed has changed reset the ramp
    if (prevousStartSpeed != startSpeed) {
        startTime = millis();
        prevousStartSpeed = startSpeed;
        speed = startSpeed;
    }
    uint16_t elapsed = millis() - startTime;

    if (elapsed > rampSpeed) {
        // Increment the speed

        Serial.println("Increasing Speed");
        if (speed < EndSpeed) {
            speed += 1;
        } else { // At the end speed so reset
            speed = startSpeed;
        }

        // Set the speed of the motor
        motors_setSpeed(motor, speed);

        // Reset the start time
        startTime = millis();
    }

    return speed;
}


/**
 * @brief Set the speed of a motor
 * 
 * @param motor The motor to set the speed of
 * @param speed The speed to set the motor to
 * 
 * @return success (0) or failure (1)
 */
bool motors_setSpeed(uint8_t motor, int16_t speed) {
    // Check to see if the speed is in range
    if (speed > MOTOR_SPEED_MAX || speed < MOTOR_SPEED_MIN) {
        return 1;
    }

    // Invert motor 2 speed
    if (motor == MOTOR_2) {
      speed *= -1;
    }

    // Convert the speed to 1050-1950
    speed = map(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX, 1050, 1950);

    // Set the speed of the motor
    if (motor == MOTOR_1) {
        M1.writeMicroseconds(speed);
    } else if (motor == MOTOR_2) {
        M2.writeMicroseconds(speed);
    } else {
        return 1;
    }

    return 0;
}




