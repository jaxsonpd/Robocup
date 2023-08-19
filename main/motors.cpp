/** 
 * @file motors.cpp
 * @brief Motor control functions for the robot cup project
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @date 2023-06-03
 */


// ===================================== Includes =====================================
#include <Arduino.h>
#include <Servo.h>

#include "motors.hpp"
#include "robotInformation.hpp"
#include "src/dcMotor.hpp"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
// ** DC Motor parameters **
#define MOTOR_1_PIN 7
#define MOTOR_2_PIN 8

#define P_CONTROL_GAIN 75 //  /100
#define I_CONTROL_GAIN 0 //  /100
#define D_CONTROL_GAIN 0 //  /100
#define GAIN_SCALING 100

#define SETPOINT_TOLERANCE 5 // Max error to recognise at setpoint
#define SETPOINT_TIME 100 // Time before registering at setpoint

#define MAX_HEADING 180
#define MIN_HEADING -179
#define HEADING_OFFSET 360

#define MS_TO_S 1000

// ===================================== Objects ======================================
dcMotor leftMotor;
dcMotor rightMotor;

// ===================================== Globals ======================================
int16_t motor1Speed = 0; // Speed of motor 1 (-100 to 100)
int16_t motor2Speed = 0; // Speed of motor 2 (-100 to 100)
int16_t headingSP = 0;

// ===================================== Function Definitions =========================
/**
 * @brief Setup the motors for use
 * 
 * @return success (0) or failure (1)
 */
bool motors_setup(void) {
    // Attach the motors to each servo object
    if (!leftMotor.init(MOTOR_1_PIN, 0)) {
        Serial.println("Failed to initialise left motor!");
        return 1;
    }

    if (!rightMotor.init(MOTOR_2_PIN, 1)) {
        Serial.println("Failed to initialise right motor!");
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
    robotInfo->leftMotorSpeed = leftMotor.getSpeed();
    robotInfo->rightMotorSpeed = rightMotor.getSpeed();
    robotInfo->targetHeading = headingSP;

    // Check to see if the robot is at the setpoint
    static elapsedMillis timeAtSetpoint = 0;

    if (abs(robotInfo->targetHeading - robotInfo->IMU_Heading) <= SETPOINT_TOLERANCE) {
        if (timeAtSetpoint > SETPOINT_TIME) {
            robotInfo->atHeading = true;
        }
    } else {
        robotInfo->atHeading = false;
        timeAtSetpoint = 0;
    }


}


/** 
 * @brief Make the robot move to the heading setpoint
 * @param robotInfo The robotInfo struct to update
 * @param headingSetpoint The heading setpoint to follow
 * @param speed The speed to move at (0 is rotate on the spot, 100 is full speed)
 * 
 * @return 0 if at setpoint, 1 if not at setpoint
 */
bool motors_followHeading(RobotInfo_t *robotInfo, int16_t headingSetpoint, int16_t speed) {
    static int32_t errorInt = 0;
    static int32_t errorPrev = 0;
    static elapsedMillis deltaT = 0;
    
    // Calculate the error
    int32_t error = headingSetpoint - robotInfo->IMU_Heading;

    // Ensure error is pointing the right way

    if (error >= MAX_HEADING) {
      error -= HEADING_OFFSET;
    } else if (error < MIN_HEADING) {
      error += HEADING_OFFSET;
    }

    // Calculate the integral error
    errorInt += error * deltaT;

    // Calculate the derivative error
    int32_t errorDer = (error - errorPrev)/deltaT;
    errorPrev = error;

    // Reset the timer
    deltaT = 0;

    // Calculate the control value
    int32_t controlValue = (P_CONTROL_GAIN * error 
                            + (I_CONTROL_GAIN * errorInt)/MS_TO_S 
                            + (D_CONTROL_GAIN * errorDer)/MS_TO_S)
                            /GAIN_SCALING;
    
    // Calculate the motor speeds
    motor1Speed = speed + controlValue;
    motor2Speed = speed - controlValue;
    headingSP = headingSetpoint;

    // Check to see if the motor speeds are in range
    motor1Speed = (motor1Speed > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : motor1Speed;
    motor1Speed = (motor1Speed < MOTOR_SPEED_MIN) ? MOTOR_SPEED_MIN : motor1Speed;
    
    motor2Speed = (motor2Speed > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : motor2Speed;
    motor2Speed = (motor2Speed < MOTOR_SPEED_MIN) ? MOTOR_SPEED_MIN : motor2Speed;

    // Set the motor speeds
    leftMotor.setSpeed(motor1Speed);
    rightMotor.setSpeed(motor2Speed);

    // Check to see if the robot is at the setpoint !! Depreciated
    static elapsedMillis timeAtSetpoint = 0;

    if (abs(error) <= SETPOINT_TOLERANCE) {
        if (timeAtSetpoint > SETPOINT_TIME) {
            return 0;
        }
    } else {
        timeAtSetpoint = 0;
    }

    return 1;
}


/** 
 * @brief Make the robot move in a shape
 * @param robotInfo the robot information struct
 * @param sideLength the lenght in seconds of each side to complete
 * @param rotationAngle the angle to rotate at each corner
 * 
 * @return 0 if at setpoint, 1 if not at setpoint
 */
bool motors_formShape(RobotInfo_t *robotInfo, uint32_t sideLenght, int16_t rotationAngle) {
    static uint8_t state = 0;
    static elapsedMillis timeAtState;
    static int16_t targetHeading = 0;
    bool atSetpoint = 0;

    switch (state) {
        case 0: // Move forward
            atSetpoint = motors_followHeading(robotInfo, targetHeading, 40);
            if (timeAtState > sideLenght) {
                state = 1;
                timeAtState = 0;
                targetHeading += rotationAngle;

                if (targetHeading > MAX_HEADING) {
                    targetHeading -= 360;
                }
            }
            break;
        case 1: // Turn 90 degrees
            atSetpoint = motors_followHeading(robotInfo, targetHeading, 0);
            if (atSetpoint == 0) {
                state = 0;
                timeAtState = 0;
            }
            break;
    }

    return atSetpoint;
}


/**
 * @brief de-initialise the motors
 * 
 * @return success (1) or failure (0)
 */
bool motors_deInit(void) {
    // De-initialise the motors
    if (!leftMotor.deInit()) {
        Serial.println("Failed to de-initialise left motor!");
        return 0;
    }

    if (!rightMotor.deInit()) {
        Serial.println("Failed to de-initialise right motor!");
        return 0;
    }

    return 1;
}
