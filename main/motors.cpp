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

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
// ** DC Motor parameters **
#define MOTOR_1_PIN 7
#define MOTOR_2_PIN 8

#define MOTOR_SPEED_MAX 100
#define MOTOR_SPEED_MIN -100

#define P_CONTROL_GAIN 70 //  /100
#define I_CONTROL_GAIN 10 //  /100
#define D_CONTROL_GAIN 0 //  /100
#define GAIN_SCALING 100
#define ERROR_INT_MAX 500000 

#define SETPOINT_TOLERANCE 3 // Max error to recognise at setpoint
#define SETPOINT_TIME 100 // Time before registering at setpoint

#define MAX_HEADING 180
#define MIN_HEADING -179
#define HEADING_OFFSET 360

#define MS_TO_S 1000

// ===================================== Objects ======================================
Servo M1, M2; // Define the servo objects for each motor

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
    bool inBound = true;

    // Check to see if the speed is in range
    if (speed > MOTOR_SPEED_MAX || speed < MOTOR_SPEED_MIN) {
        inBound = false;
        // speed = (speed > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : speed;
        // speed = (speed < MOTOR_SPEED_MIN) ? MOTOR_SPEED_MIN : speed;
    }

    // Invert motor 2 speed
    proccessedSpeed = (selectedMotor == MOTOR_1) ? -speed : speed;

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

    return inBound;
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

    // Limit the integral error
    errorInt = (errorInt > ERROR_INT_MAX) ? ERROR_INT_MAX : errorInt;
    errorInt = (errorInt < -ERROR_INT_MAX) ? -ERROR_INT_MAX : errorInt;


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
    motors_setSpeed(MOTOR_1, motor1Speed);
    motors_setSpeed(MOTOR_2, motor2Speed);

    // Check to see if the robot is at the setpoint
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
            atSetpoint = motors_followHeading(robotInfo, targetHeading, 30);
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



