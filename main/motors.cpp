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
#include "src/circBuffer.hpp"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
// #define TUNE

// ** DC Motor parameters **
#define MOTOR_1_PIN 7
#define MOTOR_2_PIN 8

#define P_CONTROL_GAIN 65 //  /100
#define I_CONTROL_GAIN 20 //  /100
#define D_CONTROL_GAIN 20 //  /100

#define GAIN_SCALING 100
#define ERROR_INT_MAX 50000
#define DERIVATIVE_OFFSET 1000
#define DERIVATIVE_BUFFER_SIZE 3

#define SETPOINT_TOLERANCE 3 // Max error to recognise at setpoint
#define SETPOINT_TIME 100 // Time before registering at setpoint

#define MAX_HEADING 180
#define MIN_HEADING -179
#define HEADING_OFFSET 360

#define MS_TO_S 1000

// ===================================== Objects ======================================
dcMotor leftMotor;
dcMotor rightMotor;

circBuffer_t* derivativeBuffer = new circBuffer_t[DERIVATIVE_BUFFER_SIZE];

// ===================================== Globals ======================================
static int32_t intergralError = 0; // The integral of the error
static int32_t previousError = 0; // The previous error

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

    circBuffer_init(derivativeBuffer, DERIVATIVE_BUFFER_SIZE);

    return 0;
}


/**
 * @brief Calculate the control value for the motors
 * @param setpoint The setpoint to follow
 * @param heading The current heading of the robot
 * 
 * @return The control value for the motors
 */
static int16_t calcControlValue(int16_t setpoint, int16_t heading) {
    // Variables
    static elapsedMillis deltaT = 0; // The time since the last calculation
    
    // Calculate the error and bound it
    int32_t error = setpoint - heading;
    if (error >= MAX_HEADING) {
      error -= HEADING_OFFSET;
    } else if (error < MIN_HEADING) {
      error += HEADING_OFFSET;
    }

    // Calculate the integral of the error
    intergralError += error * deltaT;
    if (intergralError > ERROR_INT_MAX) {
        intergralError = ERROR_INT_MAX;
    } else if (intergralError < -ERROR_INT_MAX) {
        intergralError = -ERROR_INT_MAX;
    }

    // Calculate the derivative of the error offset by 1000 to avoid floating point
    int32_t derivativeError = (error - previousError) * DERIVATIVE_OFFSET / ((int32_t) deltaT);

    circBuffer_write(derivativeBuffer, derivativeError);

    int32_t derivativeFiltered = circBuffer_average(derivativeBuffer);    

    // Update the previous error and deltaT
    previousError = error;
    deltaT = 0;

    // Calculate the control value
    int32_t PTerm = (error * P_CONTROL_GAIN) / GAIN_SCALING;
    int32_t ITerm = (intergralError * I_CONTROL_GAIN) / GAIN_SCALING / MS_TO_S;
    int32_t DTerm = (derivativeFiltered * D_CONTROL_GAIN) / GAIN_SCALING;

    int32_t controlValue = PTerm + ITerm + DTerm;

    #ifdef TUNE
    Serial.print(heading);
    Serial.print(", ");
    Serial.print(PTerm);
    Serial.print(", ");
    Serial.print(ITerm);
    Serial.print(", ");
    Serial.print(DTerm);
    Serial.print(", ");
    Serial.println(controlValue);
    #endif // TUNE

    // Bound the control value

    return controlValue;
}


/**
 * @brief Make the robot move to the heading setpoint
 * @param robotInfo The robotInfo struct to update
 * @param headingSetpoint The heading setpoint to follow
 * @param forwardSpeed The speed to move at (0 is rotate on the spot, 100 is full speed)
 * 
 * @return true if at setpoint
 */
bool motors_followHeading(RobotInfo_t* robotInfo, int16_t headingSetpoint, int16_t forwardSpeed) {
    // Variables
    int32_t motor1SpeedRaw = 0;
    int32_t motor2SpeedRaw = 0;

    // Calculate the control value
    int16_t controlValue = calcControlValue(headingSetpoint, robotInfo->IMU_Heading);

    // Set the motor speeds
    motor1SpeedRaw = forwardSpeed + controlValue;
    motor2SpeedRaw = forwardSpeed - controlValue;

    // Bound the motor speeds
    motor1SpeedRaw = (motor1SpeedRaw > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : motor1SpeedRaw;
    motor1SpeedRaw = (motor1SpeedRaw < MOTOR_SPEED_MIN) ? MOTOR_SPEED_MIN : motor1SpeedRaw;

    motor2SpeedRaw = (motor2SpeedRaw > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : motor2SpeedRaw;
    motor2SpeedRaw = (motor2SpeedRaw < MOTOR_SPEED_MIN) ? MOTOR_SPEED_MIN : motor2SpeedRaw;

    // Set the motor speeds
    leftMotor.setSpeed(motor1SpeedRaw);
    rightMotor.setSpeed(motor2SpeedRaw);

    robotInfo->leftMotorSpeed = leftMotor.getSpeed();
    robotInfo->rightMotorSpeed = rightMotor.getSpeed();

    // Check to see if the robot is at the setpoint !! Depreciated
    static elapsedMillis timeAtSetpoint = 0;

    if (abs(robotInfo->targetHeading - robotInfo->IMU_Heading) <= SETPOINT_TOLERANCE) {
        if (timeAtSetpoint > SETPOINT_TIME) {
            robotInfo->atHeading = true;
        }
    } else {
        robotInfo->atHeading = false;
        timeAtSetpoint = 0;
    }

    return robotInfo->atHeading;
}

/**
 * @brief De initialise the motors
 * @param robotInfo The robotInfo struct to update
 * 
 */
void motors_deinit(RobotInfo_t* robotInfo) {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    robotInfo->leftMotorSpeed = 0;
    robotInfo->rightMotorSpeed = 0;
    robotInfo->targetHeading = 0;
    robotInfo->atHeading = false;

    leftMotor.deInit();
    rightMotor.deInit();

    delete derivativeBuffer;

    intergralError = 0;
    previousError = 0;
}


/**
 * @brief Clear the integral and derivative errors
 * 
 */
void motors_clearErrors(void) {
    intergralError = 0;
    previousError = 0;
    circBuffer_clear(derivativeBuffer);
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
