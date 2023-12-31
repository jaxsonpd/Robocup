/** 
 * @file robotInfo.h
 * @brief Contains the robotInfo struct 
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-07-16
 */


#ifndef ROBOTINFO_H
#define ROBOTINFO_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================

// RobotInfo struct to hold all the sensor data
typedef struct {
    // Motor speeds
    int8_t leftMotorSpeed; // Left motor speed (-100 to 100)
    int8_t rightMotorSpeed; // right motor speed (-100 to 100)

    // Sensors
    // US
    uint16_t USLeft_Distance; // Ultrasonic distance in mm
    uint16_t USRight_Distance; // Ultrasonic distance in mm

    // IR
    uint16_t IRTop_Distance; // Weight target IR distance in mm
    uint16_t IRBottom_Distance; // Weight target IR distance in mm

    // IMU
    int16_t IMU_Heading; // IMU heading in degrees
    int16_t targetHeading; // Target heading in degrees
    double forwardAcceleration; // Forward acceleration in m/s^2
    double rotationAcceleration; // Rotation acceleration in m/s^2

    // General Info
    bool atHeading; // Whether the robot is at the target heading for more then 100ms
    uint8_t mode; // Current mode the robot is in
    uint8_t weightsOnBoard; // Number of weights on board
    uint8_t colorOver; // The color the robot is currently over
    uint8_t homeBase; // Which home base the robot has started at

} RobotInfo_t;


// ===================================== Function Prototypes ==========================
void printRobotInfo(RobotInfo_t* robotInfo);

#endif // ROBOTINFO_H