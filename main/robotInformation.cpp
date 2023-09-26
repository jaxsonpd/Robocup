/** 
 * @file robotInfo.cpp
 * @brief Filoe for the robotInfo module
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-16
 */


// ===================================== Includes =====================================
#include <Arduino.h>

#include "robotInformation.hpp"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>


// ===================================== Types/Constants ==============================


// ===================================== Globals ======================================


// ===================================== Function Definitions =========================     
/** 
 * @brief print the robot info to the serial monitor
 * @param robotInfo the robot info struct to print
 * 
 */
void printRobotInfo(RobotInfo_t* robotInfo) {
    // Print the robot info
    char buffer[150];

    sprintf(buffer, "M: L %2d%%, R %2d%%, US: L %4d, R %4d, IR: T %4d, B %4d, H %3d d, SP %3d d, %1d, M: %1d",
        robotInfo->leftMotorSpeed,
        robotInfo->rightMotorSpeed,

        robotInfo->USLeft_Distance,
        robotInfo->USRight_Distance,

        robotInfo->IRTop_Distance,
        robotInfo->IRBottom_Distance,
        
        robotInfo->IMU_Heading,
        robotInfo->targetHeading,

        robotInfo->atHeading,
        robotInfo->mode);


    Serial.print(buffer);
    Serial.print(", Ax: ");
    Serial.print(robotInfo->forwardAcceleration);
    Serial.print(", Az: ");
    Serial.print(robotInfo->rotationAcceleration);
    Serial.print("\n");

    char Serial1Buffer[150];
    sprintf(Serial1Buffer, "IR T: %4d, B: %4d, M: %1d, IMU: %3d, %3d \n", robotInfo->IRTop_Distance, robotInfo->IRBottom_Distance, robotInfo->mode, robotInfo->IMU_Heading, robotInfo->targetHeading);

    Serial1.write(Serial1Buffer);
    
}  