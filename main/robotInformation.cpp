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
    char buffer[100];

    sprintf(buffer, "Robot Info: %2d\%, %2d\%, US: %4d mm, %4d mm, IR: %4d mm , %4d mm %3d deg\n",
        robotInfo->leftMotorSpeed,
        robotInfo->rightMotorSpeed,

        robotInfo->USLeft_Distance,
        robotInfo->USRight_Distance,

        robotInfo->IRTop_Distance,
        robotInfo->IRBottom_Distance,
        
        robotInfo->IMU_Heading);

    Serial.print(buffer);
}  