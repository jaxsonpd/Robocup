/** 
 * @file robotInfo.c
 * @brief Filoe for the robotInfo module
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-16
 */


// ===================================== Includes =====================================
#include <Arduino.h>

#include "robotInfo.h"

#include <stdint.h>
#include <stdbool.h>


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
    Serial.printf("Robot Info: %d\%, %d\%, %d mm, %d mm, %d mm , %d mm %d deg\n",
        robotInfo->leftMotorSpeed,
        robotInfo->rightMotorSpeed,

        robotInfo->USLeft_Distance,
        robotInfo->USRight_Distance,

        robotInfo->IRTop_Distance,
        robotInfo->IRBottom_Distance,
        
        robotInfo->IMU_Heading);
}       

