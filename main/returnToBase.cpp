/** 
 * @file return_to_base.cpp
 * @brief Return to base functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-14
 */


// ===================================== Includes =====================================
#include <arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "returnToBase.hpp"
#include "robotInformation.hpp"
#include "sensors.hpp"
#include "motors.hpp"


// ===================================== Types/Constants ==============================
#define STATE_OFFSET 10 // Offset for state reporting
#define TURN_DISTANCE 30 // The distance ta wall has to be to trigger a turn
#define US_WALL_DISTANCE 30 // Maxiumum wall distance for ROATE state to not move into HEAD_HOME
#define ROTATE_SPEED 30 // The degrees offset used to rotate the robot
#define US_TOLERANCE 10 // The uncertainty of the ultrasonic sensor

enum states {
    HEAD_HOME,
    HUG_WALL,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    ROTATE,
    AT_HOME,
};

enum wallSide {
    UNKOWN,
    LEFT,
    RIGHT,
};

// ===================================== Globals ======================================
static uint8_t state = 0;
static int16_t baseHeading = 135;
static bool firstRun = true; // Flag to indicate if the state has just been entered

// ===================================== Function Definitions =========================
/**
 * @brief Return to base state machine 
 * @param robotInfo Pointer to the robot information struct
 */
void returnToBase(RobotInfo_t* robotInfo) {
    // Variable definitions
    static int16_t targetHeading = 0; // The heading try to follow

    static uint16_t previousUSDistance = 10000; // The previous ultrasonic distance
    static uint8_t wallSide = 0; // The side of the wall the robot is following  

    // State machine
    switch (state) {
        case HEAD_HOME: // Move towards the home base if there are no walls in the way
            if (firstRun) {
                firstRun = false; // Reset the first run flag
            }

            // ** State machine update **
            // Check if there is a wall in front of the robot
            if (robotInfo->IRTop_Distance < TURN_DISTANCE) { // Object detected
                state = ROTATE_RIGHT;
                firstRun = true;
            } else {
                motors_followHeading(robotInfo, baseHeading, 30);
            }
            break;
        
        case HUG_WALL: // Follow the wall
            if (firstRun) {
                firstRun = false; // Reset the first run flag
                targetHeading = robotInfo->IMU_Heading; // Set the target heading to the current heading
            }

            // Check if the robot is parallel to the wall and correct if it is not
            if (wallSide == LEFT & robotInfo->USLeft_Distance > previousUSDistance + US_TOLERANCE) { // Drifting away
                targetHeading -= 1;
            } else if (wallSide == LEFT & robotInfo->USLeft_Distance < previousUSDistance - US_TOLERANCE) { // Drifting towards
                targetHeading += 1;
            } else if (wallSide == RIGHT & robotInfo->USRight_Distance > previousUSDistance + US_TOLERANCE) { // Drifting away
                targetHeading += 1;
            } else if (wallSide == RIGHT & robotInfo->USRight_Distance < previousUSDistance - US_TOLERANCE) { // Drifting towards
                targetHeading -= 1;
            }


            // ** State machine update **
            // Check if there is a wall in front of the robot
            if (wallSide == LEFT & robotInfo->IRTop_Distance < TURN_DISTANCE) { // Object detected
                state = ROTATE_RIGHT;
                targetHeading = targetHeading + 90; // Rotate by 90 degrees
                
                // Bound Heading
                if (targetHeading > 180) {
                    targetHeading -= 360;
                } else if (targetHeading < -179) {
                    targetHeading += 360;
                }

                firstRun = true;
            } else if (wallSide == RIGHT & robotInfo->IRTop_Distance < TURN_DISTANCE) { // Object detected
                state = ROTATE_LEFT;
                targetHeading = targetHeading - 90; // Rotate by 90 degrees
                
                // Bound Heading
                if (targetHeading > 180) {
                    targetHeading -= 360;
                } else if (targetHeading < -179) {
                    targetHeading += 360;
                }
                firstRun = true;
            } else {
                motors_followHeading(robotInfo, targetHeading, 30);
            }

            break;

        case ROTATE_LEFT: // Rotate left until the robot is facing the wall
            if (firstRun) {
                firstRun = false; // Reset the first run flag
                previousUSDistance = robotInfo->USLeft_Distance;
            }

            // ** State machine update **
            // Check if the robot is parallel to the wall
            if (robotInfo->USLeft_Distance > previousUSDistance) { // The robot is parallel to the wall
                state = HUG_WALL;
                wallSide = LEFT;
                firstRun = true;
            } else {
                // Rotate the robot on the spot left
                motors_rotate(robotInfo, robotInfo->IMU_Heading - ROTATE_SPEED, 0);
            }
            previousUSDistance = robotInfo->USLeft_Distance;
            break;

        case ROTATE_RIGHT: // Rotate right until the robot is facing the wall
            if (firstRun) {
                firstRun = false; // Reset the first run flag
                previousUSDistance = robotInfo->USRight_Distance;
            }
            
            // ** State machine update **
            // Check if the robot is parallel to the wall
            if (robotInfo->USRight_Distance > previousUSDistance) { // The robot is parallel to the wall
                state = HUG_WALL;
                wallSide = RIGHT;
                firstRun = true;
            } else {
                // Rotate the robot on the spot right
                motors_rotate(robotInfo, robotInfo->IMU_Heading + ROTATE_SPEED, 0);
            }
            previousUSDistance = robotInfo->USRight_Distance;
            break;

        case ROTATE: // Rotate to the target heading
            if (firstRun) {
                firstRun = false;
            }

            // ** State machine update **
            if (robotInfo->atHeading) { // Completed Turn
                if (robotInfo->USLeft_Distance < US_WALL_DISTANCE) { // Check if the robot is close to a wall
                    wallSide = LEFT;
                    state = HUG_WALL;
                    firstRun = true;
                } else if (robotInfo->USRight_Distance < US_WALL_DISTANCE) { // Check if the robot is close to a wall
                    wallSide = RIGHT;
                    state = HUG_WALL;
                    firstRun = true;
                } else { // No wall detected
                    state = HEAD_HOME;
                    firstRun = true;
                }            
            } else {
                motors_rotate(robotInfo, targetHeading, 30);
            }
        
            break;
    }

    robotInfo->mode = state + STATE_OFFSET; // Update the robot mode
}

/**
 * @brief Initialise the return to base state machine
 * 
 */
void returnToBase_init(RobotInfo_t* robotInfo) {
    state = HEAD_HOME;
    robotInfo->mode = state + STATE_OFFSET; // Update the robot mode
    firstRun = true;
}
