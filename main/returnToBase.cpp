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


// ===================================== Globals ======================================

// #define homeHeading 0 //based on starting conditions and IMU orientation
// #define turnRight 90
// #define turnLeft -90
// #define testSpeed 60
// #define fullCircle 360
// #define halfCircle 180
// #define turnDist 300
// enum homingStates {headHome = 0, hugWall,hugLeft, hugRight, home};
// enum wallHugged {right = 90, left = -90};
// int16_t turnHeading;  // heading for turning around a corner
// int8_t wallHit = 1;

// int8_t wallEnd = 0;
// ===================================== Function Definitions =========================


// void sideHugged() { //determine what wall to hug based wall detected while heading home
//     // if (homeHeading) {

//     // }
// }

// int8_t homeFound() {
//     return 0;
// }

// void wallFollow(RobotInfo_t* robotInfo) {
//   if ((robotInfo->IRTop_Distance <=turnDist) && (robotInfo->IRBottom_Distance <= turnDist)) { //wall detected in front of robot
//     wallHit = 1;
//   //} else if (USSpike) {

//   }
// }



// void corner(int8_t cornerHeading) {
//     static int i;
//     if (cornerHeading == turnRight) {
//       i++;  
//     } else {
//       i--;
//     }
//     turnHeading += cornerHeading;
//     if (abs(i) >= 4) {
//       //island --bad
//     } 
    
//     if(turnHeading > halfCircle) {
//         turnHeading -= fullCircle;
//     } else if (turnHeading <= -halfCircle) {
//         turnHeading += fullCircle;
//     }


// }

// void homeReturn(RobotInfo_t* robotInfo) {
//     static int8_t homingState = headHome;
//     wallFollow(robotInfo);
//     switch (homingState)
//     {
//     case headHome:
//         /* code */
//         motors_followHeading(robotInfo, homeHeading, testSpeed);
//         if (wallHit) { //Make function for detecting wall hit
//         //Select wall to follow?
//           homingState = hugWall;
//         }
//         break;  
//     case hugWall:
//         if (wallHit) { 
//           //turn away from hugged wall
//           corner(turnRight); //temp
          
//         } else if (wallEnd) { // large sudden distance between wall and robot - wall ends
//           corner(-turnRight);
//           wallEnd = 0;
//             //Drive a little then Turn towards hugged wall
//         }
//         if (!motors_followHeading(robotInfo, turnHeading, 0)) {
//         //if ( abs(robotInfo->IMU_Heading - turnHeading) <=3) { //turn on the spot till within tolerance of new heading
//             motors_followHeading(robotInfo, turnHeading, testSpeed);   
//             wallHit = 0;
//         }
        


//         break;
//     // case:

//     //     break;  
//       // case home:
        
//         //unloaded weight sequence
//         //return to search algorithm
//     }
//     // if (homeFound()) {
//     //     homingState = "home";
//     // }
// }

#define STATE_OFFSET 10 // Offset for state reporting
#define TURN_DISTANCE 30 // The distance ta wall has to be to trigger a turn
#define US_WALL_DISTANCE 30 // Maxiumum wall distance for ROATE state to not move into HEAD_HOME
#define ROTATE_SPEED 30 // The degrees offset used to rotate the robot
#define US_TOLERANCE 10 // The uncertainty of the ultrasonic sensor


static uint8_t state = 0;
static int16_t baseHeading = 0;

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


/**
 * @brief Return to base state machine 
 * @param robotInfo Pointer to the robot information struct
 */
void returnToBase(RobotInfo_t* robotInfo) {
    // Variable definitions
    static int16_t targetHeading = 0; // The heading try to follow
    static bool firstRun = true; // Flag to indicate if the state has just been entered

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
            }

            // ** State machine update **
            // Check if the robot is parallel to the wall
            if (robotInfo->USLeft_Distance > previousUSDistance) { // The robot is parallel to the wall
                state = HUG_WALL;
                firstRun = true;
            } else {
                // Rotate the robot on the spot left
                motors_clearErrors();
                motors_followHeading(robotInfo, robotInfo->IMU_Heading - ROTATE_SPEED, 0);
            }
            break;

        case ROTATE_RIGHT: // Rotate right until the robot is facing the wall
            if (firstRun) {
                firstRun = false; // Reset the first run flag
            }
            
            // ** State machine update **
            // Check if the robot is parallel to the wall
            if (robotInfo->USRight_Distance > previousUSDistance) { // The robot is parallel to the wall
                state = HUG_WALL;
                firstRun = true;
            } else {
                // Rotate the robot on the spot right
                motors_clearErrors();
                motors_followHeading(robotInfo, robotInfo->IMU_Heading + ROTATE_SPEED, 0);
            }
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
                motors_followHeading(robotInfo, targetHeading, 0);
            }
        
            break;
    }
}