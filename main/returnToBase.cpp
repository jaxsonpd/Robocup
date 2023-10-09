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
#define TURN_DISTANCE 300 // The distance ta wall has to be to trigger a turn
#define US_WALL_DISTANCE 100 // Maxiumum wall distance for ROATE state to not move into HEAD_HOME

#define ROTATE_SPEED 30 // The degrees offset used to rotate the robot
#define US_TOLERANCE 10 // The uncertainty of the ultrasonic sensor
#define WALL_ADJ_DIST 400 //Distance from wall to determine which way to turn


enum states {
    HEAD_HOME,
    HUG_WALL,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    ROTATE,
    AT_HOME,
};

enum wallSide {
    UNKNOWN,
    LEFT,
    RIGHT,
};

// ===================================== Globals ======================================
static uint8_t state = 0;
 int16_t baseHeading;
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
    static int8_t wallDetections = 0; 
    Serial.print(state);
    Serial.print(" ");
    Serial.print(robotInfo->IMU_Heading);
    Serial.print(" ");
    Serial.print(targetHeading);
    Serial.print(" ");
    Serial.println(robotInfo->IRTop_Distance);
    // State machine
    switch (state) {
        case HEAD_HOME: // Move towards the home base if there are no walls in the way
            if (firstRun) {
                firstRun = false; // Reset the first run flag
            }

            // ** State machine update **
            // Check if there is a wall in front of the robot
            if (robotInfo->IRTop_Distance < TURN_DISTANCE) { // Object detected
              wallDetections++;
              if (wallDetections >= 3) {
                firstRun = true;
                state = HUG_WALL;
                wallSide = LEFT;
                if (robotInfo->USLeft_Distance < WALL_ADJ_DIST) { // Check if the robot is close to a wall
                    wallSide = LEFT;
                    state = HUG_WALL;

                } else if (robotInfo->USRight_Distance < WALL_ADJ_DIST) { // Check if the robot is close to a wall
                    wallSide = RIGHT;
                    state = HUG_WALL;

                }
              }
            } else {
              motors_followHeading(robotInfo, baseHeading, 20);
            }
            break;
        
        case HUG_WALL: // Follow the wall
            if (firstRun) {
                firstRun = false; // Reset the first run flag
                targetHeading = baseHeading; //robotInfo->IMU_Heading; // Set the target heading to the current heading
            }

            if (robotInfo->USLeft_Distance < WALL_ADJ_DIST) { // Check if the robot is close to a wall
                wallSide = LEFT;
            } else if (robotInfo->USRight_Distance < WALL_ADJ_DIST) { // Check if the robot is close to a wall
                    wallSide = RIGHT;
            }
          
            // ** State machine update **
            // Check if there is a wall in front of the robot
            if (abs(robotInfo->IMU_Heading - targetHeading) < 8) {
              if (robotInfo->IRTop_Distance < TURN_DISTANCE) { // Object detected
                wallDetections++;
                if (wallDetections >= 3) {
                  if (wallSide == LEFT) {
                    //state = ROTATE_RIGHT;
                    targetHeading = targetHeading + 90; // Rotate by 90 degrees
                    //firstRun = true;
                  } else if (wallSide == RIGHT) { // Object detected
                      //state = ROTATE_LEFT;
                      targetHeading = targetHeading - 90; // Rotate by 90 degrees
                  } 
                  if (targetHeading > 180) {
                      targetHeading -= 360;
                  } else if (targetHeading < -179) {
                      targetHeading += 360;
                  }
                }
                } else {
                  wallDetections = 0;
                  /* When Wall Disappears head home */
                //   if (wallSide == LEFT && (robotInfo->USLeft_Distance > WALL_ADJ_DIST*2) && (targetHeading == 90)) {
                //     state = HEAD_HOME;  
                //   } else if (wallSide == RIGHT && (robotInfo->USRight_Distance > WALL_ADJ_DIST*2) && (targetHeading == -90)) {
                //     state = HEAD_HOME;  
                //   }
                }
            } else {
                motors_followHeading(robotInfo, targetHeading, 30);  
            } 
      
            motors_followHeading(robotInfo, targetHeading, 30);
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
    sensors_update();
    robotInfo->mode = state + STATE_OFFSET; // Update the robot mode
    firstRun = true;
    sensors_updateInfo(robotInfo);
    if (robotInfo->USRight_Distance < 100) {
      baseHeading = 180; //135
    } else if (robotInfo->USLeft_Distance < 100) {
      baseHeading = 180; //135 if working
    } else {
      baseHeading = 180;
    }
    Serial.println(baseHeading);
    Serial.println(robotInfo->USRight_Distance);

}
