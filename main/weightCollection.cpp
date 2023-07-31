/** 
 * @file weightCollection.cpp
 * @brief Weight collection functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-14
 */


// ===================================== Includes =====================================
#include <arduino.h>

#include "weightCollection.hpp"
#include "robotInformation.hpp"
#include "motors.hpp"

#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================
enum states {
    ROTATING,
    MOVE_TO_OPEN,
    MOVE_TO_WEIGHT,
};

#define ROTATION_OFFSET 30
#define MOVE_SPEED 30
#define MOVE_TIME 1000

#define WEIGHT_DIFFERENCE_THRESHOLD 100
#define WEIGHT_CLOSE_DISTANCE 100
#define HEADING_THRESHOLD 2

// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
void findWeights(RobotInfo_t *robotInfo) {
    static uint8_t state = 0;
    static bool firstRun = true;
    static elapsedMillis moveTimer = 0;
    static bool atWeight = 0;

    static int16_t weightHeading = 0;
    static int16_t startHeading = 0;
    static int16_t mostOpenHeading = 0;
    static int16_t mostOpenDistance = 0;

    switch(state) {
        case ROTATING: // Rotate a 360 to look for weights
            if (firstRun) {
                startHeading = robotInfo->IMU_Heading;
                firstRun = false;
            }
            
            // Check for weight
            if (abs(robotInfo->IRTop_Distance - robotInfo->IRBottom_Distance) > WEIGHT_DIFFERENCE_THRESHOLD) {
                // Found a weight
                weightHeading = robotInfo->IMU_Heading;
                state = MOVE_TO_WEIGHT; 
                firstRun = true;
            } else if (abs(robotInfo->IMU_Heading - startHeading) > HEADING_THRESHOLD) { // Check if we have rotated 360
                // We have rotated 360
                state = MOVE_TO_OPEN;
                firstRun = true;
            } else {
                // Continue rotating
                motors_followHeading(robotInfo, robotInfo->IMU_Heading + ROTATION_OFFSET, 0);
            }

            // Update the most open area heading
            if (robotInfo->IRTop_Distance > mostOpenDistance) {
                mostOpenDistance = robotInfo->IRTop_Distance;
                mostOpenHeading = robotInfo->IMU_Heading;
            }

            break;

        case MOVE_TO_OPEN: // Move to the most open area
            if (firstRun) { // Reset the timer on first run
                moveTimer = 0;
                firstRun = false;
            }

            if (moveTimer > MOVE_TIME) {
                state = ROTATING;
                firstRun = true;
            } else {
                motors_followHeading(robotInfo, mostOpenHeading, MOVE_SPEED);
            }
            break;

        case MOVE_TO_WEIGHT: // Move to the weight
            if (robotInfo->IRBottom_Distance < WEIGHT_CLOSE_DISTANCE) {
                // We are close to the weight
                // Trigger weight collection    
                atWeight = true;
            } 

            if (atWeight & robotInfo->IRBottom_Distance > WEIGHT_CLOSE_DISTANCE) {
                // Weight as been collected/removed so move on
                firstRun = True;
                state = ROTATING;
            } else {
                motors_followHeading(robotInfo, weightHeading, MOVE_SPEED);
            }
            break;
    }
}


