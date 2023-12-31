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
#include "collector.hpp"
#include "motors.hpp"

#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================
#define ROTATION_SPEED 25 // Speed during rotation step in %
#define MOVE_SPEED 50 // Speed during move phase in % 
#define MOVE_TIME 3000 // Time to move to open area in ms

#define WEIGHT_DIFFERENCE_THRESHOLD 100 // Difference between IR sensors to detect weight
#define BOTTOM_SENSOR_OFFSET 40 // Offset for the bottom sensor to account for the difference in mounting postion

#define WEIGHT_CLOSE_DISTANCE 100 // Distance to weight to trigger weight collection
#define OBSTICAL_CLOSE_DISTANCE 400 // Distance to obstical to trigger rotation

#define MAX_DETECTION_RANGE 950 // Maximum range that weights can be detected at

#define HEADING_THRESHOLD 9 // Threshold for heading to be considered the same

#define NUM_DETECTIONS 2 // Number of times the weight must be detected before it is collected

// ===================================== Globals ======================================
enum states {
    ROTATING,
    MOVE_TO_OPEN,
    MOVE_TO_WEIGHT,
    AT_WEIGHT,
    GET_OUT
};

static uint8_t state = GET_OUT;

int16_t mostOpenHeading = 0; // Heading of the most open area

// ===================================== Function Definitions =========================
/** 
 * @brief Check to see if a weight has been detected
 * @param topDistance distance read from the top sensor
 * @param bottomDistance distance read from the bottom sensor
 * 
 * @return true if a weight has been detected
 */
static bool weightDetected(uint32_t topDistance, uint32_t bottomDistance) {
    if (topDistance > (bottomDistance + BOTTOM_SENSOR_OFFSET)) { // Sensors are correct 
        if ((topDistance - (bottomDistance + BOTTOM_SENSOR_OFFSET)) > WEIGHT_DIFFERENCE_THRESHOLD) { // Weight detected
            if (bottomDistance < MAX_DETECTION_RANGE) { // Within detection range
                return true;
            }
        }
    }

    return false;
}

void setMostOpen(RobotInfo_t* robotInfo) {
    mostOpenHeading = robotInfo->IMU_Heading;
}


/**
 * @brief Find weights and collect them
 * @param robotInfo the robot information struct
 */
void findWeights(RobotInfo_t *robotInfo) {
    // Variables
    static bool firstRun = true; // True if first run in state
    static elapsedMillis moveTimer = 0; // Timer for moving to open area
    static uint8_t weightDetectionOccurances = 0; // Counter for weight detection 
    static bool Actually_first_time = true;

    static int16_t weightHeading = 0; // Heading of the weight
    static uint16_t weightDistance = UINT16_MAX; // Distance of the weight

    static int16_t startHeading = 0; // Heading at start of rotation

    static int16_t mostOpenDistance = 0; // Distance of the most open area
    static int16_t mostOpenHeadingPrevInv = 180; // Previous heading of the most open area

    // State machine
    switch(state) {
        case ROTATING: // Rotate on the spot and look for a weight
            if (firstRun) { // Setup the state
                firstRun = false;
                startHeading = robotInfo->IMU_Heading - 15; // Set the start heading to 2 degrees less than the current heading
                if (startHeading < -179) {
                    startHeading += 360;
                }
                moveTimer = 0; // Reset the move timer
                mostOpenDistance = 0;
            }

            robotInfo->mode = state;

            // Check for weight
            weightDetectionOccurances = weightDetected(robotInfo->IRTop_Distance, robotInfo->IRBottom_Distance) 
                	                    ? weightDetectionOccurances + 1 : 0;


            // Update the most open area heading
            if (robotInfo->IRTop_Distance > mostOpenDistance) {
                if (abs(robotInfo->IMU_Heading - mostOpenHeadingPrevInv) > 5) { // Robot is not going back were it came from
                    mostOpenHeading = robotInfo->IMU_Heading;
                    mostOpenDistance = robotInfo->IRTop_Distance;
                }
            }


            // Update the state machine
            if (weightDetectionOccurances >= NUM_DETECTIONS) { // Check to see if we should pick up the weight
                motors_setLeft(0);
                motors_setRight(0);

                // Weight has been detected enough times so collect it
                state = MOVE_TO_WEIGHT; // Move to the weight
                firstRun = true;

                weightHeading = robotInfo->IMU_Heading; // Set the weight heading
                weightDistance = robotInfo->IRBottom_Distance; // Set the weight distance

                weightDetectionOccurances = 0;

                Serial.print("Detected at: ");
                Serial.println(weightHeading);

            } else if (abs(robotInfo->IMU_Heading - startHeading) < HEADING_THRESHOLD) {  // Check if we have rotated 360
                // We have rotated 360
                state = MOVE_TO_OPEN; // Move to the most open area
                firstRun = true;
                
                mostOpenHeadingPrevInv = mostOpenHeading + 180;

                if (mostOpenHeadingPrevInv > 180) {
                    mostOpenHeadingPrevInv -= 360;
                }
            } else {  // Rotate on the spot
                motors_setLeft(ROTATION_SPEED);
                motors_setRight(-ROTATION_SPEED);
            }

            break;

        case MOVE_TO_OPEN: // Move to the most open area
            if (firstRun) { // Setup the state
                moveTimer = 0;
                firstRun = false;
            }

            robotInfo->mode = state;

            // Update the state machine
            if (moveTimer > MOVE_TIME) { // Check if we have moved for long enough
                state = ROTATING; // Rotate on the spot
                firstRun = true;
            } else if (robotInfo->IRTop_Distance < OBSTICAL_CLOSE_DISTANCE) { // Check if obstacles infront of the robot
                state = ROTATING; // Rotate on the spot
                firstRun = true;

            } else { // Move to the most open area
                motors_followHeading(robotInfo, mostOpenHeading, MOVE_SPEED);
            }

            break;

        
        case GET_OUT: // Get out of the home base
            if (firstRun) {
                firstRun = false;
                moveTimer = 0;
            }

            if (moveTimer > 1000) {
                state = ROTATING;
                firstRun = true;
            } else {
                motors_setLeft(80);
                motors_setRight(80);
            }

            break;
        
        case MOVE_TO_WEIGHT: // Move to the weight
            if (firstRun) { // Setup the state
                moveTimer = 0;
                firstRun = false;
            }

            robotInfo->mode = state;

            // Update the state machine
            if (robotInfo->IRBottom_Distance < WEIGHT_CLOSE_DISTANCE) { // Check if we are close to the weight
                state = AT_WEIGHT; // We are at the weight
                firstRun = true;
            } else if (moveTimer > (weightDistance*4)) {
                state = AT_WEIGHT; // We are at the weight
                firstRun = true;
            } else { // Move to the weight
                motors_followHeading(robotInfo, weightHeading, 40);
            }
            
            break;
        
        case AT_WEIGHT: // Collect the weight
            if (firstRun) { // Setup the state
                moveTimer = 0;
                firstRun = false;
                motors_setLeft(0);
                motors_setRight(0);
            }

            robotInfo->mode = state;

            // Check if weight has been removed 
            uint8_t moveOnToNext = weightCollect(robotInfo);

            if (moveOnToNext) {
                state = ROTATING; // Rotate on the spot
                if (moveOnToNext == 2) { // Weight was not real
                    startHeading = robotInfo->IMU_Heading - 40; // Set the end heading to make a blank spot were the fake weight is
                    if (startHeading < -179) {
                        startHeading += 360;
                    }
                    moveTimer = 0; // Reset the move timer
                    mostOpenDistance = 0;
                } else {
                    firstRun = true;
                }
            }
            break;
    }
}


/**
 * @brief Reset the weight collection state machine
 * 
 */
void weightCollection_deInit(RobotInfo_t *robotInfo) {
    if (state == AT_WEIGHT) {
        collector_deInit();
    }
    state = GET_OUT;
    robotInfo->mode = state;
}