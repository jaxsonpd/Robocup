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
#define ROTATION_SPEED 20
#define MOVE_SPEED 30
#define MOVE_TIME 10000

#define WEIGHT_DIFFERENCE_THRESHOLD 100
#define WEIGHT_CLOSE_DISTANCE 100
#define HEADING_THRESHOLD 8
#define OBSTICAL_CLOSE_DISTANCE 300
#define NUM_DETECTIONS 2 // Number of times the weight must be detected before it is collected

// ===================================== Globals ======================================
enum states {
    ROTATING,
    MOVE_TO_OPEN,
    MOVE_TO_WEIGHT,
    AT_WEIGHT
};

static uint8_t state = ROTATING;

// ===================================== Function Definitions =========================
/**
 * @brief Find weights and collect them
 * @param robotInfo the robot information struct
 */
void findWeights(RobotInfo_t *robotInfo) {
    // Variables
    static bool firstRun = true; // True if first run in state
    static elapsedMillis moveTimer = 0; // Timer for moving to open area
    static uint8_t weightDetectionOccurances = 0; // Counter for weight detection 

    static int16_t weightHeading = 0; // Heading of the weight
    static uint16_t weightDistance = UINT16_MAX; // Distance of the weight

    static int16_t startHeading = 0; // Heading at start of rotation

    static int16_t mostOpenHeading = 0; // Heading of the most open area
    static int16_t mostOpenDistance = 0; // Distance of the most open area

    // State machine
    switch(state) {
        case ROTATING: // Rotate on the spot and look for a weight
            if (firstRun) { // Setup the state
                firstRun = false;
                startHeading = robotInfo->IMU_Heading - 10; // Set the start heading to 2 degrees less than the current heading
                moveTimer = 0; // Reset the move timer
                mostOpenDistance = 0;
            }

            // Check for weight
            if ((robotInfo->IRTop_Distance > robotInfo->IRBottom_Distance) & ((robotInfo->IRTop_Distance - robotInfo->IRBottom_Distance) > WEIGHT_DIFFERENCE_THRESHOLD)) {
                // Found a weight 
                weightDetectionOccurances ++;
            } else {
                weightDetectionOccurances = 0;
            }


            // Update the most open area heading
            if (robotInfo->IRTop_Distance > mostOpenDistance) {
                mostOpenDistance = robotInfo->IRTop_Distance;
                mostOpenHeading = robotInfo->IMU_Heading;
            }


            // Update the state machine
            if (weightDetectionOccurances >= NUM_DETECTIONS) { // Check to see if we should pick up the weight
                // Weight has been detected enough times so collect it
                state = MOVE_TO_WEIGHT; // Move to the weight
                firstRun = true;
                weightHeading = robotInfo->IMU_Heading+5; // Set the weight heading
                weightDetectionOccurances = 0;
            } else if (abs(robotInfo->IMU_Heading - startHeading) < HEADING_THRESHOLD) {  // Check if we have rotated 360
                // We have rotated 360
                state = MOVE_TO_OPEN; // Move to the most open area
                firstRun = true;
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
        
        case MOVE_TO_WEIGHT: // Move to the weight
            if (firstRun) { // Setup the state
                moveTimer = 0;
                firstRun = false;
            }


            // Update the state machine
            if (robotInfo->IRBottom_Distance < WEIGHT_CLOSE_DISTANCE) { // Check if we are close to the weight
                state = AT_WEIGHT; // We are at the weight
                firstRun = true;
            } else { // Move to the weight
                motors_followHeading(robotInfo, weightHeading, MOVE_SPEED);
            }
            
            break;
        
        case AT_WEIGHT: // Collect the weight
            if (firstRun) { // Setup the state
                moveTimer = 0;
                firstRun = false;
            }

            // Check if weight has been removed 
            // !! THIS WILL CHANGE WHEN THE CRANE IS INSTALLED !!
            if (robotInfo->IRBottom_Distance > WEIGHT_CLOSE_DISTANCE) {
                state = ROTATING; // Rotate on the spot
                firstRun = true;
            } else { // Hold Position
                motors_setLeft(0);
                motors_setRight(0);
            }
            break;


    }

    robotInfo->mode = state;
}




// /**
//  * @brief Find the weights and collect them
//  * 
//  * @param robotInfo 
//  */
// void findWeights1(RobotInfo_t *robotInfo) {
//     static bool firstRun = true;
//     static elapsedMillis moveTimer = 0;

//     static int16_t weightHeading = 0;
//     static uint16_t weightDistance = 9000;
    
//     static int16_t startHeading = 0;

//     static int16_t mostOpenHeading = 0;
//     static int16_t mostOpenDistance = 0;

//     switch(state) {
//         case ROTATING: // Rotate a 360 to look for weights
//             if (firstRun) {
//                 startHeading = robotInfo->IMU_Heading - 2;
//                 firstRun = false;
//                 moveTimer = 0;
//             }
            
//             // Check for weight
//             if ((robotInfo->IRTop_Distance > robotInfo->IRBottom_Distance) & (abs(robotInfo->IRTop_Distance - robotInfo->IRBottom_Distance) > WEIGHT_DIFFERENCE_THRESHOLD)) {
//                 // Found a weight
//                 if (weightDistance + 2 < robotInfo->IRBottom_Distance) { // Actually looking at weight
//                   weightHeading = robotInfo->IMU_Heading;
//                   state = MOVE_TO_WEIGHT; 
//                   firstRun = true;
//                 } else { // Keep going
//                   weightDistance = robotInfo->IRBottom_Distance;
//                 }

//             } else if (abs(robotInfo->IMU_Heading - startHeading) < HEADING_THRESHOLD) { // Check if we have rotated 360
//                 // We have rotated 360
//                 state = MOVE_TO_OPEN;
//                 firstRun = true;
//             } 

//             motors_followHeading(robotInfo, robotInfo->IMU_Heading + ROTATION_OFFSET, 0);
  

//             // Update the most open area heading
//             if (robotInfo->IRTop_Distance > mostOpenDistance) {
//                 mostOpenDistance = robotInfo->IRTop_Distance;
//                 mostOpenHeading = robotInfo->IMU_Heading;
//             }

//             break;

//         case MOVE_TO_OPEN: // Move to the most open area
//             if (firstRun) { // Reset the timer on first run
//                 moveTimer = 0;
//                 firstRun = false;
//             }

//             if (moveTimer > MOVE_TIME) {
//                 state = ROTATING;
//                 firstRun = true;
//             } else {
//                 motors_followHeading(robotInfo, mostOpenHeading, MOVE_SPEED);
//             }
//             break;

//         case MOVE_TO_WEIGHT: // Move to the weight
//             if (robotInfo->IRBottom_Distance < WEIGHT_CLOSE_DISTANCE) {
//                 // We are close to the weight
//                 // Trigger weight collection    
//                 state = AT_WEIGHT;
//                 firstRun = true;
//                 motors_followHeading(robotInfo, weightHeading, 0);
//             } else {
//                 motors_followHeading(robotInfo, weightHeading, MOVE_SPEED);
//             }
 
//             break;
        
//         case AT_WEIGHT:
//             if (robotInfo->IRBottom_Distance > WEIGHT_CLOSE_DISTANCE) { // Weight has been collected
//                 state = ROTATING;
//                 firstRun = true;
//             } else {
//                 motors_followHeading(robotInfo, weightHeading, 0);
//             }
//     }
//     robotInfo->mode = state;
// }


/**
 * @brief Reset the weight collection state machine
 * 
 */
void weightCollection_deInit(RobotInfo_t *robotInfo) {
    state = ROTATING;
    robotInfo->mode = state;
}