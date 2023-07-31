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
  FIND_WEIGHT
};

// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
void findWeights(RobotInfo_t *robotInfo) {
  static uint8_t state = 0;
  static int16_t targetHeading = 0;
  
  switch(state) {
    case ROTATING:
      motors_followHeading(robotInfo, robotInfo->IMU_Heading + 30, 0);
      if (abs(robotInfo->IRTop_Distance - robotInfo->IRBottom_Distance) > 100) {
        // Found a weight
        targetHeading = robotInfo->IMU_Heading;
        state = 1;
        
      }
      break;
    case FIND_WEIGHT:
      motors_followHeading(robotInfo, targetHeading, 30);
      break;
  }

  Serial.print(state);
}


