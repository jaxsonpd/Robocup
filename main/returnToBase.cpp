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

#define homeHeading 0 //based on starting conditions and IMU orientation
#define turnRight 90
#define turnLeft -90
#define testSpeed 60
#define fullCircle 360
#define halfCircle 180
enum homingStates {headHome = 0, hugWall,hugLeft, hugRight, home};
enum wallHugged {right = 90, left = -90};
int16_t turnHeading;  // heading for turning around a corner
int8_t wallHit = 1;

int8_t wallEnd = 0;
// ===================================== Function Definitions =========================


void sideHugged() { //determine what wall to hug based wall detected while heading home
    // if (homeHeading) {

    // }
}

int8_t homeFound() {
    return 0;
}

void wallFollow(RobotInfo_t* robotInfo) {
  if ((robotInfo->IRTop_Distance <=20) && (robotInfo->IRBottom_Distance <= 20)) { //wall detected in front of robot
    wallHit = 1;
  //} else if (USSpike) {

  }
}



void corner(int8_t cornerHeading) {
    static int i;
    if (cornerHeading == turnRight) {
      i++;  
    } else {
      i--;
    }
    turnHeading += cornerHeading;
    if (abs(i) >= 4) {
      //island --bad
    } 
    
    if(turnHeading > halfCircle) {
        turnHeading -= fullCircle;
    } else if (turnHeading <= -halfCircle) {
        turnHeading += fullCircle;
    }


}

void homeReturn(RobotInfo_t* robotInfo) {
    static int8_t homingState = headHome;
    wallFollow(robotInfo);
    switch (homingState)
    {
    case headHome:
        /* code */
        motors_followHeading(robotInfo, homeHeading, testSpeed);
        if (wallHit) { //Make function for detecting wall hit
        //Select wall to follow?
          homingState = hugWall;
        }
        break;  
    case hugWall:
        if (wallHit) { 
          //turn away from hugged wall
          corner(turnRight); //temp
          wallHit = 0;
        } else if (wallEnd) { // large sudden distance between wall and robot - wall ends
          corner(-turnRight);
          wallEnd = 0;
            //Drive a little then Turn towards hugged wall
        }
        if (!motors_followHeading(robotInfo, turnHeading, 0)) {
        //if ( abs(robotInfo->IMU_Heading - turnHeading) <=3) { //turn on the spot till within tolerance of new heading
            motors_followHeading(robotInfo, turnHeading, testSpeed);
        }
        


        break;
    // case hugRight:

    //     break;  
      // case home:
        
        //unloaded weight sequence
        //return to search algorithm
    }
    // if (homeFound()) {
    //     homingState = "home";
    // }
}

