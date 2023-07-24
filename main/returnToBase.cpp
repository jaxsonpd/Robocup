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

#include "returnToBase.hpp"
#include "sensors.hpp"
#include "motors.hpp"


// ===================================== Types/Constants ==============================


// ===================================== Globals ======================================

#define homeHeading 0
#define turnRight 90
#define turnLeft -90
#define testSpeed 60
static enum homingState {headHome = 0, hugWall,hugLeft, hugRight, home}

// ===================================== Function Definitions =========================

void homeReturn(RobotInfo_t* robotInfo) {
    switch (homingState)
    {
    case headHome:
        /* code */
        motors_followHeading(&robotInfo, headHome, testSpeed);
        if (wallHit()) { //Make function for detecting wall hit

        }
        break;
    case hugWall:
        if (wallHit()) { 
            //turn away from hugged wall
         
        } else if (wallEnds()) { // large sudden distance between wall and robot - wall ends
          
            //Drive x then Turn toward hugged wall
        }
        


        break;
    // case hugRight:

    //     break;  
    case home:
        //unloaded weight sequence
        //reutrn to search algorithm
    }
    if (homeFound()) {
        homingState = home;
    }
}

