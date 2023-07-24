/** 
 * @file return_to_base.cpp
 * @brief Return to base functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-14
 */


// ===================================== Includes =====================================
#include <arduino.h>

#include "returnToBase.hpp"
#include "sensors.hpp"
#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================


// ===================================== Globals ======================================

#define homeHeading 0
#define turnRight 90
#define turnLeft -90
static enum homingState {headHome = 0, hugLeft, hugRight, home}

// ===================================== Function Definitions =========================

void homeReturn(RobotInfo_t* robotInfo) {
    switch (homingState)
    {
    case headHome:
        /* code */
        break;
    case hugLeft:

        break;
    case hugRight:

        break;  
    case home:

    }
    

}

