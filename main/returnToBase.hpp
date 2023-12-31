/** 
 * @file return_to_base.h
 * @brief Return to base functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-14
 */


#ifndef RETURNTOBASE_H
#define RETURNTOBASE_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>
#include "robotInformation.hpp"

// ===================================== Types/Constants ==============================


// ===================================== Function Prototypes ==========================

void returnToBase(RobotInfo_t* robotInfo);
void returnToBase_init(RobotInfo_t* robotInfo);

#endif // RETURNTOBASE_H