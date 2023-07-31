/** 
 * @file weightCollection.h
 * @brief Weight collection functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-14
 */


#ifndef WEIGHTCOLLECTION_H
#define WEIGHTCOLLECTION_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include"robotInformation.hpp"


// ===================================== Types/Constants ==============================


// ===================================== Function Prototypes ==========================
void findWeights(RobotInfo_t *robotInfo);

#endif // WEIGHTCOLLECTION_H