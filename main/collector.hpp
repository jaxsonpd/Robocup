/** 
 * @file collector.hpp
 * @brief crane functionality
 * @author Campbell Morris
 * @date 2023-08-13
 */


#ifndef COLLECTOR_H    
#define COLLECTOR_H

#include <Arduino.h>
#include "robotInformation.hpp"

bool collector_setup();
uint8_t weightCollect(RobotInfo_t* robotInfo);
void collector_deInit();



#endif // COLLECTOR_H
