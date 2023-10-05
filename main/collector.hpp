/** 
 * @file collector.hpp
 * @brief crane functionality
 * @author Campbell Morris
 * @date 2023-08-13
 */


#ifndef COLLECTOR_H    
#define COLLECTOR_H

#include <Arduino.h>
#include "collector.hpp"

bool collector_setup();
bool weightCollect(RobotInfo_t* robotInfo);



#endif // COLLECTOR_H
