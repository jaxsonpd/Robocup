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
#include <Servo.h>


bool crane_setup();
void crane_move_weight();


#endif // COLLECTOR_H
