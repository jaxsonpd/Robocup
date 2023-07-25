/** 
 * @file utils.h
 * @brief This module contains general utility functions that are used throughout the project.
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-13
 */


#ifndef UTILS_H
#define UTILS_H


// ===================================== Includes =====================================
#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>


// ===================================== Types/Constants ==============================


// ===================================== Function Prototypes ==========================
void waitForGo();
void serialInit(uint32_t baudRate);
bool checkStopped();

#endif // UTILS_H