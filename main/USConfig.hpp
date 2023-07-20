/** 
 * @file USConfig.h
 * @brief Configuration file for the ultrasonic sensors.
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-14
 */


#ifndef USCONFIG_H
#define USCONFIG_H


// ===================================== Includes =====================================


// ===================================== Types/Constants ==============================
// ** Ultrasonic sensor parameters **
// Number of ultrasonic sensors
#define US_NUM 2

// Parameters for ultrasonic sensor 0
#define US_0 // Left ultrasonic sensor
#define US_TRIG_0 3
#define US_ECHO_0 2

// Parameters for ultrasonic sensor 1
#define US_1 // Right ultrasonic sensor
#define US_TRIG_1 5
#define US_ECHO_1 4

// ===================================== Function Prototypes ==========================


#endif // USCONFIG_H