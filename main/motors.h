/** 
 * @file motors.h
 * @brief Motor control functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-06-03
 */


#ifndef MOTOR_H
#define MOTOR_H


// ===================================== Includes =====================================
#include <stdint.h>

// ===================================== Constants ====================================
enum motor {
    MOTOR_1,
    MOTOR_2
};

// ===================================== Function Prototypes ==========================
bool motors_setup();
bool motors_setSpeed(uint8_t motor, int16_t speed);

#endif // MOTOR_H
