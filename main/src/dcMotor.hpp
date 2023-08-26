/** 
 * @file dcMotor.hpp
 * @brief Header file for the DC motor class implementation
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-08-20
 */


#ifndef DCMOTOR_H
#define DCMOTOR_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include <Arduino.h>
#include <Servo.h>


// ===================================== Types/Constants ==============================
#define MOTOR_SPEED_MAX 100
#define MOTOR_SPEED_MIN -100

// ===================================== Function Prototypes ==========================
class dcMotor {
    public:
        bool init(uint8_t pin, bool direction);
        bool setSpeed(int16_t speed);
        int16_t getSpeed(void);
        bool deInit(void);

    private:
        uint8_t motorPin;
        bool motorDirection;
        int16_t motorSpeed;
        Servo motor;
};


#endif // DCMOTOR_H