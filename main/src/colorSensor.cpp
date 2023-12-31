/** 
 * @file colorSensor.cpp
 * @brief Color Sensor Driver
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-09-09
 */


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include <Adafruit_TCS34725.h>
#include "colorSensor.hpp"
#include <Arduino.h>
#include <Wire.h>

// ===================================== Types/Constants ==============================
#define R_THRESHOLD 100
#define G_THRESHOLD 100
#define B_THRESHOLD 100
#define READ_TIME 50

// Green base color
#define R_GREEN_BASE 0
#define G_GREEN_BASE 130
#define B_GREEN_BASE 100

// Blue base color
#define R_BLUE_BASE 0
#define G_BLUE_BASE 100
#define B_BLUE_BASE 100

#define TCS34725_ADDRESS (0x29)     /**< I2C address **/

// ===================================== Globals ======================================
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);



// ===================================== Function Definitions =========================
/**
 * @brief Set the base color
 * 
 * @return what base the robot is over
 */
uint8_t colorSensor_getBase() {
    uint16_t r, g, b, c;
    tcs.setInterrupt(false);      // turn on LED
    delay(60);  // takes 50ms to read
    tcs.getRawData(&r, &g, &b, &c);
    tcs.setInterrupt(true);  // turn off LED

    // Check what base the robot is over
    if ((G_GREEN_BASE < g) && (b < B_GREEN_BASE)) {
        return GREEN;
    } else if ((B_BLUE_BASE < b)) {
        return BLUE;
    } else {
        return ARENA;
    }
}


/**
 * @brief Initialize the color sensor
 * 
 * @return true if successful
 * @return false if unsuccessful
 */
bool colorSensor_init() {
    // I2C Init
    Wire1.begin();
    Wire1.setClock(400000); // use 400 kHz I2C

    if (tcs.begin(TCS34725_ADDRESS, &Wire1)) {
        return false;
    } else {
        return true;
    }
}

/** 
 * @brief Work out if robot is over the base
 * 
 * @return what color the robot is over 
 */
uint8_t colorSensor_read() {
    uint16_t r, g, b, c;
    static uint8_t prevousColor = ARENA;
    static elapsedMillis readingTimer = 0;
    static bool ledOn = false;

    if (!ledOn) { // Start a read
        tcs.setInterrupt(false);      // turn on LED
        readingTimer = 0;
        ledOn = true;
    } else if (readingTimer > READ_TIME) { // Wait for 50ms
        tcs.getRawData(&r, &g, &b, &c);
        tcs.setInterrupt(true);  // turn off LED

        ledOn = false;
        
        // Check what base the robot is over
        if ((G_GREEN_BASE < g) && (b < B_GREEN_BASE)) {
            prevousColor = GREEN;
            return GREEN;
        } else if ((B_BLUE_BASE < b)) {
            prevousColor = BLUE;
            return BLUE;
        } else {
            prevousColor = ARENA;
            return ARENA;
        }
    }

    return prevousColor;
}
