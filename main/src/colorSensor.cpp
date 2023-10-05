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
#include <Arduino.h>
#include <Wire.h>

// ===================================== Types/Constants ==============================
#define R_THRESHOLD 100
#define G_THRESHOLD 100
#define B_THRESHOLD 100
#define READ_TIME 50

#define TCS34725_ADDRESS (0x29)     /**< I2C address **/

// ===================================== Globals ======================================
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

uint16_t r_base = 0;
uint16_t g_base = 0;
uint16_t b_base = 0;

// ===================================== Function Definitions =========================
/**
 * @brief Set the base color
 * 
 */
void colorSensor_setBase() {
    uint16_t c;
    tcs.setInterrupt(false);      // turn on LED
    delay(60);  // takes 50ms to read
    tcs.getRawData(&r_base, &g_base, &b_base, &c);
    tcs.setInterrupt(true);  // turn off LED
    Serial.println("Base Color " + String(r_base) + " " + String(g_base) + " " + String(b_base) + " " + String(c));
}


/**
 * @brief Initialize the color sensor
 * 
 * @return true if successful
 * @return false if unsuccessful
 */
bool colorSensor_init() {
    // I2C Init
    Wire1.begin(TCS34725_ADDRESS, &Wire1);
    Wire1.setClock(400000); // use 400 kHz I2C

    if (tcs.begin()) {
        return true;
    } else {
        return false;
    }

    colorSensor_setBase();
}

/** 
 * @brief Work out if robot is over the base
 * 
 * @return true if over base
 */
bool colorSensor_overBase() {
    uint16_t r, g, b, c;
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
        
        if (abs(r - r_base) < R_THRESHOLD && abs(g - g_base) < G_THRESHOLD && abs(b - b_base) < B_THRESHOLD) {
            return true;
        } 
    }

    return false;
}
