/** 
 * @file utils.cpp
 * @brief This module contains general utility functions that are used throughout the project.
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-13
 */


// ===================================== Includes =====================================
#include <Arduino.h>

#include "utils.hpp"
#include <Wire.h>

#include <stdint.h>
#include <stdbool.h>

// ===================================== Types/Constants ==============================
#define GO_BUTTON_PIN 26
#define GREEN_LED_PIN 27
#define RED_LED_PIN 28
#define BLUE_LED_PIN 29

// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
/**
 * @brief Wait for the Go button to be pressed
 * 
 */
void waitForGo() {
    pinMode(GO_BUTTON_PIN, INPUT);
	
	Serial.println("Waiting for go button to be pressed");
	while (digitalRead(GO_BUTTON_PIN) == LOW) {
		delay(10);
	}
}


/** 
 * @brief Initialise the serial monitor
 * @param baudRate The baud rate to use for the serial monitor
 *
 */
void serialInit(uint32_t baudRate) {
    // I2C Init
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    Serial.begin(baudRate);

    // Let the user know that the serial output has started
	char startString[50];
	sprintf(startString, "Serial Output started at %ld baud", baudRate);
	Serial.println(startString);
}


/**
 * @brief check for the stopped conditions
 * 
 * @return true The robot should keep running
 */
bool checkStopped() {
    // Check if the user wants to stop the robot
    if (Serial.available() > 0) {
        char input = Serial.read();
        if (input == 'q') {
            Serial.println("Stopping robot (Serial)");
            return false;
        }
    } else if (digitalRead(GO_BUTTON_PIN) == HIGH) {
        Serial.println("Stopping robot (Button)");
        return false;
    }

    return true;
}


/** 
 * @brief Set the LED colours
 * @param ledColor The colout to set the LED to
 * @param brightness The brightness of the LED (0 or 1)
 * 
 * 
 */
void setLED(uint8_t ledColor, uint8_t brightness) {
    if (ledColor == LED_RED) {
        digitalWrite(RED_LED_PIN, brightness);
    } else if (ledColor == LED_GREEN) {
        digitalWrite(GREEN_LED_PIN, brightness);
    } else if (ledColor == LED_BLUE) {
        digitalWrite(BLUE_LED_PIN, brightness);
    }
}