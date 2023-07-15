/** 
 * @file main.ino
 * @brief Main file for the robot cup project
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-06-03
 */


// ===================================== Includes =====================================
#include <Arduino.h>
#include <Servo.h>

#include "motors.h"
#include "sensors.h"
#include "utils.h"

#include "IRConfig.h"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
#define SERIAL_BAUD_RATE 57600

// IR triangulating sensor structs
irTri_sensor_t irTri_0 = {IRTRI_0_PIN, IRTRI_0_TYPE};


// ===================================== Globals ======================================
bool running = true;
extern uint8_t numUS;

void robot_setup() {
	// Initialise the serial output
	serialInit(SERIAL_BAUD_RATE);

	// Initialise the main drive motors
	motors_setup();

    sensors_init();

    delay(1000);

	// Wait for the go button to be pressed
	waitForGo();
	Serial.println("Go button pressed, starting robot");
  
	delay(1000);
	running = true;
}

void setup() {} // Keep the Arduino IDE happy

void loop() {
	robot_setup();

	while(running) {
        // Update robot information
        uint16_t IRDistance = sensors_getIRTriDistance(irTri_0);
        Serial.print(IRDistance);
        Serial.print(",");
        Serial.print(analogRead(irTri_0.pin));
        Serial.print(",");
        delay(100);

        uint16_t USDistances[numUS] = {0};

        sensors_pingUS();
        delay(30);
        sensors_getUSDistances(USDistances);

        for (uint8_t i = 0; i < numUS; i++) {
            Serial.print(USDistances[i]);
            Serial.print(",");
        }

        Serial.println();

        // perform actions

        // Check if the robot should keep running
        running = checkStopped();
	}
}


