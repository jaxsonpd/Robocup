/** 
 * @file main.ino
 * @brief Main file for the robot cup project
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-06-03
 */


// ===================================== Includes =====================================
#include "Arduino.h"

#include "motors.h"
#include "sensors.h"

#include <stdio.h>
#include <Servo.h>


// ===================================== Constants ====================================
#define GO_BUTTON_PIN 23

#define SERIAL_BAUD_RATE 57600


// ===================================== Globals ======================================
bool running = true;


/**
 * @brief Wait for the Go button to be pressed
 * 
 */
void waitForGo() {
    pinMode(GO_BUTTON_PIN, INPUT);
	
	Serial.println("Waiting for go button to be pressed");
	while (digitalRead(GO_BUTTON_PIN) == LOW) {
		delay(100);
	}
}

/**
 * @brief Initialise the serial output
 * 
 */
void serialInit() {
	Serial.begin(SERIAL_BAUD_RATE);

	// Let the user know that the serial output has started
	char startString[50];
	sprintf(startString, "Serial Output started at %d baud", SERIAL_BAUD_RATE);
	Serial.println(startString);
}

void robot_setup() {
	// Initialise the serial output
	serialInit();

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

void setup() {}

void loop() {
	robot_setup();

	while(running) {
        // Ramp the motor 
        Serial.printf("%4d \n", read_infrared());
        delay(100);

        // Check if the user wants to stop the robot
		if (Serial.available() > 0) {
			char input = Serial.read();
			if (input == 'q') {
				running = false;
				Serial.println("Stopping robot (Serial)");
			}
		} else if (digitalRead(GO_BUTTON_PIN) == HIGH) {
            running = false;
            Serial.println("Stopping robot (Button)");
        }
	}

}


