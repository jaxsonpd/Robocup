/** 
 * @file main.ino
 * @brief Main file for the robot cup project
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-06-03
 */


// ===================================== Includes =====================================
#include <Arduino.h>
#include <Servo.h>

#include "motors.hpp"
#include "sensors.hpp"
#include "utils.hpp"
#include "robotInformation.hpp"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ===================================== Constants ====================================
#define SERIAL_BAUD_RATE 57600


// ===================================== Globals ======================================
bool running = true; // Whether the robot is running or not
extern uint8_t numUS; // Number of ultrasonic sensors from sensors.cpp

// robotInfo struct 
RobotInfo_t robotInfo = {0};


// ===================================== Function Definitions =========================

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
        sensors_update();

        // Update robot information
        motors_updateInfo(&robotInfo);
        sensors_updateInfo(&robotInfo);
        
        // perform actions
        printRobotInfo(&robotInfo);

        // Check if the robot should keep running
        running = checkStopped();

        delay(10);
	}
}


