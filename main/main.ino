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
    elapsedMillis sensorUpdateTimer = 0;
    elapsedMillis robotInfoUpdateTimer = 0;
    elapsedMillis PIDTimer = 0;
    elapsedMIllis slowUpdateTimer = 0;
    bool offSetpoint = 0;

	robot_setup();

	while(running) {
        if (sensorUpdateTimer > 5) {
            sensors_update();
            sensorUpdateTimer = 0;
        }

        // Update robot information
        if (robotInfoUpdateTimer > 100) {
            motors_updateInfo(&robotInfo);
            sensors_updateInfo(&robotInfo);
            robotInfoUpdateTimer = 0;
        }
        
        // perform actions
        if (PIDTimer > 100) {
            offSetpoint = motors_formShape(&robotInfo, 5000, 90);

            PIDTimer = 0;
        }

        if (slowUpdateTimer == 1000) {
            if (offSetpoint) {
                setLED(LED_RED, 1);
            } else {
                setLED(LED_GREEN, 1);
            }
        }





        // Check if the robot should keep running
        running = checkStopped();

	}
}


