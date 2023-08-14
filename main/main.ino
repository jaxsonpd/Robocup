/** 
 * @file main.ino
 * @brief Main file for the robot cup project
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-06-03
 */


// ===================================== Includes =====================================
#include <Arduino.h>

#include "motors.hpp"
#include "sensors.hpp"
#include "utils.hpp"
#include "robotInformation.hpp"
#include "returnToBase.hpp"
#include "weightCollection.hpp"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ===================================== Constants ====================================
#define SERIAL_BAUD_RATE 115200


// ===================================== Globals ======================================
bool running = true; // Whether the robot is running or not

// robotInfo struct 
RobotInfo_t robotInfo = {0};


// ===================================== Function Definitions =========================

void robot_setup() {
	// Initialise the serial output
	serialInit(SERIAL_BAUD_RATE);
    Serial1.begin(SERIAL_BAUD_RATE);

	// Initialise the main drive motors
    if (motors_setup()) {
        Serial.println("Error setting up motors");

    }

    if (sensors_init()) {
        Serial.println("Error setting up sensors");
    }

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

    elapsedMillis sensorUpdateTimer = 0;
    elapsedMillis robotInfoUpdateTimer = 0;
    elapsedMillis PIDTimer = 0;
    elapsedMillis slowUpdateTimer = 0;

	  while(running) {
        if (sensorUpdateTimer > 5) {
            sensors_update();
            sensorUpdateTimer = 0;
        }
        
        // Update robot information
        if (robotInfoUpdateTimer > 100) {
            sensors_updateInfo(&robotInfo);
            printRobotInfo(&robotInfo);
            robotInfoUpdateTimer = 0;
        }

        
        // perform actions
        if (PIDTimer > 50) {
            findWeights(&robotInfo);
            // motors_formShape(&robotInfo, 5000, 90);
            // motors_followHeading(&robotInfo, 0, 35);
            PIDTimer = 0;
        }

        if (slowUpdateTimer > 100) {
            char buffer[150];
            sprintf(buffer, "IR T: %4d, B: %4d, M: %1d, IMU: %3d, %3d \n", robotInfo.IRTop_Distance, robotInfo.IRBottom_Distance, robotInfo.mode, robotInfo.IMU_Heading, robotInfo.targetHeading);

            Serial1.write(buffer);
            slowUpdateTimer = 0;
        }

        // Check if the robot should keep running
        running = checkStopped();
	  }

    sensor_deInit();
    weightCollection_deInit(&robotInfo);
    motors_deinit(&robotInfo);
}


