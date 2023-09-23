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
#include "collector.hpp"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ===================================== Constants ====================================
#define SERIAL_BAUD_RATE 115200

// Scheduler constants
#define SENSOR_UPDATE_TIME 20
#define ROBOT_INFO_UPDATE_TIME 50
#define FSM_UPDATE_TIME 2000
#define SLOW_UPDATE_TIME 500


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
    Serial.println("Initialising motors");
    if (motors_setup()) {
        Serial.println("Error setting up motors");

    }

    Serial.println("Initialising sensors");
    if (sensors_init()) {
        Serial.println("Error setting up sensors");
    }
    if (crane_setup()) {
      Serial.println("Error setting up crane");
    }

    Serial.println("Initialising weight collection");
    returnToBase_init(&robotInfo);

    // Wait for the go button to be pressed
    waitForGo();
    Serial.println("Go button pressed, starting robot");
    
    delay(1000);
    running = true;
}


/**
 * @Brief Monitor the robots movements and get it unstuck if it gets stuck
 * @param robotInfo Pointer to the robotInfo struct 
 *
 * @return true if the robot is stuck
 */
 void watchDog(robotInfo* robotInfo) {
    // Check if the robot is trying to go forward but is not moving

    // Check if the robot is trying to rotate but is not moving
 }




enum FSMStates {
    FIND_WEIGHTS,
    RETURN_HOME,
    WATCH_DOG
};

/**
 * @Breif Update the finite state machine that controls the robots behavour
 * @param robotInfo Pointer to the robotInfo struct
 */
void FSM(robotInfo* robotInfo) {
    static uint8_t state = 0;
    static firstRun = true;
    static prevousStateWatchDog = FIND_WEIGHTS; // The state the robot was in before the watchdog was triggered

    switch (state) {
        case FIND_WEIGHTS:
            if (firstRun) {
                firstRun = false;
            }

            findWeights(robotInfo);
            break;
        case RETURN_HOME:
            if (firstRun) {
                firstRun = false;
            }

            returnToBase(robotInfo);
            break;
        case WATCH_DOG:
            if (firstRun) {
                firstRun = false;
            }

            // Do nothing and let the watch dog do its thing
            if (!watchDog(robotInfo)) {
                state = prevousStateWatchDog;
            }
            break;
    }

    // Check the robots watch dog
    if (state != WATCH_DOG) {
        if (watchDog(robotInfo)) {
            prevousStateWatchDog = state;
            state = WATCH_DOG;
            firstRun = true;
        }
    }
}




void setup() {} // Keep the Arduino IDE happy

void loop() {
	robot_setup();

    elapsedMillis sensorUpdateTimer = 0;
    elapsedMillis robotInfoUpdateTimer = 0;
    elapsedMillis FSMTimer = 0;
    elapsedMillis slowUpdateTimer = 0;

	while(running) {
        if (sensorUpdateTimer > SENSOR_UPDATE_TIME) {
            sensors_update();
            sensorUpdateTimer = 0;
        }

        // Update robot information
        if (robotInfoUpdateTimer > ROBOT_INFO_UPDATE_TIME) {
            sensors_updateInfo(&robotInfo);
            motors_update(&robotInfo);
            // printRobotInfo(&robotInfo);
            robotInfoUpdateTimer = 0;
        }

        
        // perform actions
        if (FSMTimer > FSM_UPDATE_TIME) {
            // findWeights(&robotInfo);
            // motors_formShape(&robotInfo, 5000, 90);
            // motors_followHeading(&robotInfo, 0, 35);
            // crane_move_weight();
            // returnToBase(&robotInfo);
            motors_setLeft(40);
            motors_setRight(40);
            
            FSMTimer = 0;
        }

        if (slowUpdateTimer > SLOW_UPDATE_TIME) {
            // Check if the robot should keep running
            running = checkStopped();
            slowUpdateTimer = 0;
        }
	  }

    sensor_deInit();
    weightCollection_deInit(&robotInfo);
    motors_deInit(&robotInfo);
}
