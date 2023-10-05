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
#include "src/colorSensor.hpp"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ===================================== Constants ====================================
#define SERIAL_BAUD_RATE 115200

// Scheduler constants
#define SENSOR_UPDATE_TIME 20
#define ROBOT_INFO_UPDATE_TIME 50
#define FSM_UPDATE_TIME 100
#define SLOW_UPDATE_TIME 500

// Watchdog constants
#define WATCH_DOG_ENABLED false // Whether the watchdog is enabled or not

#define FORWARD_SPEED_THRESHOLD 10 // Threshold for when the robot is considered to be moving forward
#define ROTATION_SPEED_THRESHOLD 10 // Threshold for difference in speeds when the robot is considered to be rotating
#define REVERSE_SPEED_THRESHOLD -10 // Threshold for when the robot is considered to be moving backwards

#define FOWARD_ACC_THRESHOLD 0.5 // Threshold for when the robot is considered to be accelerating forward
#define ROTATION_ACC_THRESHOLD 0.55 // Threshold for when the robot is considered to be accelerating rotation

#define STATIONARY_TIME 2000 // Time the robot must be stationary for before the watchdog is triggered

// FSM constants
enum FSMStates {
    FIND_WEIGHTS,
    RETURN_HOME,
    WATCH_DOG
};

#define REVERSE_TIME 1000 // Time to reverse for when the watchdog is triggered
#define REVERSE_SPEED -50 // Speed to reverse at when the watchdog is triggered


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
 * @param reset Whether to reset the watchdog or not
 *
 * @return > 0 if the robot is stuck (1 = Forward Blocked, 2 = Rotation Blocked, 3 = Reveresed Blocked, 4 = B)
 */
uint8_t watchDog(RobotInfo_t* robotInfo, bool reset) {
    static elapsedMillis watchDogTimer = 0;

    if (reset) {
        watchDogTimer = 0;
    }

    // Check if the robot is trying to go forward but is not moving
    if (robotInfo->leftMotorSpeed + robotInfo->rightMotorSpeed > FORWARD_SPEED_THRESHOLD) {
        if (abs(robotInfo->forwardAcceleration) < FOWARD_ACC_THRESHOLD) {
            if (watchDogTimer > STATIONARY_TIME) {
                watchDogTimer = 0;
                return 1;
            } else {
                return 0;
            }
        }
    }

    // Check if the robot is trying to rotate but is not moving
    if (abs(robotInfo->leftMotorSpeed - robotInfo->rightMotorSpeed) > ROTATION_SPEED_THRESHOLD) {
        if (abs(robotInfo->rotationAcceleration) < ROTATION_ACC_THRESHOLD) {
            if (watchDogTimer > STATIONARY_TIME) {
                watchDogTimer = 0;
                return 2;
            } else {
                return 0;
            }
        }
    }

    // Check if the robot is trying to reverse but is not moving
    if (robotInfo->leftMotorSpeed + robotInfo->rightMotorSpeed < REVERSE_SPEED_THRESHOLD) {
        if (abs(robotInfo->forwardAcceleration) < FOWARD_ACC_THRESHOLD) {
            if (watchDogTimer > STATIONARY_TIME) {
                watchDogTimer = 0;
                return 3;
            } else {
                return 0;
            }
        }
    }

    watchDogTimer = 0;
    return 0;
}

/**
 * @Breif Update the finite state machine that controls the robots behavour
 * @param robotInfo Pointer to the robotInfo struct
 */
void FSM(RobotInfo_t* robotInfo) {
    static uint8_t state = FIND_WEIGHTS;
    static bool firstRun = true;
    static uint8_t prevousStateWatchDog = FIND_WEIGHTS; // The state the robot was in before the watchdog was triggered
    static elapsedMillis stateTimer = 0;

    switch (state) {
        case FIND_WEIGHTS:
            if (firstRun) {
                firstRun = false;
            }

            if (robotInfo->weightsOnBoard >= 3) {
                state = RETURN_HOME;
                firstRun = true;
            } else {
                findWeights(robotInfo);
            }
            break;

        case RETURN_HOME:
            if (firstRun) {
                firstRun = false;
            }

            if (/*returnToBase(robotInfo)*/ false) {
                state = FIND_WEIGHTS;
                firstRun = true;
            }
            break;

        case WATCH_DOG:
            if (firstRun) {
                firstRun = false;
                stateTimer = 0;
            }

            if (stateTimer > REVERSE_TIME) {
                state = prevousStateWatchDog;
                firstRun = true;

                motors_setLeft(0);
                motors_setRight(0);
                watchDog(robotInfo, true);
            } else {
                motors_setLeft(REVERSE_SPEED);
                motors_setRight(REVERSE_SPEED);
            }
            break;
    }

    // Check the robots watch dog
    if (state != WATCH_DOG & WATCH_DOG_ENABLED) {
        if (watchDog(robotInfo, false)) {
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
            printRobotInfo(&robotInfo);
            robotInfoUpdateTimer = 0;
        }

        
        // perform actions
        if (FSMTimer > FSM_UPDATE_TIME) {
            // findWeights(&robotInfo);
            // motors_formShape(&robotInfo, 5000, 90);
            // motors_followHeading(&robotInfo, 0, 35);
            // crane_move_weight();
            // returnToBase(&robotInfo);
            FSM(&robotInfo);
            // colorSensor_setBase();
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
