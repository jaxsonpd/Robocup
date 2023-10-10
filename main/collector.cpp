/** 
 * @file collector.cpp
 * @brief crane functionality
 * @author Campbell Morris
 * @date 2023-08-13
 */

#include <Arduino.h>

#include "collector.hpp"
#include "robotInformation.hpp"
#include "motors.hpp"

#include <Servo.h>

// ===================================== Defines/Constants =====================================
// Crane constants
#define CRANE_SERVO_PIN 30
#define ELECTROMAGNET_PIN 14

#define RELEASE_ANGLE 165
#define COLLECT_ANGLE 0
#define STANDBY_ANGLE 105

// Claw constants
#define LEFT_CLAW_SERVO_PIN 31
#define RIGHT_CLAW_SERVO_PIN 32

#define LEFT_OPEN_ANGLE 75
#define LEFT_CLOSED_ANGLE 170
#define RIGHT_OPEN_ANGLE 113
#define RIGHT_CLOSED_ANGLE 10

#define CLAW_TEST_PIN 24

// FSM constants
#define CHECK_WEIGHT_TIME 1000

#define REVERSE_TIME 700 // Time to reverse when positioning the weight
#define REVERSE_SPEED -45 // Speed to reverse at

#define FORWARD_TIME 700 // Time to move forward when positioning the weight
#define FORWARD_SPEED 45 // Speed to move forward at

#define COLLECT_TIME 2000 // Time to collect the weight before dropping it
#define WEIGHT_IN_CLAW_DISTANCE 100 // Distance to weight to trigger weight collection (wether or not the weight has been caught)

#define BACK_OFF_TIME 500 // Time to back off when a weight is not detected
#define BACK_OFF_ROTATE_TIME 500 // Time to rotate when a weight is not detected

enum CranePositions {
  COLLECT = 0,
  RELEASE,
  STANDBY
};

enum FSMStates {
    CHECK_WEIGHT = 0,
    WAIT,
    BACK_OFF_WEIGHT,
    POSITION_WEIGHT,
    COLLECT_WEIGHT
};
// ===================================== Globals ======================================

Servo craneServo;
Servo leftClawServo;
Servo rightClawServo;

// ===================================== Function Definitions =========================
/**
 * @breif postion the crane to a position
 * @param position the position to move to
 *
*/
void crane_move(CranePositions position) {
  switch(position) {
    case COLLECT:
      craneServo.write(COLLECT_ANGLE);
      break;
    case RELEASE:
      craneServo.write(RELEASE_ANGLE);
      break;
    case STANDBY:
      craneServo.write(STANDBY_ANGLE);
      break;
  }
}

/**
 * @brief turn on the electromagnet
 * @param on whether to turn on or off the electromagnet
*/
void electromagnet(bool on) {
  if (on) {
    digitalWrite(ELECTROMAGNET_PIN, HIGH);
  } else {
    digitalWrite(ELECTROMAGNET_PIN, LOW);
  }
}




/**
 * @brief setup the claw servos
 * 
 * @return false if setup was successful
*/
bool claw_setup() {
    leftClawServo.attach(LEFT_CLAW_SERVO_PIN);
    rightClawServo.attach(RIGHT_CLAW_SERVO_PIN);
    // pinMode(CLAW_TEST_PIN, INPUT);

    return 0;
}

/*
 * @brief close the claw
 *
*/
void close_claws() { 
    leftClawServo.write(LEFT_CLOSED_ANGLE);
    rightClawServo.write(RIGHT_CLOSED_ANGLE);
} 


/*
 * @brief open the claw
 *
*/
void open_claws() { 
    leftClawServo.write(LEFT_OPEN_ANGLE);
    rightClawServo.write(RIGHT_OPEN_ANGLE);
}

bool crane_setup() {   
    craneServo.attach(CRANE_SERVO_PIN); 
    pinMode(ELECTROMAGNET_PIN, OUTPUT);

    electromagnet(false);
    crane_move(STANDBY);

    return 0;
}


/**
 * @brief test to see if the weight is in the claw
 * 
 * @return true if the weight is conductive (pin reads 0)
*/
bool claw_test() {
    return !digitalRead(CLAW_TEST_PIN);
}


// +++++++++++++++++++++++++++++++++++++++++ FSM +++++++++++++++++++++++++++++++++++++++++
/** 
 * @brief Initalise the collector components
 * 
 * @return false if initalisation was successful
 */
bool collector_setup() {
    if (crane_setup()) {
        Serial.println("Error setting up crane");
        return 1;
    }

    if (claw_setup()) {
        Serial.println("Error setting up claw");
        return 1;
    }

    return 0;
}

static uint8_t collectionState = CHECK_WEIGHT;

/**
 * @brief Collect the weights using a FSM
 * @param robotInfo the robot information struct
 * 
 * @return 1 if the robot is finished and weight collected and 2 if weight wasn't collected
*/
uint8_t weightCollect(RobotInfo_t* robotInfo) {
    static elapsedMillis stateTimer = 0;
    static bool firstRun = true;

    // State machine
    switch (collectionState) {
        case CHECK_WEIGHT:
            if (firstRun) {
                firstRun = false;
                stateTimer = 0;
                close_claws();
            }

            if (stateTimer > CHECK_WEIGHT_TIME) { // Check the weight to see if it is there and conductive
                if ((robotInfo->IRBottom_Distance < WEIGHT_IN_CLAW_DISTANCE)) {
                    if ((analogRead(A10) < 50) && (robotInfo->colorOver == 2)) {
                        collectionState = POSITION_WEIGHT;
                        firstRun = true;
                    } else {
                        open_claws();
                        firstRun = true;
                        collectionState = BACK_OFF_WEIGHT;
                    }
                    
                } else {
                    open_claws();
                    firstRun = true;
                    collectionState = WAIT;
                }
            }

            break;

        case WAIT:
            if (firstRun) {
                firstRun = false;
                stateTimer = 0;
            }

            if (stateTimer > 500) {
                collectionState = CHECK_WEIGHT;
                firstRun = true;
                return 1;
            }

            break;

        case BACK_OFF_WEIGHT: // Move away from a weight that is not real
            if (firstRun) {
                firstRun = false;
                stateTimer = 0;
            }

            if (stateTimer < BACK_OFF_TIME) {
                motors_setLeft(-40);
                motors_setRight(-40);
            } else if (stateTimer < (BACK_OFF_TIME + BACK_OFF_ROTATE_TIME)) {
                motors_setLeft(40);
                motors_setRight(-40);
            } else {
                collectionState = CHECK_WEIGHT;
                firstRun = true;
                return 2;
            }


        case POSITION_WEIGHT:
            if (firstRun) {
                firstRun = false;
                stateTimer = 0;
                electromagnet(true);
            }

            if (stateTimer < REVERSE_TIME) {
                motors_setLeft(REVERSE_SPEED);
                motors_setRight(REVERSE_SPEED);
            } else if (stateTimer < (REVERSE_TIME + FORWARD_TIME)) {
                motors_setLeft(FORWARD_SPEED);
                motors_setRight(FORWARD_SPEED);
            } else if (stateTimer < (REVERSE_TIME + FORWARD_TIME + 1000)) {
                crane_move(COLLECT);

                motors_setLeft(0);
                motors_setRight(0);
                
            } else {
                collectionState = COLLECT_WEIGHT;
                firstRun = 1;
            }

            break;
        
        case COLLECT_WEIGHT:
            if (firstRun) {
                firstRun = false;
                stateTimer = 0;
                crane_move(RELEASE);
            }

            if (stateTimer > COLLECT_TIME) {
                electromagnet(false);
                open_claws();

                collectionState = CHECK_WEIGHT;
                firstRun = 1;

                crane_move(STANDBY);

                if (robotInfo->IRBottom_Distance > WEIGHT_IN_CLAW_DISTANCE) {
                    robotInfo->weightsOnBoard++;
                }

                return true;
            }

            break;
    }

    robotInfo->mode = 20 + collectionState;
    return 0;
}

void collector_deInit() {
    collectionState = CHECK_WEIGHT;
    crane_move(STANDBY);
    electromagnet(false);
    open_claws();
}

