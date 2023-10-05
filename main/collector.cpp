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
#define CRANE_SERVO_PIN 29
#define ELECTROMAGNET_PIN 20

#define RELEASE_ANGLE 140
#define COLLECT_ANGLE 15
#define STANDBY_ANGLE 100

// Claw constants
#define LEFT_CLAW_SERVO_PIN 30
#define RIGHT_CLAW_SERVO_PIN 31

#define LEFT_OPEN_ANGLE 50
#define LEFT_CLOSED_ANGLE 3
#define RIGHT_OPEN_ANGLE 135
#define RIGHT_CLOSED_ANGLE 180

#define CLAW_TEST_PIN 24

// FSM constants
#define REVERSE_TIME 1000 // Time to reverse when positioning the weight
#define REVERSE_SPEED 20 // Speed to reverse at

#define FORWARD_TIME 1000 // Time to move forward when positioning the weight
#define FORWARD_SPEED 20 // Speed to move forward at

#define COLLECT_TIME 3000 // Time to collect the weight before dropping it
#define WEIGHT_IN_CLAW_DISTANCE 100 // Distance to weight to trigger weight collection (wether or not the weight has been caught)



enum CranePositions {
  COLLECT = 0,
  RELEASE,
  STANDBY
};

enum FSMStates {
    CHECK_WEIGHT = 0,
    POSITION_WEIGHT,
    COLLECT_WEIGHT
}
// ===================================== Globals ======================================

Servo craneServo, leftClawServo, rightClawServo;

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

bool crane_setup() {   
    craneServo.attach(CRANE_SERVO_PIN); 
    pinMode(ELECTROMAGNET_PIN, OUTPUT);

    electromagnet(false);
    crane_move(STANDBY);

    return 0;
}


/**
 * @brief setup the claw servos
 * 
 * @return false if setup was successful
*/
bool claw_setup() {
    leftClawServo.attach(LEFT_CLAW_SERVO_PIN);
    rightClawServo.attach(RIGHT_CLAW_SERVO_PIN);
    pinMode(CLAW_TEST_PIN, INPUT);

    return 0;
}

/*
 * @brief close the claw
 *
*/
void close_claws() { 
    leftClaw.write(LEFT_CLOSED_ANGLE);
    rightClaw.write(RIGHT_CLOSED_ANGLE);
} 


/*
 * @brief open the claw
 *
*/
void open_claws() { 
    leftClaw.write(LEFT_OPEN_ANGLE);
    rightClaw.write(RIGHT_OPEN_ANGLE);
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


/**
 * @brief Collect the weights using a FSM
 * @param robotInfo the robot information struct
 * 
 * @return true if the robot is finished (Regardless of wether weight was collected or not)
*/
bool weightCollect(RobotInfo_t* robotInfo) {
    static uint8_t collectionState = CHECK_WEIGHT;
    static ellapsedMillis stateTimer = 0;
    static bool firstRun = true;

    // State machine
    switch (collectionState) {
        case CHECK_WEIGHT:
            if (firstRun) {
                firstRun = false;
                stateTimer = 0;
                close_claws();
            }

            if (stateTimer > 1000) { // Check the weight to see if it is there and conductive
                if (robotInfo->IRBottom_Distance < WEIGHT_IN_CLAW_DISTANCE) {
                    if (claw_test()) {
                        collectionState = POSITION_WEIGHT;
                        firstRun = true;
                    } else {
                        open_claws();
                        return true;
                    }
                    
                } else {
                    open_claws();
                    return true;
                }
            }

            break;

        case POSITION_WEIGHT:
            if (firstRun) {
                firstRun = false;
                stateTimer = 0;
                crane_move(COLLECT);
                electromagnet(true);
            }

            if (stateTimer < REVERSE_TIME) {
                motors_setLeft(REVERSE_SPEED);
                motors_setRight(REVERSE_SPEED);
            } else if (stateTimer < (REVERSE_TIME + FORWARD_TIME)) {
                motors_setLeft(FORWARD_SPEED);
                motors_setRight(FORWARD_SPEED);
            } else {
                motors_setLeft(0);
                motors_setRight(0);
                collectionState = COLLECT_WEIGHT;
                firstRun = true;
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
                firstRun = true;

                crane_move(STANDBY);

                if (robotInfo->IRBottom_Distance > WEIGHT_IN_CLAW_DISTANCE) {
                    robotInfo->weightsOnBoard++;
                }

                return true;
            }

            break;
    }
}



