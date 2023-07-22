/** 
 * @file sensors.cpp
 * @brief Sensor control functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-10
 */


// ===================================== Includes =====================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "sensors.hpp"
#include "ultrasonic.hpp"
#include "robotInformation.hpp"
#include "circBuffer.hpp"

#include "USConfig.hpp"
#include "IRConfig.hpp"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
#define HEADING_OFFSET 360
#define MAX_HEADING 180

#define BUFFER_SIZE 8

// ===================================== Globals ======================================
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Number of ultrasonic sensors for external use
uint8_t numUS = US_NUM;

// Array of ultrasonic sensors
extern UltrasonicSensor_t usArray[US_NUM]; 
uint16_t* usDistances = new uint16_t[US_NUM]; // Array of computed distances

// IR triangulating sensor structs
irTri_sensor_t top_IRTriSensor = {IRTRI_0_PIN, IRTRI_0_TYPE};
irTri_sensor_t bottom_IRTriSensor = {IRTRI_1_PIN, IRTRI_1_TYPE};


// Circular buffers for sensors
circBuffer_t* us0Buffer = new circBuffer_t[BUFFER_SIZE]; // Left US
circBuffer_t* us1Buffer = new circBuffer_t[BUFFER_SIZE]; // Right US
circBuffer_t* ir0Buffer = new circBuffer_t[BUFFER_SIZE]; // Top IR
circBuffer_t* ir1Buffer = new circBuffer_t[BUFFER_SIZE]; // Bottom IR
circBuffer_t* headingBuffer = new circBuffer_t[BUFFER_SIZE];


// ===================================== Function Definitions =========================
/** 
 * @brief Setup the sensors for use
 * 
 * @return success (0) or failure (1)
 */
bool sensors_init(void) {
    // Initialise ultrasonic sensor counters
    usCounterInit(); 

    // Initialise the IMU
    if(!bno.begin()) {
        return 1;
    }

    // Add ultrasonic sensors to array
    #ifdef US_0
        usAddToArray(US_TRIG_0, US_ECHO_0, 0);
    #endif
    #ifdef US_1
        usAddToArray(US_TRIG_1, US_ECHO_1, 1);
    #endif
    #ifdef US_2
        usAddToArray(US_TRIG_2, US_ECHO_2, 2);
    #endif
    #ifdef US_3
        usAddToArray(US_TRIG_3, US_ECHO_3, 3);
    #endif

    // Initialise the buffers
    circBuffer_init(us0Buffer, BUFFER_SIZE);
    circBuffer_init(us1Buffer, BUFFER_SIZE);
    circBuffer_init(ir0Buffer, BUFFER_SIZE);
    circBuffer_init(ir1Buffer, BUFFER_SIZE);
    circBuffer_init(headingBuffer, BUFFER_SIZE);

    return 0;
}

/** 
 * @brief Get the current distance messured by an IR triangulating sensor
 * @param sensor The sensor to read from
 * 
 * @return the distance in mm
 */
static uint16_t getIRTriDistance(irTri_sensor_t sensor) {
    uint16_t rawValue = analogRead(sensor.pin);
    // uint16_t distance = 0;

    // if (sensor.type == 0) {
    //     return 0;
    // } else if (sensor.type == 1) {
    //     return 0;
    // } else if (sensor.type == IRTRI_20_150) { // 20-150cm IR Sensor
    //     distance = rawValue;
    // }
    return rawValue;
}   


/** 
 * @brief Ping the ultrasonic sensor array
 * 
 */
static void pingUS(void) {
    // Send a pulse to each ultrasonic sensor
    usPingArray(usArray, US_NUM); 
}


/** 
 * @brief Get the distance from the ultrasonic sensor array
 * @param distances The array to store the distances in
 *  
 */
static void getUSDistances(uint16_t distances[US_NUM]) {
    // Get the distances from each ultrasonic sensor
    usCalcArray(usArray, US_NUM, distances);
}


/** 
 * @brief Get the current heading of the robot
 * 
 * @return the heading in degrees
 */
static int16_t getHeading(void) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int16_t heading = euler.x();

    if (heading > MAX_HEADING) {
        heading -= HEADING_OFFSET;
    }

    return heading;
}

/** 
 * @brief Get the filtered heading of the robot
 * 
 * @return the filtered heading in degrees
 */
static int32_t getFilteredHeading(void) {
    int32_t heading = 0;

    for (uint8_t i = 0; i < headingBuffer->size; i++) {
        heading += headingBuffer->data[i];
    }

    return headingBuffer->data[0];
}


/** 
 * @brief get the filtered distance from the left ultrasonic sensor
 * 
 * @return the filtered distance in mm
 */
static int32_t getFilteredLeftUS(void) {
    uint16_t distance = 0;

    for (uint8_t i = 0; i < us0Buffer->size; i++) {
        distance += us0Buffer->data[i];
    }

    return distance / us0Buffer->size;
}


/** 
 * @brief get the filtered distance from the right ultrasonic sensor
 * 
 * @return the filtered distance in mm
 */
static int32_t getFilteredRightUS(void) {
    int32_t distance = 0;

    for (uint8_t i = 0; i < us1Buffer->size; i++) {
        distance += us1Buffer->data[i];
    }

    return distance / us1Buffer->size;
}


/** 
 * @brief get the filtered distance from the top IR sensor
 * 
 * @return the filtered distance in mm
 */
static int32_t getFilteredTopIR(void) {
    int32_t distance = 0;

    for (uint8_t i = 0; i < ir0Buffer->size; i++) {
        distance += ir0Buffer->data[i];
    }

    return distance / ir0Buffer->size;
}


/** 
 * @brief get the filtered distance from the bottom IR sensor
 * 
 * @return the filtered distance in mm
 */
static int32_t getFilteredBottomIR(void) {
    int32_t distance = 0;

    for (uint8_t i = 0; i < ir1Buffer->size; i++) {
        distance += ir1Buffer->data[i];
    }

    return distance / ir1Buffer->size;
}


/** 
 * @brief Update the robot info struct with the sensor data
 * @param robotInfo The robot info struct to update
 * 
 */
void sensors_updateInfo(RobotInfo_t* robotInfo) {
    robotInfo->IMU_Heading = getFilteredHeading();
    robotInfo->USLeft_Distance = getFilteredLeftUS();
    robotInfo->USRight_Distance= getFilteredRightUS();
    robotInfo->IRTop_Distance = getFilteredTopIR();
    robotInfo->IRBottom_Distance = getFilteredBottomIR();
}

/**
 * @breif update the sensors to recive new readings
 * 
 */
void sensors_update(void) {
    // Add the new readings to the buffers
    getUSDistances(usDistances);
    circBuffer_write(us0Buffer, usDistances[0]);
    circBuffer_write(us1Buffer, usDistances[1]);
    circBuffer_write(ir0Buffer, getIRTriDistance(top_IRTriSensor));
    circBuffer_write(ir1Buffer, getIRTriDistance(bottom_IRTriSensor));
    circBuffer_write(headingBuffer, getHeading());

    // Start new ping cycle
    pingUS();

    // delay(30); // Allow the sensors to update
}
