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
#include "src/ultrasonic.hpp"
#include "robotInformation.hpp"
#include "src/circBuffer.hpp"
#include "src/VL53L0X.h"

#include "src/USConfig.hpp"
#include "src/IRTriConfig.hpp"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
// Heading constants
#define HEADING_OFFSET 360
#define MAX_HEADING 180
#define HEADING_BOUND 2

// Buffer sizes for sensor readings
#define BUFFER_SIZE 8

// ===================================== Globals ======================================
// IMU Setup
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

// create IR TOF sensors
#ifdef IRTOF_0
    VL53L0X irTOF0;
#endif // IRTOF_0
#ifdef IRTOF_1
    VL53L0X irTOF1;
#endif // IRTOF_1


// ===================================== Function Definitions =========================
/** 
 * @brief Setup the sensors for use
 * 
 * @return success (0) or failure (1)
 */
bool sensors_init(void) {
    wire.begin();

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

    // Initailise the IR TOF sensors
    #ifdef IRTOF_0
        irTOF0.setTimeout(500);
        if (!irTOF0.init()) {
            Serial.println("Failed to detect and initialise IR TOF sensor 0!");
            return 1;
        }
        irTOF0.setAddress(IRTOF_0_ADDR);

        // Start continuous back-to-back mode (take readings as fast as possible).
        irTOF0.startContinuous();
    #endif // IRTOF_0
    #ifdef IRTOF_1
        irTOF1.setTimeout(500);
        if (!irTOF1.init()) {
            Serial.println("Failed to detect and initialise IR TOF sensor 1!");
            return 1;
        }
        irTOF1.setAddress(IRTOF_1_ADDR);

        // Start continuous back-to-back mode (take readings as fast as possible).
        irTOF1.startContinuous();
    #endif // IRTOF_1

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
    uint16_t distance = 0;

    if (sensor.type == 0) {
        return 0;
    } else if (sensor.type == 1) {
        return 0;
    } else if (sensor.type == IRTRI_20_150) { // 20-150cm IR Sensor
        distance = K_1 + (K_2 / (rawValue + K_3));
    }
    return distance;
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
    static int16_t prevHeading = 0;

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int16_t heading = euler.x();

    // Filter out bad zeros
    if (heading == 0 && (prevHeading > HEADING_BOUND && prevHeading < (360 - HEADING_BOUND))) { // Giving bad data
        heading = prevHeading;        
    } else {
        prevHeading = heading;
    }

    // Bound to 180 to -180
    if (heading > MAX_HEADING) {
        heading -= HEADING_OFFSET;
    }

    return heading;
}

/** 
 * @brief Get the current distance messured by an IR TOF sensor in continuous mode
 * @param sensor The sensor to read from
 * 
 * @return the distance in mm
 */
static uint32_t getIRTOFDistanceContinuous(VL53L0X sensor) {
    uint32_t distance = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    return distance;
}


/** 
 * @brief Get the filtered heading of the robot
 * 
 * @return the filtered heading in degrees
 */
static int32_t getFilteredHeading(void) {
    return circBuffer_average(headingBuffer);
}


/** 
 * @brief get the filtered distance from the left ultrasonic sensor
 * 
 * @return the filtered distance in mm
 */
static int32_t getFilteredLeftUS(void) {
    return circBuffer_average(us0Buffer);
}


/** 
 * @brief get the filtered distance from the right ultrasonic sensor
 * 
 * @return the filtered distance in mm
 */
static int32_t getFilteredRightUS(void) {
    return circBuffer_average(us1Buffer);
}


/** 
 * @brief get the filtered distance from the top IR sensor
 * 
 * @return the filtered distance in mm
 */
static int32_t getFilteredTopIR(void) {
    return circBuffer_average(ir0Buffer);
}


/** 
 * @brief get the filtered distance from the bottom IR sensor
 * 
 * @return the filtered distance in mm
 */
static int32_t getFilteredBottomIR(void) {
    return circBuffer_average(ir1Buffer);
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
    
    // Start new ping cycle
    pingUS();
    
    circBuffer_write(us0Buffer, usDistances[0]);
    circBuffer_write(us1Buffer, usDistances[1]);
    circBuffer_write(ir0Buffer, getIRTOFDistanceContinuous(irTOF0));
    circBuffer_write(ir1Buffer, getIRTOFDistanceContinuous(irTOF1));
    circBuffer_write(headingBuffer, getHeading());
}

    
