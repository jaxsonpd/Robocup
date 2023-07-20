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

#include "USConfig.hpp"
#include "IRConfig.hpp"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================

// ===================================== Globals ======================================
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Number of ultrasonic sensors for external use
uint8_t numUS = US_NUM;

// Array of ultrasonic sensors
extern UltrasonicSensor_t usArray[US_NUM]; 

// IR triangulating sensor structs
irTri_sensor_t top_IRTriSensor = {IRTRI_0_PIN, IRTRI_0_TYPE};
irTri_sensor_t bottom_IRTriSensor = {IRTRI_1_PIN, IRTRI_1_TYPE};

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
    
    return 0;
}

/** 
 * @brief Get the current distance messured by an IR triangulating sensor
 * @param sensor The sensor to read from
 * 
 * @return the distance in mm
 */
uint16_t sensors_getIRTriDistance(irTri_sensor_t sensor) {
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
void sensors_pingUS(void) {
    // Send a pulse to each ultrasonic sensor
    usPingArray(usArray, US_NUM); 
}


/** 
 * @brief Get the distance from the ultrasonic sensor array
 * @param distances The array to store the distances in
 *  
 */
void sensors_getUSDistances(uint16_t distances[US_NUM]) {
    // Get the distances from each ultrasonic sensor
    usCalcArray(usArray, US_NUM, distances);
}


/** 
 * @brief Get the current heading of the robot
 * 
 * @return the heading in degrees
 */
int16_t sensors_getHeading(void) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    return euler.x();
}


/** 
 * @brief Update the robot info struct with the sensor data
 * @param robotInfo The robot info struct to update
 * 
 */
void sensors_updateInfo(RobotInfo_t* robotInfo) {
    // Update the ultrasonic sensor data
    uint16_t* usDistances = new uint16_t[US_NUM]; 

    sensors_getUSDistances(usDistances);
    robotInfo->USLeft_Distance = usDistances[0];
    robotInfo->USRight_Distance = usDistances[1];

    delete[] usDistances;
    
    // Update the IR sensor data
    robotInfo->IRTop_Distance = sensors_getIRTriDistance(top_IRTriSensor);
    robotInfo->IRBottom_Distance = sensors_getIRTriDistance(bottom_IRTriSensor);

    // Update the IMU data
    robotInfo->IMU_Heading = sensors_getHeading();
}

/**
 * @breif update the sesnors to recive new readings
 * 
 */
void sensors_update(void) {
  sensors_pingUS();

  delay(30); // Allow the sensors to update
}
