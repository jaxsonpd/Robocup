/** 
 * @file sensors.cpp
 * @brief Sensor control functions for the robot cup project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-07-10
 */


// ===================================== Includes =====================================
#include <Arduino.h>

#include "sensors.h"
#include "USConfig.h"
#include "ultrasonic.hpp"

#include <stdint.h>
#include <stdbool.h>


// ===================================== Constants ====================================
// ** IR triangulating sensor parameters **
#define IRTRI_20_150 2 // 20-150cm sensor type
#define IRTRI_20_150_RAWMIN 0 // 20-150cm sensor minium raw value (150cm)
#define IRTRI_20_150_RAWMAX 1024 // 20-150cm sensor maximum raw value (20cm)

// ===================================== Globals ======================================
// Number of ultrasonic sensors for external use
extern uint8_t numUS = US_NUM;


// ===================================== Function Definitions =========================
/** 
 * @brief Setup the sensors for use
 * 
 * @return success (0) or failure (1)
 */
bool sensors_init(void) {
    // Initialise ultrasonic sensor counters
    usCounterInit(); 
    // Add ultrasonic sensors to array
    usAddToArray(US_TRIG_0, US_ECHO_0, 0); 
    usAddToArray(US_TRIG_1, US_ECHO_1, 1); 
}

/** 
 * @brief Get the current distance messured by an IR triangulating sensor
 * @param sensor The sensor to read from
 * 
 * @return the distance in mm
 */
uint16_t sensors_getIRTriDistance(irTri_sensor_t sensor) {
    uint16_t rawValue = analogRead(sensor.pin);

    uint16_t distance = 0;

    if (sensor.type == 0) {
        return 0;
    } else if (sensor.type == 1) {
        return 0;
    } else if (sensor.type == IRTRI_20_150) { // 20-150cm
        distance = map(rawValue, IRTRI_20_150_RAWMIN, IRTRI_20_150_RAWMAX, 200, 1500);
    }

    return distance;
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
void sensors_getUSDistance(uint16_t distances[US_NUM]) {
    // Get the distances from each ultrasonic sensor
    usCalcArray(usArray, US_NUM, distances);
}


