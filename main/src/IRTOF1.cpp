/** 
 * @file IRTOF1.cpp
 * @brief This is the source file for the IRTOF class, which is used to interface with the VL53L1x Time of Flight sensor.
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-08-20
 */


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include "VL53L1X.h"
#include "IRTOF1.hpp"

#include "circBuffer.hpp"

// ===================================== Types/Constants ==============================
#define ADDRESS_DEFAULT 0b0101001 // Defult IR TOF Address

// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
/**
 * @brief Initialise a new IRTOF1 object
 * @param address The address of the VL53L1x sensor
 * @param XSHUT The pin connected to the XSHUT pin of the VL53L1x sensor
 * @param bufferSize The size of the buffer to store the sensor readings in
 * 
 * @return 1 if the sensor was successfully initialised, 0 otherwise
 */
bool IRTOF1::init(uint8_t address, uint8_t XSHUT, uint8_t bufferSize) {
    // Assign the parameters to the class variables
    address = address;
    XSHUT = XSHUT;
    bufferSize = bufferSize;

    // Initialise the buffer
    distanceBuffer = new circBuffer_t[bufferSize];
    circBuffer_init(distanceBuffer, bufferSize);

    sensor.setTimeout(500);
    if (!sensor.init()) {
        Serial.println("Failed to detect and initialise IR TOF sensor!");
        return 0;
    }
    sensor.setAddress(address);

    sensor.setDistanceMode(VL53L1X::Short);
    sensor.startContinuous(20);

    return 1;
}


/**
 * @brief Deinitialise the IRTOF1 object
 * 
 */
void IRTOF1::deInit(void) {
    sensor.setAddress(ADDRESS_DEFAULT);
    delay(100);
}


/**
 * @brief Update the IRTOF1 object
 * 
 */
void IRTOF1::update(void) {
    circBuffer_write(distanceBuffer, sensor.read(false));
}


/**
 * @brief Get the distance from the IRTOF1 object
 * 
 * @return int32_t The distance in mm
 */
int32_t IRTOF1::getDistance(void) {
    return circBuffer_average(distanceBuffer);
}