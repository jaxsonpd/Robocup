/** 
 * @file IRTOF.cpp
 * @brief This is the source file for the IRTOF class, which is used to interface with the VL53L0x Time of Flight sensor.
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-08-06
 */


// ===================================== Includes =====================================
#include <arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include "VL53L0X.h"
#include "IRTOF0.hpp"

#include "circBuffer.hpp"


// ===================================== Types/Constants ==============================
#define ADDRESS_DEFAULT 0b0101001 // Defult IR TOF Address

// ===================================== Globals ======================================

// ===================================== Function Definitions =========================


/**
 * @brief Initialise a new IRTOF object
 * 
 * @param address The address of the VL53L0x sensor
 * @param XSHUT The pin connected to the XSHUT pin of the VL53L0x sensor
 * @param bufferSize The size of the buffer to store the sensor readings in
 * 
 * @return 1 if the sensor was successfully initialised, 0 otherwise
 */
bool IRTOF0::init(uint8_t address, uint8_t XSHUT, uint8_t bufferSize) {
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



    // Start continuous back-to-back mode (take readings as fast as possible).
    sensor.startContinuous();

    return 1;
}


/**
 * @brief De-initialise the IRTOF object
 * 
 */
void IRTOF0::deInit(void) {
    sensor.setAddress(ADDRESS_DEFAULT);
    delay(100);
}

/**
 * @brief Add a new reading to the buffer
 * 
 */
void IRTOF0::update(void) {
    int32_t sensorReading = sensor.readRangeContinuousMillimeters();
    circBuffer_write(distanceBuffer, sensorReading);

}

/**
 * @brief Get the latest reading from the buffer
 * 
 * @return The latest reading from the buffer
 */
int32_t IRTOF0::getDistance(void) {
    return circBuffer_average(distanceBuffer);
}