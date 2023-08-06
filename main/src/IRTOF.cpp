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
#include <SparkFunSX1509.h>

#include "VL53L0X.h"

#include "circBuffer.hpp"


// ===================================== Types/Constants ==============================
#define ADDRESS_DEFAULT 0b0101001 // Defult IR TOF Address

// ===================================== Globals ======================================

// ===================================== Function Definitions =========================
/**
 * @brief The IRTOF class used to interface with the VL53L0x Time of Flight sensor.
 * 
 */
class IRTOF {
    public:

    private:
        circBuffer_t* distanceBuffer;
        uint8_t bufferSize;
        uint8_t XSHUT;
        uint8_t address;
        SX1509* externalIO;
        VL53L0X sensor;
}:

/**
 * @brief Initialise a new IRTOF object
 * 
 * @param address The address of the VL53L0x sensor
 * @param XSHUT The pin connected to the XSHUT pin of the VL53L0x sensor
 * @param bufferSize The size of the buffer to store the sensor readings in
 * @param externalIO The SX1509 object used to interface with the XSHUT pin
 * 
 * @return 1 if the sensor was successfully initialised, 0 otherwise
 */
bool IRTOF::init(uint8_t address, uint8_t XSHUT, uint8_t bufferSize, SX1509* externalIO) {
    // Assign the parameters to the class variables
    address = address;
    XSHUT = XSHUT;
    bufferSize = bufferSize;
    externalIO = externalIO;

    // Initialise the buffer
    distanceBuffer = new circBuffer_t[bufferSize];

    // Initialise pins
    externalIO->pinMode(XSHUT, OUTPUT);
    externalIO->digitalWrite(XSHUT, LOW); // Reset the sensor
    delay(100);

    // Initialise the sensor
    externalIO->digitalWrite(XSHUT, HIGH); // Set the sensor to active mode
    delay(100);

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
void IRTOF::deInit(void) {
    sensor.setAddress(ADDRESS_DEFAULT);
    delay(100);
    
    externalIO->digitalWrite(XSHUT, LOW);  
}

/**
 * @brief Add a new reading to the buffer
 * 
 */
void IRTOF::update(void) {
    circBuffer_write(distanceBuffer, sensor.readRangeContinuousMillimeters());
}

/**
 * @brief Get the latest reading from the buffer
 * 
 * @return The latest reading from the buffer
 */
int32_t IRTOF::getLatest(void) {
    return circBuffer_average(distanceBuffer);
}