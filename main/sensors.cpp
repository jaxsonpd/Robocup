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
#include <SparkFunSX1509.h>

#include "sensors.hpp"
#include "src/ultrasonic.hpp"
#include "robotInformation.hpp"
#include "src/circBuffer.hpp"
#include "src/IRTOF.hpp"

#include "src/USConfig.hpp"
#include "src/IRTOFConfig.hpp"

#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
// Heading constants
#define HEADING_OFFSET 360
#define MAX_HEADING 180
#define HEADING_BOUND 2

// Buffer sizes for sensor readings
#define BUFFER_SIZE 8
#define IRTOF_BUFFER_SIZE 1

#define SX1509_ADDRESS 0x3F // TOF IO expander address

// ===================================== Globals ======================================
// IMU Setup
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Create an SX1509 object to be used throughout
SX1509 io; 

// Array of ultrasonic sensors
extern UltrasonicSensor_t usArray[US_NUM]; 
uint16_t* usDistances = new uint16_t[US_NUM]; // Array of computed distances

// Circular buffers for sensors
circBuffer_t* us0Buffer = new circBuffer_t[BUFFER_SIZE]; // Left US
circBuffer_t* us1Buffer = new circBuffer_t[BUFFER_SIZE]; // Right US
circBuffer_t* headingBuffer = new circBuffer_t[BUFFER_SIZE];

// create IR TOF sensor objects
IRTOF irTOF0;
IRTOF irTOF1;


// ===================================== Function Definitions =========================
/** 
 * @brief Setup the sensors for use
 * 
 * @return success (0) or failure (1)
 */
bool sensors_init(void) {
    // Initialise the IO expander
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    io.begin(SX1509_ADDRESS);

    // Initialise the IMU
    if(!bno.begin()) {
        return 1;
    }

    // Initialise the ultrasonic sensors
    init_USTOF();
    
    // Initailise the IR TOF sensors
    init_IRTOF();

    // Initialise the buffers
    circBuffer_init(us0Buffer, BUFFER_SIZE);
    circBuffer_init(us1Buffer, BUFFER_SIZE);
    circBuffer_init(headingBuffer, BUFFER_SIZE);

    return 0;
}


/**
 * @brief initialise the IR TOF sensor
 * 
 */
static void init_IRTOF(void) {
    // Reset the sensors
    io.pinMode(IRTOF_0_XSHUT_PIN, OUTPUT);
    io.pinMode(IRTOF_1_XSHUT_PIN, OUTPUT);
    io.digitalWrite(IRTOF_0_XSHUT_PIN, LOW);
    io.digitalWrite(IRTOF_1_XSHUT_PIN, LOW);
    delay(100);

    // Initialise sensor 1
    io.digitalWrite(IRTOF_0_XSHUT_PIN, HIGH);
    delay(100);
    irTOF0.init(IRTOF_0_ADDR, IRTOF_0_XSHUT_PIN, IRTOF_BUFFER_SIZE);

    // Initialise sensor 2
    io.digitalWrite(IRTOF_1_XSHUT_PIN, HIGH);
    delay(100);
    irTOF1.init(IRTOF_1_ADDR, IRTOF_1_XSHUT_PIN, IRTOF_BUFFER_SIZE);
}

/**
 * @brief Initialise the US TOF sensors
 * 
 */
static void init_USTOF(void) {
    // Initialise ultrasonic sensor counters
    usCounterInit(); 

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
}

/*
 * @brief de initialize the sensor componetes
 *
 */
void sensor_deInit(void) {
    irTOF0.deInit();    
    io.digitalWrite(IRTOF_0_XSHUT_PIN, LOW);  
    
    irTOF1.deInit();
    io.digitalWrite(IRTOF_1_XSHUT_PIN, LOW); 
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
 * @brief Update the robot info struct with the sensor data
 * @param robotInfo The robot info struct to update
 * 
 */
void sensors_updateInfo(RobotInfo_t* robotInfo) {
    robotInfo->IMU_Heading = circBuffer_average(headingBuffer);
    robotInfo->USLeft_Distance = circBuffer_average(us0Buffer);
    robotInfo->USRight_Distance= circBuffer_average(us1Buffer);
    robotInfo->IRTop_Distance = irTOF0.getDistance();
    robotInfo->IRBottom_Distance = irTOF1.getDistance();
}

/**
 * @brief update the sensors to recive new readings
 * 
 */
void sensors_update(void) {
    // Add the new readings to the buffers
    usCalcArray(usArray, US_NUM, usDistances);
    
    // Start new ping cycle
    usPingArray(usArray, US_NUM)
    
    // Update Buffers
    circBuffer_write(us0Buffer, usDistances[0]);
    circBuffer_write(us1Buffer, usDistances[1]);
    circBuffer_write(headingBuffer, getHeading());

    // Update IR TOF sensors
    irTOF0.update();
    irTOF1.update();
}

    
