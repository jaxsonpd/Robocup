//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.


#include "sensors.h"
#include "Arduino.h"

#include <stdio.h>
// Local definitions
#define IR_SENSOR_1 A8

// Read ultrasonic value
void sensors_init(){
    Serial.println("Initialising sensors \n");

}


void read_ultrasonic(/* Parameters */){
  Serial.println("Ultrasonic value \n");
}

// Read infrared value
uint16_t read_infrared(/* Parameters */){
    return analogRead(IR_SENSOR_1); // Read the value from the sensor
}

// Read colour sensor value
void read_colour(/* Parameters */){
  Serial.println("colour value \n");  
}

// Pass in data and average the lot
void sensor_average(/* Parameters */){
  Serial.println("Averaging the sensors \n");
}

