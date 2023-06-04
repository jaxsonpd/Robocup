//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdio.h>

// initalise the sensors
void sensors_init();

// Read ultrasonic value
void read_ultrasonic(/* Parameters */);

// Read infrared value
uint16_t read_infrared(/* Parameters */);

void read_colour(/* Parameters */);

// Pass in data and average the lot
void sensor_average(/* Parameters */);

#endif /* SENSORS_H_ */
