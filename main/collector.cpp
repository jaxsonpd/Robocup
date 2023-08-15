/** 
 * @file collector.cpp
 * @brief crane functionality
 * @author Campbell Morris
 * @date 2023-08-13
 */

#include <Arduino.h>
#include "collector.hpp"
#include <Servo.h>


#define SERVO_1_PIN 29
#define SERVO_2_PIN 28

#define RELEASE_ANGLE 180
#define COLLECT_ANGLE 0
#define STANDBY_ANGLE 90

enum states {
  collect = 0,
  release,
  standby
};

Servo craneServo;


bool crane_setup()
{   
  craneServo.attach(SERVO_1_PIN); 
  
  return 0;
}


void crane_move_weight() {
  static int state = standby;
  switch(state) {
    case standby:
      craneServo.write(STANDBY_ANGLE);
      state = collect;
      break;
    case collect:
      craneServo.write(COLLECT_ANGLE);
      state = release;
      break;
    case release:
      craneServo.write(RELEASE_ANGLE);
      state = standby;
      
  };

}
