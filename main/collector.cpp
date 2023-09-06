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
#define ELECTROMAGNET_PIN 20

#define RELEASE_ANGLE 140
#define COLLECT_ANGLE 15
#define STANDBY_ANGLE 100

enum states {
  collect = 0,
  release,
  standby
};

Servo craneServo;


bool crane_setup()
{   
  craneServo.attach(SERVO_1_PIN); 
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  return 0;
}


void crane_move_weight() {
  static int state = standby;
  switch(state) {
    case standby:
      craneServo.write(STANDBY_ANGLE);
      digitalWrite(ELECTROMAGNET_PIN, LOW);
      state = collect;
      break;
    case collect:
      craneServo.write(COLLECT_ANGLE);
      digitalWrite(ELECTROMAGNET_PIN, HIGH);
      state = release;
      break;
    case release:
      craneServo.write(RELEASE_ANGLE);
      digitalWrite(ELECTROMAGNET_PIN, HIGH);
      state = standby;
      
  };

}
