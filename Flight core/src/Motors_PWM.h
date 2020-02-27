/*
 * Control motors using PWM signal on MOSFET transistors.
 */

#include <Arduino.h>

extern const byte MOTOR_PIN[4];
extern byte MOTOR_SPEED[4];

void MOTOR_CONNECT(){
  for(int i=0;i<4;i++){ pinMode(MOTOR_PIN[i],OUTPUT); }
}

void MOTOR_SET(){
  for(int i=0;i<4;i++){ analogWrite(MOTOR_PIN[i],MOTOR_SPEED[i]); }
}
