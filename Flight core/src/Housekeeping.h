/*
 * Monitor the power supply, check and print drone state.
 */

#include <Arduino.h>
#include "drone_define.h"

extern float STATE[9];
extern float COMMAND[6];
extern byte MOTOR_SPEED[4];
extern byte MOTOR_SPEED_AVG;
extern int DRONE_STATUS;
extern float BATTERY_VOLT;
extern int ERR;

void UPDATE_HOUSEKEEPING(){
  ERR = NO_ERROR;
  // Get voltage
  float newMes = analogRead(PIN_V_CHECK)*HK_BIT_2_VOLT;
  BATTERY_VOLT += FILTER_VOLT_ACCEPT*(newMes - BATTERY_VOLT); // low pass filter
  if(BATTERY_VOLT<MIN_VOLT){ ERR = SIMPLE_ERROR; } // Error if voltage is too low
  // Set drone status
  if((MOTOR_SPEED_AVG*BATTERY_VOLT)>VOLT_LEVEL_FLIGHT){
    DRONE_STATUS = DRONE_STATUS_FLIGHT;
  }else{
    DRONE_STATUS = DRONE_STATUS_LANDED;
  }
}

void INIT_HOUSEKEEPING(){
  pinMode(PIN_V_CHECK,INPUT);
  for(int i=0;i<100;i++){ UPDATE_HOUSEKEEPING(); } // initialize voltage
}

void HOUSEKEEPING_PRINT_STATE(){
  Serial.print(F("COM: "));
  Serial.print(COMMAND[1]);
  Serial.print(F(" "));
  Serial.print(COMMAND[2]);
  Serial.print(F(" AVG: "));
  Serial.print(MOTOR_SPEED_AVG);
  Serial.print(F(" MOT: "));
  for(int i=0;i<4;i++){
    Serial.print(MOTOR_SPEED[i]-MOTOR_SPEED_AVG);
    Serial.print(F(" "));
  }
  Serial.println(F(" "));
}
