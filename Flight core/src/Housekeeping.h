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

unsigned long T_HK_ERR;
byte HK_LED_STATE;

void housekeeping_on_error(){
  unsigned long Tcur = millis();
  if((Tcur-T_HK_ERR)>BLINK_TIME){
    HK_LED_STATE = !HK_LED_STATE;
    digitalWrite(LED_BUILTIN,HK_LED_STATE);
    T_HK_ERR = Tcur;
  }
}

void UPDATE_HOUSEKEEPING(){
  // Get voltage
  float newMes = analogRead(PIN_V_CHECK)*HK_BIT_2_VOLT;
  BATTERY_VOLT += FILTER_VOLT_ACCEPT*(newMes - BATTERY_VOLT); // low pass filter
  if(BATTERY_VOLT<MIN_VOLT){ housekeeping_on_error(); } // Error if voltage is too low
  // Set drone status
  if((MOTOR_SPEED_AVG*BATTERY_VOLT)>VOLT_LEVEL_FLIGHT){
    DRONE_STATUS = DRONE_STATUS_FLIGHT;
  }else{
    DRONE_STATUS = DRONE_STATUS_LANDED;
  }
}

void INIT_HOUSEKEEPING(){
  pinMode(PIN_V_CHECK,INPUT);
  T_HK_ERR = millis();
  HK_LED_STATE = LOW;
  for(int i=0;i<100;i++){ UPDATE_HOUSEKEEPING(); }
}

void HOUSEKEEPING_PRINT_STATE(){
  Serial.print(F("ACC: "));
  Serial.print(STATE[6]);
  Serial.print(F(" "));
  Serial.print(STATE[7]);
  Serial.print(F(" AVG: "));
  Serial.print(MOTOR_SPEED_AVG);
  Serial.print(F(" MOT: "));
  for(int i=0;i<4;i++){
    Serial.print(MOTOR_SPEED[i]-MOTOR_SPEED_AVG);
    Serial.print(F(" "));
  }
  Serial.println(F(" "));
}
