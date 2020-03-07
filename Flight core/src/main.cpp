/*
 * Flight controller for quadcopter drone
 * --------------------------------------
 *
 * This is the main file, it requires different headers to work.
 * You also need the remote control project to send data to the drone.
 * Capital names in the code indicate global definitions or global variables.
 *
 * Todo list:
 *  Tune the corrector PID gains for the linear acceleration
 *  Add the BMP180 sensor (low pass filtered) and Kalman (?)
 *  Send command according to housekeeping measured voltage
 *
 * History:
 *  13 Feb 2020 - creation from drone_light.ino
 *  15 Feb 2020 - start writing linear acceleration control loop
 *  26 Feb 2020 - moved project to PlatformIO
 *                gathered all #define into one unique header file
 *  07 Mar 2020 - better management of errors and fatal errors
 *
 * Author: Romain JL. Fétick (Toulouse, France)
 *
 * License: MIT
 */

// INCLUDE LIBRARIES
#include <Arduino.h>
#include <Wire.h>

// GLOBAL DEFINITIONS
#include "drone_define.h"

// GLOBAL VARIABLES
int ERR = NO_ERROR; // error variable
float STATE[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // Yaw, Pitch, Roll, GyroX, GyroY, GyroZ, aX, aY, aZ
float COMMAND[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; // ypr+xyz
int DRONE_STATUS = DRONE_STATUS_LANDED;

float BATTERY_VOLT = 7.5; // approx. 7.5V, helps for housekeeping convergence at start

const byte MOTOR_PIN[4] = {5,6,10,11};
byte MOTOR_SPEED_AVG = 0;
byte MOTOR_OFFSET[4] = {0,0,0,0};
byte MOTOR_SPEED[4] = {0,0,0,0};

// INCLUDE HEADERS
#include "Radio_HC12.h"
#include "Housekeeping.h"
#include "Motors_PWM.h"
#include "Correct.h"
#include "Sensor_MPU6050.h"

// TIME MANAGEMENT
unsigned long Told, Tnew;
float DT;

/*** ERROR management ***/
int error_nb_blink = 0;
bool error_led_state = false;
unsigned long Terr = 0;

void process_error(){
  if(error_nb_blink){
    unsigned long Tcur = millis();
    if((Tcur-Terr)>BLINK_TIME){
      error_led_state = !error_led_state;
      digitalWrite(LED_BUILTIN,error_led_state);
      Terr = Tcur;
      error_nb_blink -= 1;
    }
  }
}

void raise_error(){
  error_nb_blink = 6;
}

void raise_fatal_error(){
  // Fatal error turns off definitely the motors, and enters in infinite error loop
  for(int i=0;i<4;i++){ MOTOR_SPEED[i] = 0; }
  MOTOR_SET();
  while(true){
    raise_error();
    process_error();
  }
}

/*** SETUP ***/
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH); // Indicate that setup is not finished

  if(VERBOSE){
    Serial.begin(BAUDRATE);
    Serial.println(F("DRONE FLIGHT CORE [07 Mar 2020]"));
  }

  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  INIT_HOUSEKEEPING();

  MOTOR_CONNECT();

  INIT_MPU6050();
  if(ERR!=NO_ERROR){
    if(VERBOSE){Serial.println(F("FATAL ERROR: Could not connect to MPU6050"));}
    raise_fatal_error();
  }

  RADIO_INIT();
  if(ERR!=NO_ERROR){
    if(VERBOSE){Serial.println(F("FATAL ERROR: Could not connect to radio HC12"));}
    raise_fatal_error();
  }

  digitalWrite(LED_BUILTIN,LOW); // setup is finished

  Told = micros();
}


/*** LOOP ***/
void loop() {

  UPDATE_HOUSEKEEPING();
  if(ERR!=NO_ERROR){
    //if(VERBOSE){Serial.println(F("ERROR: Housekeeping"));}
    raise_error();
  }

  RADIO_RECEIVE();
  if(ERR!=NO_ERROR){
    if(ERR==FATAL_ERROR){ raise_fatal_error(); } // Radio can raise a fatal error
    if(VERBOSE){Serial.println(F("ERROR: Radio"));}
    raise_error();
  }

  UPDATE_MPU6050();
  if(ERR!=NO_ERROR){
    if(VERBOSE){Serial.println(F("ERROR: MPU6050"));}
    raise_error();
  }

  if((abs(STATE[1])>ANGLE_MAX_ERROR) || (abs(STATE[2])>ANGLE_MAX_ERROR)){
    if(VERBOSE){Serial.println(F("FATAL ERROR: Too much angle"));}
    raise_fatal_error();
  }

  Tnew = micros();
  DT = Tnew-Told;
  Told = Tnew;

  UPDATE_CORRECTION(DT);
  if(ERR!=NO_ERROR){
    if(VERBOSE){Serial.println(F("ERROR: Corrector"));}
    raise_error();
  }

  MOTOR_SET();

  if(VERBOSE){ HOUSEKEEPING_PRINT_STATE(); }

  process_error();
}
