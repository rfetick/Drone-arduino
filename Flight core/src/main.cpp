/*
 *  Flight controller for quadcopter drone.
 *
 * TODO:
 *  Tune the corrector PID gains (angle + speed correction)
 *  Add the BMP180 sensor (low pass filtered) and Kalman (?)
 *  Send command according to housekeeping measured voltage
 *  Stop initialization if error is detected
 *
 * Author: Romain JL. Fétick (Toulouse, France)
 *
 * History:
 *  13 Feb 2020 - creation from drone_light.ino
 *  15 Feb 2020 - start writing linear acceleration control loop
 *  26 Feb 2020 - moved project to PlatformIO
 *                gathered all #define into one unique header file
 *
 * License: MIT
 */

// INCLUDE LIBRARIES
#include <Arduino.h>
#include <Wire.h>

// GLOBAL DEFINITIONS
#include "drone_define.h"

// GLOBAL VARIABLES
int ERR = 0; // error variable
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
#include "Sensor_MPU6050_tockn.h"

// TIME MANAGEMENT
unsigned long Told, Tnew;
float DT;

/*** SETUP ***/
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH); // Indicate that setup is not finished

  if(VERBOSE){
    Serial.begin(BAUDRATE);
    Serial.println(F("DRONE_FLIGHT.ino [15 Feb 2020]"));
  }

  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  ERR = -1;
  INIT_MPU6050();
  if(VERBOSE & (ERR!=0)){Serial.println(F("FATAL ERROR: Could not connect to MPU6050"));}

  ERR = -1;
  RADIO_INIT();
  if(VERBOSE & (ERR!=0)){Serial.println(F("FATAL ERROR: Could not connect to radio HC12"));}

  MOTOR_CONNECT();
  INIT_HOUSEKEEPING();

  digitalWrite(LED_BUILTIN,LOW); // setup is finished

  Told = micros();
}


/*** LOOP ***/
void loop() {
  UPDATE_HOUSEKEEPING();

  ERR = -1;
  RADIO_RECEIVE();
  if(VERBOSE & (ERR!=0)){Serial.println(F("ERROR: Corrupted radio message (message is ignored)"));}

  ERR = -1;
  UPDATE_MPU6050();
  if(VERBOSE & (ERR!=0)){Serial.println(F("ERROR: Could not get data from MPU6050 (state might be aberrant)"));}

  Tnew = micros();
  DT = Tnew-Told;
  Told = Tnew;
  UPDATE_CORRECTION(DT);
  MOTOR_SET();
  if(VERBOSE){HOUSEKEEPING_PRINT_STATE();}
}
