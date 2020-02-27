/*
 * Drone remote control
 *
 * Author: Romain JL. Fétick
 *
 * History:
 *   01 Nov 2019 - creation
 *   16 Feb 2020 - simplification and better mapping
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "remote_control_define.h"

SoftwareSerial HC12(RADIO_PIN_TX, RADIO_PIN_RX);
int last_val = 0;

#include "communicate_user.h"
#include "communicate_drone.h"

void setup() {
  Serial.begin(250000);
  Serial.println(F("DRONE_REMOTE_CONTROL.ino [16 Feb 2020]"));
  HC12.begin(RADIO_BAUD);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(POTEN_PIN,INPUT);
  ask_init_poten();
}

void loop() {
  byte val = readPoten();
  int diff = last_val-val;
  if(abs(diff)>SIGMA){
    write_message(1,val);
    last_val = val;
  }

  if(Serial.available()){
    char msg[2];
    int index = retrieve_serial(msg);
    if(index==2){
      write_message(msg[0],msg[1]);
    }else{
      Serial.println("Error retrieving serial");
    }
    while(Serial.available()){ Serial.read(); } // empty serial
  }
}
