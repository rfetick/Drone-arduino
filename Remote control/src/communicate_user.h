#include <Arduino.h>
#include "remote_control_define.h"
extern int last_val;

byte readPoten(){
  int poten = analogRead(POTEN_PIN);
  byte val = map(poten,POTEN_MIN,POTEN_MAX,0,255);
  return val;
}

void ask_init_poten(){
  byte val = readPoten();

  while(val>SIGMA){
    Serial.print(F("Set potentiometer to 0   (current="));
    Serial.print(val);
    Serial.println(")");
    val = readPoten();
    delay(200);
  }

  Serial.println(F("Ready to transmit data"));
  last_val = val;
}


int retrieve_serial(char* msg){
  byte index = 0;
  msg[0] = 0;
  msg[1] = 0;
  while(Serial.available()){
    char c = Serial.read();
    if((c==' ') | (c=='\n')){
      index++;
    }else{
      if(index<2){ msg[index] = 10*msg[index] + (c-'0'); }
    }
  }
  return index;
}
