#include <Arduino.h>
#include <SoftwareSerial.h>
#include "remote_control_define.h"

extern SoftwareSerial HC12;

void write_message(byte target, byte value){
  char msg[3];
  msg[0] = target;
  msg[1] = value;
  msg[2] = msg[0] ^ msg[1];

  Serial.print((unsigned char)msg[0],DEC);
  Serial.print(" ");
  Serial.println((unsigned char)msg[1],DEC);

  digitalWrite(LED_BUILTIN,HIGH);
  HC12.write(msg);
  delay(MSG_DELAY); // avoid writing consecutive messages
  digitalWrite(LED_BUILTIN,LOW);
}
