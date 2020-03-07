/*
 * Use radio module HC-12 to communicate with the ground
 *
 * The message structure is made of 3 bytes as following:
 *        [target, value, check]
 * A message smaller than 3 bytes is not valid and is not processed.
 * The check byte must satisfy the following bitwise XOR condition:
 *        check = target ^ value
 * Otherwise the message is not processed.
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "drone_define.h"

extern int ERR;
extern byte MOTOR_SPEED_AVG;
extern byte MOTOR_OFFSET[4];

SoftwareSerial hc12(RADIO_PIN_TX, RADIO_PIN_RX);

void RADIO_INIT(){
  hc12.begin(RADIO_BAUD);
  ERR = (int)(!hc12.isListening());
}

void process_message(unsigned char* msg){
  if((msg[0] ^ msg[1]) != msg[2]){ // message check operation
    ERR = SIMPLE_ERROR;
  }else{
    switch(msg[0]){
      case 1: MOTOR_SPEED_AVG = msg[1]; break;
      case 2: MOTOR_OFFSET[0] = msg[1]; break;
      case 3: MOTOR_OFFSET[1] = msg[1]; break;
      case 4: MOTOR_OFFSET[2] = msg[1]; break;
      case 5: MOTOR_OFFSET[3] = msg[1]; break;
      case 10: ERR = FATAL_ERROR; break; // stop message from remote control
      default: ERR = SIMPLE_ERROR; break; // message not understood
    }
  }
}

void RADIO_RECEIVE(){
  ERR = NO_ERROR;
  unsigned char msg[MSG_LEN];
  int nb = 0;
  while (hc12.available()) {
    unsigned char temp = hc12.read();
    if(nb<MSG_LEN){
      msg[nb] = temp;
      nb += 1;
    }
  }

  if((nb<MSG_LEN) & (nb>0)){ ERR = SIMPLE_ERROR; } // if too few data, raise an error
  if(nb==MSG_LEN){ process_message(msg); }
}

void RADIO_EMIT(byte target, byte value){
  char msg[MSG_LEN];
  msg[0] = target;
  msg[1] = value;
  msg[2] = msg[0] ^ msg[1];
  hc12.write(msg);
}
