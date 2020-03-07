#ifndef REMOTE_CONTROL_DEFINE_H
#define REMOTE_CONTROL_DEFINE_H

#define RADIO_PIN_RX 2
#define RADIO_PIN_TX 3
#define RADIO_BAUD   9600

#define BUTTON_PIN 9 // pin connected to stop button
#define POTEN_PIN  A1 // pin connected to potentiometer
#define POTEN_MIN  20 // min value of analogRead on the potentiometer
#define POTEN_MAX  1010 // max value of analogRead on the potentiometer
#define SIGMA      4 // threshold to accept a new value (noise filtering)
#define MSG_DELAY  60 // minimal delay between two consecutive messages


#endif
