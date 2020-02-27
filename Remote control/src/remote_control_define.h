#ifndef REMOTE_CONTROL_DEFINE_H
#define REMOTE_CONTROL_DEFINE_H

#define RADIO_PIN_RX 2
#define RADIO_PIN_TX 3
#define RADIO_BAUD   9600

#define POTEN_PIN A4 // pin connected to potentiometer
#define POTEN_MIN 15 // min value of analogRead on the potentiometer
#define POTEN_MAX 1000 // max value of analogRead on the potentiometer
#define SIGMA     2 // threshold to accept a new value (noise filtering)
#define MSG_DELAY 100 // minimal delay between two consecutive messages


#endif
