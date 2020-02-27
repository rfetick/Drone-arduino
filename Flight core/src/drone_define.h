#ifndef DRONE_DEFINE_H
#define DRONE_DEFINE_H

// SERIAL PRINT
#define BAUDRATE 250000
#define VERBOSE  true

// DRONE STATUS
#define DRONE_STATUS_LANDED 0
#define DRONE_STATUS_FLIGHT 1

// PID CORRECTORS PITCH ROLL
#define C_PR_KP 2.0 // [bit/deg]
#define C_PR_KI 1e-9 // [bit/deg/us]
#define C_PR_KD 3e-1 // [bit/(deg/s)]

// PID CORRECTORS YAW
#define C_YAW_KP 0.5 // [bit/deg]
#define C_YAW_KI 1e-10 // [bit/deg/us]
#define C_YAW_KD 1e-1 // [bit/(deg/s)]

// PID CORRECTORS LINEAR ACCELERATION
#define C_XY_KP 4.0 // [deg/gravity]
#define C_XY_KI 1e-8 // [deg/gravity/us]

// MOTOR SATURATION LEVEL
#define MOT_SAT_MAX 200 // motor command is between 0 and 255

// HOUSEKEEPING
#define PIN_V_CHECK        A6 // pin for the measurement of the battery voltage
#define HK_BIT_2_VOLT      0.0274 // (R_high+R_low)/R_low*5.0/1024 = 0.02929 [V/bit]    with R_high=10kOhm and R_low=2kOhm
#define BLINK_TIME         300 // on error blinking time [ms]
#define FILTER_VOLT_ACCEPT 0.03 // acceptance ratio of a new voltage measure (between 0 and 1)
#define MIN_VOLT           7.0 // trigger an error if voltage is too low
#define VOLT_LEVEL_FLIGHT  480 // motor bit level 60 at 8V that defines the DRONE_STATUS_FLIGHT

// RADIO
#define RADIO_PIN_RX A1 // radio reception pin
#define RADIO_PIN_TX A2 // radio transmission pin
#define RADIO_BAUD   9600 // must be similar to remote control radio baud
#define MSG_LEN      3 // [target,value,check]

// MPU6050
#define DEG2RAD    0.01745329252 // [rad/°]
#define ACC_THRESH 0.03 // threshold on acceleration to remove eventual accelerometer drift


#endif
