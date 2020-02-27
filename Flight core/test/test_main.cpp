#if defined(ARDUINO) && defined(UNIT_TEST)

#include <Arduino.h>
#include "unity.h"

#include "../src/Radio_HC12.h"
#include "../src/Sensor_MPU6050_tockn.h"

int ERR = 1;
byte MOTOR_SPEED_AVG = 0;
byte MOTOR_OFFSET[4] = {0,0,0,0};
float STATE[9];

void testHC12() {
  Serial.println(F(">>> Testing connection with radio HC12"));
  delay(100);
  RADIO_INIT();
  TEST_ASSERT_EQUAL(0,ERR);
}

void testMPU6050() {
  Serial.println(F(">>> Testing connection with MPU6050"));
  delay(100);
  INIT_MPU6050();
  TEST_ASSERT_EQUAL(0,ERR);
}

void setup() {
  Serial.begin(115200);
  UNITY_BEGIN();
  RUN_TEST(testHC12);
  RUN_TEST(testMPU6050);
  UNITY_END();
}

void loop() {
  // nothing to be done here.
  UNITY_END();
}

#endif
