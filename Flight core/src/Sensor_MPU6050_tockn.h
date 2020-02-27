/*
 * Get data from MPU6050 using TOCKN library
 */

#include <Arduino.h>

#include <MPU6050_tockn.h>
#include "drone_define.h"

extern float STATE[9];
extern int ERR;

MPU6050 mpu6050(Wire);

void INIT_MPU6050(){
  ERR = (int)mpu6050.begin();
  mpu6050.calcGyroOffsets();
}

void UPDATE_MPU6050(){
  ERR = (int)mpu6050.update();
  STATE[0] =  mpu6050.getAngleZ();
  STATE[1] = -mpu6050.getAngleY();
  STATE[2] =  mpu6050.getAngleX();
  STATE[3] =  mpu6050.getGyroZ();
  STATE[4] = -mpu6050.getGyroY();
  STATE[5] =  mpu6050.getGyroX();
  STATE[6] =  mpu6050.getAccX();
  STATE[7] =  mpu6050.getAccY();
  STATE[8] =  mpu6050.getAccZ();
  // Linear acceleration [units of gravity] from drone frame to reference ground frame
  STATE[6] = STATE[6]*cos(STATE[1]*DEG2RAD) + STATE[8]*sin(STATE[1]*DEG2RAD);
  STATE[7] = STATE[7]*cos(STATE[2]*DEG2RAD) - STATE[8]*sin(STATE[2]*DEG2RAD);
  // Correct for acceleration drift by thresholding
  STATE[6] *= (abs(STATE[6])>ACC_THRESH);
  STATE[7] *= (abs(STATE[7])>ACC_THRESH);
}
