#include "MPU6050_tockn.h"
#include "Arduino.h"

#define GYRO_LSB_2_DEGSEC  65.5     // [bit/(°/s)]
#define ACC_LSB_2_G        16384.0  // [bit/gravity]
#define RAD_2_DEG          57.29578 // [°/rad]
#define GYRO_OFFSET_NB_MES 3000     //
#define TEMP_LSB_2_DEGREE  340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET    12412.0  //

MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  accCoef = 0.02f;
  gyroCoef = 0.98f;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
  wire = &w;
  accCoef = aC;
  gyroCoef = gC;
}

void MPU6050::begin(){
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  this->update();
  angleGyroX = 0;
  angleGyroY = 0;
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis();
}

void MPU6050::writeMPU6050(byte reg, byte data){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg) {
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(MPU6050_ADDR, 1);
  byte data =  wire->read();
  return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::calcGyroOffsets(bool console, uint16_t delayBefore, uint16_t delayAfter){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

  for(int i = 0; i < GYRO_OFFSET_NB_MES; i++){
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x43);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 6);

    rx  = wire->read() << 8;
	rx |= wire->read();
    ry  = wire->read() << 8;
	ry |= wire->read();
    rz  = wire->read() << 8;
	rz |= wire->read();

    x += ((float)rx) / GYRO_LSB_2_DEGSEC;
    y += ((float)ry) / GYRO_LSB_2_DEGSEC;
    z += ((float)rz) / GYRO_LSB_2_DEGSEC;
  }
  gyroXoffset = x / GYRO_OFFSET_NB_MES;
  gyroYoffset = y / GYRO_OFFSET_NB_MES;
  gyroZoffset = z / GYRO_OFFSET_NB_MES;
}

void MPU6050::update(){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX   = wire->read() << 8;
  rawAccX  |= wire->read();
  rawAccY   = wire->read() << 8;
  rawAccY  |= wire->read();
  rawAccZ   = wire->read() << 8;
  rawAccZ  |= wire->read();
  rawTemp   = wire->read() << 8;
  rawTemp  |= wire->read();
  rawGyroX  = wire->read() << 8;
  rawGyroX |= wire->read();
  rawGyroY  = wire->read() << 8;
  rawGyroY |= wire->read();
  rawGyroZ  = wire->read() << 8;
  rawGyroZ |= wire->read();

  temp = (rawTemp + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;

  accX = ((float)rawAccX) / ACC_LSB_2_G;
  accY = ((float)rawAccY) / ACC_LSB_2_G;
  accZ = ((float)rawAccZ) / ACC_LSB_2_G;

  angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * RAD_2_DEG;
  angleAccY = - atan2(accX, sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG;

  gyroX = ((float)rawGyroX) / GYRO_LSB_2_DEGSEC;
  gyroY = ((float)rawGyroY) / GYRO_LSB_2_DEGSEC;
  gyroZ = ((float)rawGyroZ) / GYRO_LSB_2_DEGSEC;

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  unsigned long Tnew = millis();
  interval = (Tnew - preInterval) * 0.001;
  preInterval = Tnew;

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

  angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

}
