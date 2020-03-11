/*
 * PID correctors transforming the measured state
 * (angles and linear acceleration) into motor voltage commands.
 *
 * TODO: Include measured voltage BATTERY_VOLT somewhere...
 *       and the voltage-lift experimental curves
 */
#include <Arduino.h>
#include "drone_define.h"

extern float STATE[9];
extern float COMMAND[6];
extern byte MOTOR_SPEED[4];
extern byte MOTOR_SPEED_AVG;
extern byte MOTOR_OFFSET[4];
extern int DRONE_STATUS;
extern int ERR;

/*** CLASS DEFINITION ***/

class PIDcorrector{
  public:
    float Kp, Ki, Kd = 0.0;
    float sum_err = 0.0; // public, but better don't modify this inloop

  PIDcorrector(float kp, float ki, float kd){
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
  }

  float compute(float command_pos, float state_pos, float state_speed, float dt){
    float err = command_pos - state_pos;
    if(DRONE_STATUS==DRONE_STATUS_FLIGHT){
      this->sum_err += err*dt;
    }else{
      this->sum_err = 0.0; // no integration if drone is on the ground
    }
    err = this->Kp * err + this->Ki * this->sum_err - this->Kd * state_speed;
    return err;
  }
};

/*** CODE ***/

PIDcorrector Pitch_Corr(C_PR_KP, C_PR_KI, C_PR_KD);
PIDcorrector Roll_Corr(C_PR_KP, C_PR_KI, C_PR_KD);
PIDcorrector Yaw_Corr(C_YAW_KP, C_YAW_KI, C_YAW_KD);
PIDcorrector X_Corr(C_XY_KP, C_XY_KI, 0.0);
PIDcorrector Y_Corr(C_XY_KP, C_XY_KI, 0.0);

void UPDATE_CORRECTION(float dt){
  ERR = NO_ERROR;

  // Compute angles to correct for linear acceleration and put them into COMMAND angles
  #if ENABLE_XY_PID
    COMMAND[1] = X_Corr.compute(0.0,-STATE[6],0.0,dt);
    COMMAND[2] = Y_Corr.compute(0.0,-STATE[7],0.0,dt);
  #endif

  // Compute motor command from the MPU6050 data
  float Kyaw   = 0.0;
  #if ENABLE_YAW_PID
    Kyaw = Yaw_Corr.compute(COMMAND[0],STATE[0],STATE[3],dt); // check +--+ or -++- signs
  #endif

  float Kpitch = Pitch_Corr.compute(COMMAND[1],STATE[1],STATE[4],dt);
  float Kroll  =  Roll_Corr.compute(COMMAND[2],STATE[2],STATE[5],dt);

  if(MOTOR_SPEED_AVG>MOT_SAT_AVG){ ERR = SIMPLE_ERROR; }
  MOTOR_SPEED_AVG = min(MOTOR_SPEED_AVG,MOT_SAT_AVG); // saturate average

  // The +- sign depends on the motor position on the drone
  MOTOR_SPEED[0] = min(max(MOTOR_SPEED_AVG + MOTOR_OFFSET[0] + Kroll + Kpitch - Kyaw,0),MOT_SAT_MAX);
  MOTOR_SPEED[1] = min(max(MOTOR_SPEED_AVG + MOTOR_OFFSET[1] - Kroll + Kpitch + Kyaw,0),MOT_SAT_MAX);
  MOTOR_SPEED[2] = min(max(MOTOR_SPEED_AVG + MOTOR_OFFSET[2] + Kroll - Kpitch + Kyaw,0),MOT_SAT_MAX);
  MOTOR_SPEED[3] = min(max(MOTOR_SPEED_AVG + MOTOR_OFFSET[3] - Kroll - Kpitch - Kyaw,0),MOT_SAT_MAX);
}
