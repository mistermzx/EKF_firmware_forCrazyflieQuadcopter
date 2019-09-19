/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "commander.h"
#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

struct pidInit_s {
  float kp;
  float ki;
  float kd;
};

struct deadband_input_s {
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
} deadband_input ;


struct pidAxis_s {
  PidObject pid;

  struct pidInit_s init;
  mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidVX;
  struct pidAxis_s pidVY;
  struct pidAxis_s pidVZ;

  struct pidAxis_s pidX;
  struct pidAxis_s pidY;
  struct pidAxis_s pidZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  float    pitchBase; // approximate pitch needed when in perfect hover.
  float    rollBase; // approximate roll needed when in perfect hover.

  uint16_t thrustMin;  // Minimum thrust value to output
};

// Maximum roll/pitch angle permited
static float rpLimit  = 10;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xyVelMax = 10.0f;
static float zVelMax  = 10.0f;
//static float velMaxOverhead = 1.10f;
static float poserror[3];
static float raw[3];
static float raw_data[3];

static const float thrustScale = 1000.0f;

#define DT (float)(1.0f/POSITION_RATE)
#define POSITIONx_LPF_CUTOFF_FREQ 5.0f
#define POSITIONy_LPF_CUTOFF_FREQ 5.0f
#define POSITIONz_LPF_CUTOFF_FREQ 5.0f
#define POSITION_LPF_ENABLE true

#ifndef UNIT_TEST
static struct this_s this = {
  .pidVX = {
    .init = {
      .kp = 0.0f,
      .ki = 0.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
  },

  .pidVY = {
    .init = {
      .kp = 0.0f,
      .ki = 0.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
  },

  .pidVZ = {
    .init = {
      .kp = 0.0f,
      .ki = 0.0f,
      .kd = 0.0f,
    },
    .pid.dt = DT,
  },

  .pidX = {
    .init = {
      .kp = 10.0f,
      .ki = 0.2f,
      .kd = 20.0f,//12.0f,
    },
    .pid.dt = DT,
  },

  .pidY = {
    .init = {
      .kp = 12.0f,
      .ki = 0.2f,
      .kd = 12.0f,
    },
    .pid.dt = DT,
  },

  .pidZ = {
    .init = {
      .kp = 20.0f,
      .ki = 1.5f,
      .kd = 18.0f,
    },
    .pid.dt = DT,
  },

  .thrustBase = 38000,
  .pitchBase  = -0.5f, //-0.3f,
  .rollBase   = 1.0f,
  .thrustMin  = 30000,
};
#endif

void positionControllerInit()
{
  pidInit(&this.pidX.pid, this.pidX.setpoint, this.pidX.init.kp, this.pidX.init.ki, this.pidX.init.kd,
      this.pidX.pid.dt, POSITION_RATE, POSITIONx_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidY.pid, this.pidY.setpoint, this.pidY.init.kp, this.pidY.init.ki, this.pidY.init.kd,
      this.pidY.pid.dt, POSITION_RATE, POSITIONy_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp, this.pidZ.init.ki, this.pidZ.init.kd,
      this.pidZ.pid.dt, POSITION_RATE, POSITIONz_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);

  pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.init.kp, this.pidVX.init.ki, this.pidVX.init.kd,
      this.pidVX.pid.dt, POSITION_RATE, POSITIONx_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.init.kp, this.pidVY.init.ki, this.pidVY.init.kd,
      this.pidVY.pid.dt, POSITION_RATE, POSITIONy_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp, this.pidVZ.init.ki, this.pidVZ.init.kd,
      this.pidVZ.pid.dt, POSITION_RATE, POSITIONz_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);

}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float deriv) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate_mod(&axis->pid, input, true, deriv);
  //return pidUpdate(&axis->pid, input, true);

}

void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{
	  this.pidX.pid.outputLimit = rpLimit * rpLimitOverhead;
	  this.pidY.pid.outputLimit = rpLimit * rpLimitOverhead;
	  // The ROS landing detector will prematurely trip if
	  // this value is below 0.5
	  this.pidZ.pid.outputLimit = (UINT16_MAX / 6 / thrustScale);

	  // X, Y
	  // Eigener Deadband

	  float deadband_error = deadband(setpoint->position.x-state->position.x, 0.001);
	  deadband_input.x = setpoint->position.x - deadband_error;

	  deadband_error = deadband(setpoint->position.y-state->position.y, 0.001);
	  deadband_input.y = setpoint->position.y - deadband_error;

	  deadband_error = deadband(setpoint->position.z-state->position.z, 0.001);
	  deadband_input.z = setpoint->position.z - deadband_error;


	  poserror[0]=setpoint->position.x-deadband_input.x;
	  poserror[1]=setpoint->position.y-deadband_input.y;
	  poserror[2]=setpoint->position.z-deadband_input.z;

	  deadband_input.vx =deadband(state->velocity.x, 0.04);
	  deadband_input.vy =deadband(state->velocity.y, 0.03);
	  deadband_input.vz =deadband(state->velocity.z, 0.03);

	  /*
	  if (abs(poserror[0]) < 0.05f && poserror[0]!=0)
	  {
		  this.pidVX.pid.kp = min(1.5f,(0.05f/abs(poserror[0])))*this.pidVX.init.kp;
	  }
	  else
	  {
		  this.pidVX.pid.kp =this.pidVX.init.kp;
	  }


	  if (abs(poserror[1]) < 0.05f && poserror[1]!=0)
	  {
		  this.pidVY.pid.kp = min(1.5f,(0.05f/abs(poserror[1])))*this.pidVY.init.kp;

	  }
	  else
	  {
		  this.pidVY.pid.kp =this.pidVY.init.kp;
	  }


	  if (abs(poserror[2]) < 0.05f && poserror[2]!=0)
	  {
		  this.pidVZ.pid.kp = min(1.5f,(0.05f/abs(poserror[2])))*this.pidVZ.init.kp;

	  }
	  else
	  {
		  this.pidVZ.pid.kp =this.pidVZ.init.kp;
	  }*/



	  float pitchRaw = runPid(deadband_input.x, &this.pidX, setpoint->position.x, deadband_input.vx);
	  float rollRaw = runPid(deadband_input.y, &this.pidY, setpoint->position.y, deadband_input.vy);
	  raw[0]=this.pidX.pid.outP+this.pidX.pid.outD+this.pidX.pid.outI;
	  raw[1]=this.pidY.pid.outP+this.pidY.pid.outD+this.pidY.pid.outI;

	  raw_data[0]=pitchRaw;
	  raw_data[1]=rollRaw;

	  float yawRad = state->attitude.yaw * (float)M_PI / 180;
	  //Für Testzwecke
	  attitude->roll = -(rollRaw  * cosf(yawRad)) - (pitchRaw * sinf(yawRad)) + this.rollBase;
	  attitude->pitch  = -(pitchRaw * cosf(yawRad)) + (rollRaw  * sinf(yawRad)) + this.pitchBase;

	  attitude->roll  = constrain_with_AntiWindUp(attitude->roll,  -rpLimit, rpLimit, &this.pidY.pid);
	  attitude->pitch  = constrain_with_AntiWindUp(attitude->pitch,  -rpLimit, rpLimit, &this.pidX.pid);

	  // Thrust
	  float thrustRaw = runPid(deadband_input.z, &this.pidZ, setpoint->position.z, deadband_input.vz);
	  raw_data[2]=thrustRaw;
	  // Scale the thrust and add feed forward term
	  *thrust = thrustRaw*thrustScale + this.thrustBase;
	  // Check for minimum thrust
	  if (*thrust < this.thrustMin)
	  {
	    *thrust = this.thrustMin;
	  }

}

void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state)
{

}

void positionControllerResetAllPID()
{
  pidReset(&this.pidX.pid);
  pidReset(&this.pidY.pid);
  pidReset(&this.pidZ.pid);
  pidReset(&this.pidVX.pid);
  pidReset(&this.pidVY.pid);
  pidReset(&this.pidVZ.pid);
}

float constrain_with_AntiWindUp(float value, const float minVal, const float maxVal, PidObject* pid)
{
  float AntiWindUp = 0;
  if (value > maxVal)
  {
	  AntiWindUp = maxVal - value;
	  pid->integ += AntiWindUp * pid->dt;
  }
  else if (value < minVal)
  {
	  AntiWindUp = minVal - value;
	  pid->integ += AntiWindUp * pid->dt;
  }

  return min(maxVal, max(minVal,value));
}

LOG_GROUP_START(posCtl)

LOG_ADD(LOG_FLOAT, targetVX, &this.pidVX.pid.desired)
LOG_ADD(LOG_FLOAT, targetVY, &this.pidVY.pid.desired)
LOG_ADD(LOG_FLOAT, targetVZ, &this.pidVZ.pid.desired)

LOG_ADD(LOG_FLOAT, targetX, &this.pidX.pid.desired)
LOG_ADD(LOG_FLOAT, targetY, &this.pidY.pid.desired)
LOG_ADD(LOG_FLOAT, targetZ, &this.pidZ.pid.desired)

LOG_ADD(LOG_FLOAT, Xp, &this.pidX.pid.outP)
LOG_ADD(LOG_FLOAT, Xi, &this.pidX.pid.outI)
LOG_ADD(LOG_FLOAT, Xd, &this.pidX.pid.outD)

LOG_ADD(LOG_FLOAT, Yp, &this.pidY.pid.outP)
LOG_ADD(LOG_FLOAT, Yi, &this.pidY.pid.outI)
LOG_ADD(LOG_FLOAT, Yd, &this.pidY.pid.outD)

LOG_ADD(LOG_FLOAT, Zp, &this.pidZ.pid.outP)
LOG_ADD(LOG_FLOAT, Zi, &this.pidZ.pid.outI)
LOG_ADD(LOG_FLOAT, Zd, &this.pidZ.pid.outD)

LOG_ADD(LOG_FLOAT, VXp, &this.pidVX.pid.outP)
LOG_ADD(LOG_FLOAT, VXi, &this.pidVX.pid.outI)
LOG_ADD(LOG_FLOAT, VXd, &this.pidVX.pid.outD)

LOG_ADD(LOG_FLOAT, VYp, &this.pidVY.pid.outP)
LOG_ADD(LOG_FLOAT, VYi, &this.pidVY.pid.outI)
LOG_ADD(LOG_FLOAT, VYd, &this.pidVY.pid.outD)

LOG_ADD(LOG_FLOAT, VZp, &this.pidVZ.pid.outP)
LOG_ADD(LOG_FLOAT, VZi, &this.pidVZ.pid.outI)
LOG_ADD(LOG_FLOAT, VZd, &this.pidVZ.pid.outD)

LOG_GROUP_STOP(posCtl)



LOG_GROUP_START(poserror)
LOG_ADD(LOG_FLOAT, pos_x, &poserror[0])
LOG_ADD(LOG_FLOAT, pos_y, &poserror[1])
LOG_ADD(LOG_FLOAT, pos_z, &poserror[2])
LOG_ADD(LOG_FLOAT, raw_x, &raw[0])
LOG_ADD(LOG_FLOAT, raw_y, &raw[1])
LOG_ADD(LOG_FLOAT, raw_z, &raw[2])
LOG_ADD(LOG_FLOAT, pitchraw,  &raw_data[0])
LOG_ADD(LOG_FLOAT, rollraw, &raw_data[1])
LOG_ADD(LOG_FLOAT, thrustraw,&raw_data[2])
LOG_GROUP_STOP(poserror)


PARAM_GROUP_START(velCtlPid)

PARAM_ADD(PARAM_FLOAT, vxKp, &this.pidVX.pid.kp)
PARAM_ADD(PARAM_FLOAT, vxKi, &this.pidVX.pid.ki)
PARAM_ADD(PARAM_FLOAT, vxKd, &this.pidVX.pid.kd)

PARAM_ADD(PARAM_FLOAT, vyKp, &this.pidVY.pid.kp)
PARAM_ADD(PARAM_FLOAT, vyKi, &this.pidVY.pid.ki)
PARAM_ADD(PARAM_FLOAT, vyKd, &this.pidVY.pid.kd)

PARAM_ADD(PARAM_FLOAT, vzKp, &this.pidVZ.pid.kp)
PARAM_ADD(PARAM_FLOAT, vzKi, &this.pidVZ.pid.ki)
PARAM_ADD(PARAM_FLOAT, vzKd, &this.pidVZ.pid.kd)

PARAM_GROUP_STOP(velCtlPid)

PARAM_GROUP_START(posCtlPid)

PARAM_ADD(PARAM_FLOAT, xKp, &this.pidX.pid.kp)
PARAM_ADD(PARAM_FLOAT, xKi, &this.pidX.pid.ki)
PARAM_ADD(PARAM_FLOAT, xKd, &this.pidX.pid.kd)

PARAM_ADD(PARAM_FLOAT, yKp, &this.pidY.pid.kp)
PARAM_ADD(PARAM_FLOAT, yKi, &this.pidY.pid.ki)
PARAM_ADD(PARAM_FLOAT, yKd, &this.pidY.pid.kd)

PARAM_ADD(PARAM_FLOAT, zKp, &this.pidZ.pid.kp)
PARAM_ADD(PARAM_FLOAT, zKi, &this.pidZ.pid.ki)
PARAM_ADD(PARAM_FLOAT, zKd, &this.pidZ.pid.kd)

PARAM_ADD(PARAM_UINT16, thrustBase,&this.thrustBase)
PARAM_ADD(PARAM_FLOAT,  pitchBase, &this.pitchBase)
PARAM_ADD(PARAM_FLOAT,  rollBase,  &this.rollBase)
PARAM_ADD(PARAM_UINT16, thrustMin, &this.thrustMin)

PARAM_ADD(PARAM_FLOAT, rpLimit,  &rpLimit)
PARAM_ADD(PARAM_FLOAT, xyVelMax, &xyVelMax)
PARAM_ADD(PARAM_FLOAT, zVelMax,  &zVelMax)

PARAM_GROUP_STOP(posCtlPid)
