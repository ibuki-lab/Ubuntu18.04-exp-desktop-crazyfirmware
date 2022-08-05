/*
The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This controller is based on the following publication and controller_mellinger.c:
This controller is assumed to have been used after take off completed. 

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.


*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "position_controller.h"
#include "controller_rpyt.h"
#include "physicalConstants.h"
#include "pid.h"

static float g_vehicleMass = CF_MASS; //0.027
// static float massThrust = 132000;

// Logging variables
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float x_error_i;
static float y_error_i;
static float z_error_i;

static float desiredRoll;
static float stateRoll;

static float desiredPitch;
static float statePitch;

static float desiredRollrate;
static float stateRollratedeg;

static float desiredPitchrate;
static float statePitchratedeg;

static const float thrustScale = 1000.0f;

static float Roll_error;
// static float Roll_error_pre;
static float Roll_i_error;
static float Roll_d_error;


static float Pitch_error;
// static float Pitch_error_pre;
static float Pitch_i_error;
static float Pitch_d_error;

static float Rollrate_error;
static float Rollrate_error_pre;
static float Rollrate_i_error;
static float Rollrate_d_error;

static float Pitchrate_error;
static float Pitchrate_error_pre;
static float Pitchrate_i_error;
static float Pitchrate_d_error;

static float dt;

// static float kR_xy = 60000; // P
// static float kw_xy = 10000; // D


// static float prev_roll = 0.0;
// static float prev_pitch = 0.0;



void controllerrpytReset(void)
{
  x_error_i = 0;
  y_error_i = 0;
  z_error_i = 0;
 
  Rollrate_error_pre = 0;
  Pitchrate_error_pre = 0;

  Roll_i_error = 0;
  Pitch_i_error = 0;
  
  Roll_d_error = 0;
  Pitch_d_error = 0;
  
  Rollrate_i_error = 0;
  Pitchrate_i_error = 0;

  Rollrate_d_error = 0;
  Pitchrate_d_error = 0;
}

void controllerrpytInit(void)
{
  controllerrpytReset();
}

bool controllerrpytTest(void)
{
  return true;
}

void controllerrpyt(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{                           // kato:
  struct vec target_thrust;         // nominal thrust
  struct vec M;                     // Moment
  struct vec Pos_error;
  struct vec Vel_error;
  struct mat33 Ryaw;
  // float dt;
  dt = (float)(1.0f/POSITION_RATE);
  // kato: RATE_DO_EXECUTE is in stabilizer_types.h and ATTITUDE_RATE is 500 Hz
  if (!RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    return;
  }

//  Get Rotation Matrix
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  // struct mat33 R = quat2rotmat(q);
  struct vec rpy = quat2rpy(q);
  float yaw = rpy.z;

  // Rotation matrics about Yaw
  Ryaw.m[0][0] = cosf(yaw);
  Ryaw.m[0][1] = -sinf(yaw);
  Ryaw.m[0][2] = (float)0.0;
  Ryaw.m[1][0] = sinf(yaw);
  Ryaw.m[1][1] = cosf(yaw);
  Ryaw.m[1][2] = (float)0.0;
  Ryaw.m[2][0] = (float)0.0;
  Ryaw.m[2][1] = (float)0.0;
  Ryaw.m[2][2] = (float)1.0;

  // struct mat33 R_transpose = mtranspose(R);
  struct mat33 Ryaw_transpose = mtranspose(Ryaw);

// --- calculate Position Error and integrate Postion Error ---
  struct vec desiredPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);

  Pos_error = vsub(statePos, desiredPos);

  x_error_i += x_error_i * dt;
  y_error_i += y_error_i * dt;
  z_error_i += z_error_i * dt;
  
// --- calculate Velosity Error ---
  struct vec desiredVel = mvmul(Ryaw_transpose, mkvec(-2.0f * Pos_error.x, -2.0f * Pos_error.y, -2.0f * Pos_error.z));
  struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  
  Vel_error = vsub(stateVel, desiredVel);

// --- calculate Roll Pitch Error (P, I) ---
  desiredRoll = 25.0f * Vel_error.y + 0.0f * y_error_i;
  desiredPitch = 25.0f * Vel_error.x+ 0.0f * x_error_i;
  
  stateRoll = state->attitude.roll;
  statePitch = state->attitude.pitch;

  // P
  Roll_error = stateRoll - desiredRoll;
  Pitch_error = statePitch - desiredPitch;

  // I
  Roll_i_error += Roll_error * dt;
  Pitch_i_error += Pitch_error * dt;
  Roll_i_error = clamp(Roll_i_error, -20.0f, 20.0f);
  Pitch_i_error = clamp(Pitch_i_error, -20.0f, 20.0f);

  // D
  // Roll_d_error = (Roll_error - Roll_error_pre) / dt;
  // Pitch_d_error = (Pitch_error - Pitch_error_pre)/dt;
  // Roll_error_pre = Roll_error;
  // Pitch_error_pre = Pitch_error;

// --- calculate Rollrate Pitchrate error (P, I, D) ---

  desiredRollrate = -6.0f * Roll_error -3.0f * Roll_i_error;
  desiredPitchrate = -6.0f * Pitch_error  -3.0f * Pitch_i_error;

  stateRollratedeg = sensors->gyro.x;
  statePitchratedeg = -sensors->gyro.y;
  
  // P
  Rollrate_error = stateRollratedeg - desiredRoll;
  Pitchrate_error = statePitchratedeg - desiredPitch;
  
  // I
  Rollrate_i_error += Rollrate_error * dt;
  Pitchrate_i_error += Pitchrate_i_error * dt;
  Roll_i_error = clamp(Roll_i_error, -33.0f, 33.0f);
  Pitch_i_error = clamp(Pitch_i_error, -33.0f, 33.0f);

  // D
  Rollrate_d_error = (Rollrate_error - Rollrate_error_pre) / dt;
  Pitchrate_d_error = (Pitchrate_error - Pitchrate_error_pre)/dt;
  Rollrate_error_pre = Rollrate_error;
  Pitchrate_error_pre = Pitchrate_error;

// --- input Thrust ---

  target_thrust.z = - 25.0f * thrustScale * Vel_error.z - 0.0f * z_error_i + 36000.0f;

// --- input Moment ---

  M.x = -250.0f * Rollrate_error  - 100.0f * Rollrate_i_error - 2.5f * Rollrate_d_error;
  M.y = -250.0f * Pitchrate_error - 100.0f * Pitchrate_i_error - 2.5f * Pitchrate_d_error;
  M.z = 0;

// determine Thrust input 
  if (setpoint->mode.z != modeDisable){
    // control->thrust = massThrust * target_thrust.z;
    control->thrust = target_thrust.z;
  } else{
    control->thrust = (float)0 * target_thrust.z;
    M.z = (float)0.0;
    M.x = (float)0.0;
    M.y = (float)0.0;
  }

  // M.x = - (float)10000 * eRM.m[2][1];
// --- Logging param and saturation input ---
  cmd_thrust = control->thrust;

  control->roll = clamp(M.x, -32000, 32000);
  control->pitch = clamp(M.y, -32000, 32000);
  control->yaw = clamp(-M.z, -32000, 32000);

  cmd_roll = control->roll;
  cmd_pitch = control->pitch;
  cmd_yaw = control->yaw;

}

PARAM_GROUP_START(ctrlrpyt)
PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)
// PARAM_ADD(PARAM_FLOAT, massThrust, &massThrust)
PARAM_GROUP_STOP(ctrlrpyt)

LOG_GROUP_START(ctrlrpyt)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_GROUP_STOP(ctrlrpyt)