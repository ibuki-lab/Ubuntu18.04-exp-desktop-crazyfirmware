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
#include "controller_thrust.h"
#include "physicalConstants.h"
#include "pid.h"

static float g_vehicleMass = CF_MASS; //0.027
// static float massThrust = 132000;

// Logging variables
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float dt;
static float stateRollratedeg;
static float statePitchratedeg;

void controllerthrustReset(void)
{

}

void controllerthrustInit(void)
{
  controllerthrustReset();
}

bool controllerthrustTest(void)
{
  return true;
}

void controllerthrust(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{                           // kato:
  struct vec target_thrust;         // nominal thrust
  struct vec M;                     // Moment

  // float dt;
  // kato: RATE_DO_EXECUTE is in stabilizer_types.h and ATTITUDE_RATE is 500 Hz
  dt = (float)(1.0f/POSITION_RATE);
  if (!RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    return;
  }

// get thrust input 
  target_thrust.x = setpoint->acceleration.x;
  target_thrust.y = setpoint->acceleration.y;
  target_thrust.z = setpoint->acceleration.z;

//  Get Rotation Matrix
//   struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
//   struct vec rpy = quat2rpy(q);
  stateRollratedeg = sensors->gyro.x;
  statePitchratedeg = -sensors->gyro.y;

// --- input Moment ---
  M.x = (float)0;
  M.y = (float)0;
  M.z = (float)0;

// determine Thrust input 
  if (setpoint->mode.z != modeDisable){
    // control->thrust = massThrust * target_thrust.z;
    control->thrust = target_thrust.z * (float)10000;
  } else{
    control->thrust = (float)0 * target_thrust.z;
    M.z = (float)0.0;
    M.x = (float)0.0;
    M.y = (float)0.0;
  }

  // M.x = - (float)10000 * eRM.m[2][1];
// --- Logging param and saturation input ---
  cmd_thrust = clamp(control->thrust, 0, 35000);

  control->roll = clamp(M.x, -32000, 32000);
  control->pitch = clamp(M.y, -32000, 32000);
  control->yaw = clamp(-M.z, -32000, 32000);

  cmd_roll = control->roll;
  cmd_pitch = control->pitch;
  cmd_yaw = control->yaw;

}

PARAM_GROUP_START(ctrlthrust)
PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)
// PARAM_ADD(PARAM_FLOAT, massThrust, &massThrust)
PARAM_GROUP_STOP(ctrlthrust)

LOG_GROUP_START(ctrlthrust)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_GROUP_STOP(ctrlthrust)