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

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
#define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false

#define PID_Wx_KP  0.0 //250.0
#define PID_Wx_KI  0.0  //500.0
#define PID_Wx_KD  0.0  //2.5
#define PID_Wx_INTEGRATION_LIMIT    33.3

#define PID_Wy_KP  0.0 //250.0
#define PID_Wy_KI  0.0  //500.0
#define PID_Wy_KD  0.0  //2.5
#define PID_Wy_INTEGRATION_LIMIT   33.3

#define PID_Wz_KP  0.0 //120.0
#define PID_Wz_KI  0.0  //16.7
#define PID_Wz_KD  0.0  //0.0
#define PID_Wz_INTEGRATION_LIMIT     166.7


static float g_vehicleMass = 0.5; //0.027

// Logging variables
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float desiredWx;
static float stateWx;
static int16_t WxOutput;

static float desiredWy;
static float stateWy;
static int16_t WyOutput;

static float desiredWz;
static float stateWz;
static int16_t WzOutput;

static float dt;

static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

void controllerrpytReset(void)
{

}

PidObject pidWx;
PidObject pidWy;
PidObject pidWz;

void controllerrpytInit(void)
{

  pidInit(&pidWx,  0, PID_Wx_KP,  PID_Wx_KI,  PID_Wx_KD,
      ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pidInit(&pidWy, 0, PID_Wy_KP, PID_Wy_KI, PID_Wy_KD,
      ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pidInit(&pidWz,   0, PID_Wz_KP,   PID_Wz_KI,   PID_Wz_KD,
      ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

  pidSetIntegralLimit(&pidWx, PID_Wx_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidWy, PID_Wy_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidWz, PID_Wz_INTEGRATION_LIMIT);
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
  struct vec M_p;                     // Moment [pwm]
  struct vec4 trpy_g;                 // moter [gram]
  struct vec4 Moter_g;                // moter [gram]
  struct vec4 Moter_p;                // moter [gram]
  struct mat44 CT_rpyt2g=Ctrl_m(g_vehicleMass);
  // float dt;
  dt = (float)(1.0f/POSITION_RATE);
  // kato: RATE_DO_EXECUTE is in stabilizer_types.h and ATTITUDE_RATE is 500 Hz
  if (!RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    return;
  }
// --- calculate Rollrate Pitchrate error (P, I, D) ---

  desiredWx = radians(setpoint->attitudeRate.roll);
  desiredWy = radians(setpoint->attitudeRate.pitch);
  desiredWz = radians(setpoint->attitudeRate.yaw);

  stateWx = radians(sensors->gyro.x);
  stateWy = -radians(sensors->gyro.y);
  stateWz = radians(sensors->gyro.z);
  
  pidSetDesired(&pidWx, desiredWx);
  WxOutput = saturateSignedInt16(pidUpdate(&pidWx, stateWx, true));

  pidSetDesired(&pidWy, desiredWy);
  WyOutput = saturateSignedInt16(pidUpdate(&pidWy, stateWy, true));

  pidSetDesired(&pidWz, desiredWz);
  WzOutput = saturateSignedInt16(pidUpdate(&pidWz, stateWz, true));

// --- thrust roll pitch yaw input vector [gram] ---

  // target_thrust.z = - 25.0f * thrustScale * Vel_error.z - 0.0f * z_error_i + 36000.0f;
  trpy_g.x = setpoint->acceleration.z;
  trpy_g.y = WxOutput;
  trpy_g.z = WyOutput;
  trpy_g.w = WzOutput;

// --- change trpy[gram] to Moter[gram]
  Moter_g = mvmul4(CT_rpyt2g, trpy_g);

// --- change gram to pwm
  
  if (Moter_g.x >= 0) {Moter_p.x = sqrt((double)Moter_g.x*8.309953163553529e-7+8.77420076969215e-6)*2.406752433662037e+6-4.951128620134714e+3;}
  else {Moter_p.x = -(sqrt((double)-Moter_g.x*8.309953163553529e-7+8.77420076969215e-6)*2.406752433662037e+6-4.951128620134714e+3);}
  
  if (Moter_g.y >= 0) {Moter_p.y = sqrt((double)Moter_g.y*8.763965396891775e-7+1.033956593757182e-5)*2.282071995297151e+6-5.020028004430765e+3;}
  else {Moter_p.y = -(sqrt((double)-Moter_g.y*8.763965396891775e-7+1.033956593757182e-5)*2.282071995297151e+6-5.020028004430765e+3);}

  if (Moter_g.z >= 0) {Moter_p.z = sqrt((double)Moter_g.z*8.464410710967024e-7+1.206468234853723e-5)*2.362834305061159e+6-5.75446231128539e+3;}
  else{Moter_p.z = -(sqrt((double)-Moter_g.z*8.464410710967024e-7+1.206468234853723e-5)*2.362834305061159e+6-5.75446231128539e+3);}
  
  if (Moter_g.w >= 0) {Moter_p.w = sqrt((double)Moter_g.w*8.464410710967024e-7+1.206468234853723e-5)*2.362834305061159e+6-5.75446231128539e+3;}
  else{Moter_p.w = -(sqrt((double)-Moter_g.w*8.464410710967024e-7+1.206468234853723e-5)*2.362834305061159e+6-5.75446231128539e+3);}

  //calcurate input thrust and moment
  target_thrust.z = Moter_p.x + Moter_p.y + Moter_p.z + Moter_p.w;
  M_p.x = -Moter_p.x - Moter_p.y + Moter_p.z + Moter_p.w;
  M_p.y = -Moter_p.x + Moter_p.y + Moter_p.z - Moter_p.w;
  M_p.z = -Moter_p.x + Moter_p.y - Moter_p.z + Moter_p.w;

// determine Thrust input 
  if (setpoint->mode.z != modeDisable  && trpy_g.x != 0){
    // control->thrust = massThrust * target_thrust.z;
    control->thrust = target_thrust.z; 
  } else{

    control->flag = false;
    control->thrust = (float)0.0;
    M_p.z = (float)0.0;
    M_p.x = (float)0.0;
    M_p.y = (float)0.0;

    control->m1 = (float)0.0;
    control->m2 = (float)0.0;
    control->m3 = (float)0.0;
    control->m4 = (float)0.0;
  }

  // M.x = - (float)10000 * eRM.m[2][1];
// --- Logging param and saturation input ---
  cmd_thrust = control->thrust;

  control->roll = clamp(M_p.x, -32000, 32000);
  control->pitch = clamp(M_p.y, -32000, 32000);
  control->yaw = clamp(M_p.z, -32000, 32000);

  control->flag = true;
  control->m1 = Moter_p.x;
  control->m2 = Moter_p.y;
  control->m3 = Moter_p.z;
  control->m4 = Moter_p.w;

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