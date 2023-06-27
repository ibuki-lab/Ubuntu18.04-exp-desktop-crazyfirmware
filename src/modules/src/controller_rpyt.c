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

#define PID_Wx_KP  2.1 //50
#define PID_Wx_KI  0.01  //500.0
#define PID_Wx_KD  0.0  //2.5
#define PID_Wx_INTEGRATION_LIMIT    1.5

#define PID_Wy_KP  2.1 //50
#define PID_Wy_KI  0.01  //500.0
#define PID_Wy_KD  0.0  //2.5
#define PID_Wy_INTEGRATION_LIMIT   1.5

#define PID_Wz_KP  0.1 //120.0
#define PID_Wz_KI  0.0  //16.7
#define PID_Wz_KD  0.0  //0.0
#define PID_Wz_INTEGRATION_LIMIT     1


static float g_vehicleMass = 0.545; //0.027 battery full:0.45

// Logging variables
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float desiredWx;
static float stateWx;
static float WxOutput;

static float desiredWy;
static float stateWy;
static float WyOutput;

static float desiredWz;
static float stateWz;
static float WzOutput;

static float m1y;
static float m2y;
static float m3y;
static float m4y;

static float Ix;
static float Iy;
static float Iz;

static float Arm_L = 0.13;

static float dt;

PidObject pidWx;
PidObject pidWy;
PidObject pidWz;

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
static inline float saturateYawTorque(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > 0.04f)
    return 0.04f;
  else if (in < -0.04f)
    return -0.04f;
  else
    return (float)in;
}

void controllerrpytReset(void)
{
  pidReset(&pidWx);
  pidReset(&pidWy);
  pidReset(&pidWz);
}

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
  Moter_p.x = 0.0f;
  Moter_p.y = 0.0f;
  Moter_p.z = 0.0f;
  Moter_p.w = 0.0f;
  Ix = 0.8f*powf(10.0f, -1.0f);
  Iy = 0.8f*powf(10.0f, -1.0f);
  Iz = 1.0f*powf(10.0f, -1.0f);
  // float dt;
  dt = (float)(1.0f/ATTITUDE_RATE);
  // kato: RATE_DO_EXECUTE is in stabilizer_types.h and ATTITUDE_RATE is 500 Hz
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }
// --- calculate Rollrate Pitchrate error (P, I, D) ---

  desiredWx = radians(setpoint->attitudeRate.roll);
  desiredWy = radians(setpoint->attitudeRate.pitch);
  desiredWz = radians(setpoint->attitudeRate.yaw);

  stateWx = radians(sensors->gyro.x);
  stateWy = radians(sensors->gyro.y);
  stateWz = radians(sensors->gyro.z);
  
  pidSetDesired(&pidWx, desiredWx);
  // WxOutput = saturateSignedInt16(pidUpdate(&pidWx, stateWx, true));
  WxOutput = pidUpdate(&pidWx, stateWx, true);

  pidSetDesired(&pidWy, desiredWy);
  // WyOutput = saturateSignedInt16(pidUpdate(&pidWy, stateWy, true));
  WyOutput = pidUpdate(&pidWy, stateWy, true);

  pidSetDesired(&pidWz, desiredWz);
  // WzOutput = saturateSignedInt16(pidUpdate(&pidWz, stateWz, true));
  WzOutput = pidUpdate(&pidWz, stateWz, true);

// --- thrust roll pitch yaw input vector [gram] ---

  // target_thrust.z = - 25.0f * thrustScale * Vel_error.z - 0.0f * z_error_i + 36000.0f;
  trpy_g.x = setpoint->acceleration.z * (float)100.0;
  trpy_g.y = sqrtf(2) * Iy * WxOutput / Arm_L  + (stateWx*Iz*stateWz - stateWz*Ix*stateWx); // F = M/L
  trpy_g.z = sqrtf(2) * Ix * WyOutput / Arm_L + (stateWy*Iz*stateWz - stateWz*Iy*stateWy);
  trpy_g.w = 0.0f;

// --- change trpy[gram] to Moter[gram]
  Moter_g = mvmul4(CT_rpyt2g, trpy_g);
// --- change gram to pwm
  
    if (Moter_g.x >= 0) {Moter_p.x = sqrtf((float)Moter_g.x*3.49342067707335e-9F+7.519565474518213e-10F)*5.725047696447257e+8F-1.210549048793241e+4F;}
    else {Moter_p.x = -(sqrtf((float)-Moter_g.x*3.49342067707335e-9F+7.519565474518213e-9F)*5.725047696447257e+8F-1.210549048793241e+3F);}
    
    if (Moter_g.y >= 0) {Moter_p.y = sqrtf((float)Moter_g.y*3.676708012100728e-9F+9.57548729512879e-10F)*5.439648711340767e+8F-1.33303255184544e+4F;}
    else {Moter_p.y = -(sqrtf((float)-Moter_g.y*3.676708012100728e-9F+9.57548729512879e-10F)*5.439648711340767e+8F-1.33303255184544e+4F);}

    if (Moter_g.z >= 0) {Moter_p.z = sqrtf((float)Moter_g.z*3.534256419343375e-9F+7.685109594847897e-10F)*5.658898966848527e+8F-1.206101077524088e+4F;}
    else{Moter_p.z = -(sqrtf((float)-Moter_g.z*3.534256419343375e-9F+7.685109594847897e-10F)*5.658898966848527e+8F-1.206101077524088e+4F);}
    
    if (Moter_g.w >= 0) {Moter_p.w = sqrtf((float)Moter_g.w*3.383110040629546e-9F+1.083081055571476e-9F)*5.911720210046228e+8F-1.571109747039748e+4F;}
    else{Moter_p.w = -(sqrtf((float)-Moter_g.w*3.383110040629546e-9F+1.083081055571476e-9F)*5.911720210046228e+8F-1.571109747039748e+4F);}

    // ---- change torque to pwm
    trpy_g.w = WzOutput*Iz;
    m1y = saturateYawTorque(0.25f*trpy_g.w);
    m2y = saturateYawTorque(-0.25f*trpy_g.w);
    m3y = saturateYawTorque(0.25f*trpy_g.w);
    m4y = saturateYawTorque(-0.25f*trpy_g.w);

    if (m1y >= 0) {Moter_p.x += m1y*2.433222931475113e+6F-powf(m1y, 2.0f)*8.262978550309481e+7F+powf(m1y, 3.0f)*1.980881715377327e+9F-powf(m1y, 4.0f)*1.793340165338566e+10F+2.214999601783874e+3F;}
    else {Moter_p.x += -(-m1y*2.433222931475113e+6F-powf(m1y, 2.0f)*8.262978550309481e+7F-powf(m1y, 3.0f)*1.980881715377327e+9F-powf(m1y, 4.0f)*1.793340165338566e+10F+2.214999601783874e+3F);}

    if (m2y >= 0) {Moter_p.y += m2y*2.149639083455116e+6F-powf(m2y, 2.0f)*6.803893846695112e+7F+powf(m2y, 3.0f)*1.536492175525759e+9F-powf(m2y, 4.0f)*1.292287693311652e+10F+2.828993334806114e+3F;}
    else {Moter_p.y += -(-m2y*2.149639083455116e+6F-powf(m2y, 2.0f)*6.803893846695112e+7F-powf(m2y, 3.0f)*1.536492175525759e+9F-powf(m2y, 4.0f)*1.292287693311652e+10F+2.828993334806114e+3F);}

    if (m3y >= 0) {Moter_p.z += m3y*2.484916365761186e+6F-powf(m3y, 2.0f)*9.434339044685014e+7F+powf(m3y, 3.0f)*2.545856842307786e+9F-powf(m3y, 4.0f)*2.543056087992886e+10F+2.704129092816906e+3F;}
    else {Moter_p.z += -(-m3y*2.484916365761186e+6F-powf(m3y, 2.0f)*9.434339044685014e+7F-powf(m3y, 3.0f)*2.545856842307786e+9F-powf(m3y, 4.0f)*2.543056087992886e+10F+2.704129092816906e+3F);}

    if (m4y >= 0) {Moter_p.w += m4y*2.211882424553347e+6F-powf(m4y, 2.0f)*7.041719505184467e+7F+powf(m4y, 3.0f)*1.582040637385563e+9F-powf(m4y, 4.0f)*1.320682078035621e+10F+2.633856322644595e+3F;}
    else {Moter_p.w += -(-m4y*2.211882424553347e+6F-powf(m4y, 2.0f)*7.041719505184467e+7F-powf(m4y, 3.0f)*1.582040637385563e+9F-powf(m4y, 4.0f)*1.320682078035621e+10F+2.633856322644595e+3F);}

  //calcurate input thrust and moment
  target_thrust.z = 0.0;
  M_p.x = 0.0;
  M_p.y = 0.0;
  M_p.z = 0.0;

// determine Thrust input 
  if (setpoint->mode.z != modeDisable  && trpy_g.x != 0){
    // control->thrust = massThrust * target_thrust.z;
    control->thrust = target_thrust.z; 

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
