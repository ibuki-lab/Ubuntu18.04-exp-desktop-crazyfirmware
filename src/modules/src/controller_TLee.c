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
#include "controller_TLee.h"
#include "physicalConstants.h"

static float g_vehicleMass = CF_MASS; //0.027
static float massThrust = 132000;

// XY Position PD
static float kp_xy = 0.4;       // P
static float kd_xy = 0.2;       // D

// Z Position
static float kp_z = 1.25;       // P
static float kd_z = 0.4;        // D

// Attitude
static float kR_xy = 60000; // P
static float kw_xy = 10000; // D

// Yaw
static float kR_z = 60000; // P
static float kw_z = 12000; // D

// roll and pitch angular velocity
static float kd_omega_rp = 200; // D


// Logging variables
static struct vec z_axis_desired;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerTLeeReset(void)
{

}

void controllerTLeeInit(void)
{
  controllerTLeeReset();
}

bool controllerTLeeTest(void)
{
  return true;
}

void controllerTLee(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{                           // kato:
  struct vec r_error;               // position error
  struct vec v_error;               // velosity error
  struct vec target_thrust;         // nominal thrust
  //struct vec z_axis;                // body axis Z 
  float current_thrust;             // actual thust
  struct vec x_axis_desired;        // desired axis X
  struct vec y_axis_desired;        // desired axis Y
  struct vec x_c_des;               // desired axis Xc
  struct vec eR, ew, M;             // rotation Matrix error, anguler verosity error, Moment
  float desiredYaw = 0;             //deg
// kato: RATE_DO_EXECUTE is in stabilizer_types.h and ATTITUDE_RATE is 500 Hz
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

// --- Get Setpoints[reference] and state --- 
  //  mkvec is in math3d.h. Create 3D vector
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

// --- Calculate Position and Velocity Error ---
  // vsub is subtraction of 3D vector
  r_error = vsub(setpointPos, statePos);
  v_error = vsub(setpointVel, stateVel);

// --- Calculate desired thrust [F_des] ---
  // State feedback controller, only modeAbs
  if (setpoint->mode.x == modeAbs) {
    target_thrust.x = g_vehicleMass * setpoint->acceleration.x                       + kp_xy * r_error.x + kd_xy * v_error.x;
    target_thrust.y = g_vehicleMass * setpoint->acceleration.y                       + kp_xy * r_error.y + kd_xy * v_error.y;
    target_thrust.z = g_vehicleMass * (setpoint->acceleration.z + GRAVITY_MAGNITUDE) + kp_z  * r_error.z + kd_z  * v_error.z;
  } 
  
// --- Get deisred Yaw[rad] ---
  //If and elseif statements does same thing.
  if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);   // get Yaw directly
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    struct vec rpy = quat2rpy(setpoint_quat);       // get Yaw from quaternion
    desiredYaw = degrees(rpy.z);
  }

// --- Get current Z_axis[3Dvec] of attitude ---
  // get current quaternion
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  // get current matrix form current quaternion
  struct mat33 R = quat2rotmat(q);
  // get current z_axis of quad
  // z_axis = mcolumn(R, 2);

// --- Calculate desired Z-axis[zB_des] ---
  z_axis_desired = vnormalize(target_thrust);

// --- Calculate input thrust [F] ---
  //kato: z_axis --- current z_axis of attitude 
  // current_thrust = vdot(target_thrust, z_axis);           // default
  current_thrust = vdot(target_thrust, z_axis_desired);   // Try this

  
// --- Calculate xC_des[3D vector] ---
  // desired x, y, z axis of desired yaw rotation Matrix 
  x_c_des.x = cosf(radians(desiredYaw));
  x_c_des.y = sinf(radians(desiredYaw));
  x_c_des.z = 0;

// --- Calculate yB_des[3D vector] ---
  // cross of desired Z axis and desired X axis and normalize
  y_axis_desired = vnormalize(vcross(z_axis_desired, x_c_des));

// --- Calculate xB_des[3D vector] ---
  // cross of desired Y axis and desired Z axis 
  x_axis_desired = vcross(y_axis_desired, z_axis_desired);

// --- Calculate eR: attitude error ---
  // create desired Rotation Matrix
  struct mat33 Rdes = mcolumns(
    mkvec(x_axis_desired.x, x_axis_desired.y, x_axis_desired.z),
    mkvec(y_axis_desired.x, y_axis_desired.y, y_axis_desired.z),
    mkvec(z_axis_desired.x, z_axis_desired.y, z_axis_desired.z));
  
  // Get current R and desired R transpose
  struct mat33 R_transpose = mtranspose(R);
  struct mat33 Rdes_transpose = mtranspose(Rdes);
  
  // Calculate Rotation Error
  struct mat33 eRM = msub(mmul(Rdes_transpose, R), mmul(R_transpose, Rdes));
  
  // Get Error of Skew-symmetric matrix
  eR.x = eRM.m[2][1]/2;
  eR.y = -eRM.m[0][2]/2;
  eR.z = eRM.m[1][0]/2;

// --- Calculate ew: angluer velocity error
  // Get current anguler velocity
  float stateAttitudeRateRoll = radians(sensors->gyro.x);
  float stateAttitudeRatePitch = -radians(sensors->gyro.y);
  float stateAttitudeRateYaw = radians(sensors->gyro.z);
  
  // Calculate anguler velosity error
  ew.x = radians(setpoint->attitudeRate.roll) - stateAttitudeRateRoll;
  ew.y = -radians(setpoint->attitudeRate.pitch) - stateAttitudeRatePitch;
  ew.z = radians(setpoint->attitudeRate.yaw) - stateAttitudeRateYaw;
  
  // --- Calculate Moment ---
  M.x = -kR_xy * eR.x + kw_xy * ew.x;
  M.y = -kR_xy * eR.y + kw_xy * ew.y;
  M.z = -kR_z  * eR.z + kw_z  * ew.z;

  // determine Thrust input
  if (setpoint->mode.z != modeDisable){
    control->thrust = massThrust * current_thrust;
  } else{
      control->thrust = (float)0;
  }

// --- Get euler angle and accelaletion 
  r_roll = radians(sensors->gyro.x);
  r_pitch = -radians(sensors->gyro.y);
  r_yaw = radians(sensors->gyro.z);
  accelz = sensors->acc.z;

// --- Logging param and saturation input ---
  cmd_thrust = control->thrust;

  if (control->thrust > 0) {
    control->roll = clamp(M.x, -32000, 32000);
    control->pitch = clamp(M.y, -32000, 32000);
    control->yaw = clamp(-M.z, -32000, 32000);

    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

  } else {
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    controllerTLeeReset();
  }
}

PARAM_GROUP_START(ctrlTLee)
PARAM_ADD(PARAM_FLOAT, kp_xy, &kp_xy)
PARAM_ADD(PARAM_FLOAT, kd_xy, &kd_xy)
PARAM_ADD(PARAM_FLOAT, kp_z, &kp_z)
PARAM_ADD(PARAM_FLOAT, kd_z, &kd_z)
PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)
PARAM_ADD(PARAM_FLOAT, massThrust, &massThrust)
PARAM_ADD(PARAM_FLOAT, kR_xy, &kR_xy)
PARAM_ADD(PARAM_FLOAT, kR_z, &kR_z)
PARAM_ADD(PARAM_FLOAT, kw_xy, &kw_xy)
PARAM_ADD(PARAM_FLOAT, kw_z, &kw_z)
PARAM_ADD(PARAM_FLOAT, kd_omega_rp, &kd_omega_rp)
PARAM_GROUP_STOP(ctrlTLee)

LOG_GROUP_START(ctrlTLee)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, zdx, &z_axis_desired.x)
LOG_ADD(LOG_FLOAT, zdy, &z_axis_desired.y)
LOG_ADD(LOG_FLOAT, zdz, &z_axis_desired.z)
LOG_GROUP_STOP(ctrlTLee)