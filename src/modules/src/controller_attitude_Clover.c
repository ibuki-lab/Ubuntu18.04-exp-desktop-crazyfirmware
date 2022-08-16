/*
Editor: kato
Attitude controller of Big Crazyflie-Clover

main dependencies
  attitude_controller
  position_controller

function
  controllerattitudeCloverInit
  controllerattitudeCloverTest
  controllerattitudeClover

*/
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_attitude_Clover.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

// controll cycle rate 100Hz, ATTITUDE_RATE:define in stabilizer_type
#define ATTITUDE_UPDATE_DT  (float)(1.0f/ATTITUDE_RATE)

// attitude_t: define in atabilizer_type
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerattitudeCloverInit(void)
{   // attitude
    attitudeControllerInit(ATTITUDE_UPDATE_DT);
}

// attitude pid controller test(set parametor)
bool controllerattitudeCloverTest(void)
{
  bool pass = true;
  
  pass &= attitudeControllerTest();

  return pass;
}
// -180 < angle < 180
static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

//control: output, setpoint: reference, sensor:gyro, state:state, tick:time
void controllerattitudeClover(control_t *control, setpoint_t *setpoint,
                                        const sensorData_t *sensors,
                                        const state_t *state,
                                        const uint32_t tick)
{   // Control yaw angle , RATE_DO_EXECUTE: stabilizer_type.h
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)){
    // Desired yaw angle is integration of yaw rate
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw); // -180 < yaw < 180
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust; // manual controll, jou stick
    }

    // set desired attitude value
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    // get desired euler angle rate
    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                              attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                              &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                              rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
    
    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);
    // control->yaw = -control->yaw;
    control->yaw = 0;
    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->gyro.z;
  }

  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(controller)
// PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)