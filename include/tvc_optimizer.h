#ifndef TVC_OPTIMIZER_H
#define TVC_OPTIMIZER_H

#include <math.h>
#include "adam_optimizer.h"

#define SERVO_ANGLE_LIMIT  0.2618f   // ~15 degrees in radians
#define SERVO_DT           0.02f     // 50 Hz loop rate

extern float dynamicTorqueInput[3];
void setDynamicTorque(float tx, float ty, float tz);

struct TVCMotor {
  float roll;         // radians
  float thrust;       // newtons
  float position[3];  // meters
};

struct TVCState {
  TVCMotor M1, M2, M3;
  float targetTorque[3];
};

// Clamp value between min and max
inline float clamp(float x, float minVal, float maxVal) {
  return fmaxf(minVal, fminf(maxVal, x));
}

// Clamp servo pitch/yaw to lie within a circular cone limit
inline void clampServoCircle(float& pitch, float& yaw, float maxAngleRad) {
  float mag = sqrtf(pitch * pitch + yaw * yaw);
  if (mag > maxAngleRad) {
    float scale = maxAngleRad / mag;
    pitch *= scale;
    yaw   *= scale;
  }
}

// Convert servo angles to thrust vector directions (implementation needed)
void TVC_servo_to_vec(float S1, float S2, float S3, float S4, float S5, float S6);

// Cost function to evaluate torque error
float torqueCost(const float* x, int n, const TVCState* tvc);

// Run optimization using Adam to minimize torque error
void optimizeTVC(TVCState& tvc, float* x, AdamOptimizer& optimizer, int steps = 100, bool debug = true);

void setupServos();
void TVC_out(float* servo_cmd_rad);

#endif