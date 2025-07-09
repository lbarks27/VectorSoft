#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "guidance.h"
#include "sensor_fetch.h"
#include "defines.h"

// ----- Error and command vectors -----
static double GLOBvecErr[3];    // global-frame error
static double BODYvecErr[3];    // body-frame error
static double angAccTarg[2];    // target angular acceleration [roll, pitch]
static double torqueTarg[2];     // target torque [roll, pitch]

// Dynamics constants
static const float MMOI[3] = {0.2218606474f, 0.2218606474f, 0.01f}; // kg·m²
static const float cm_TVC_distance = 0.5f;    // m
static const float mass = 0.749f;             // kg
static const float thrustForceMeas = 2.0f;     // N

// Controller gains
#define GAIN_P (-0.1)
#define GAIN_D (0.2)

// External target vector (in guidance.h or defines.h)
// extern float vecTarg[3];

// ----- Control law -----
inline void controlLaw() {
  // 1) Global error: cross product between measured forward axis and target
  GLOBvecErr[0] = vecCross1(
    BNO_ROT[0][0], BNO_ROT[0][1], BNO_ROT[0][2],
    vecTarg[0], vecTarg[1], vecTarg[2]);
  GLOBvecErr[1] = vecCross2(
    BNO_ROT[0][0], BNO_ROT[0][1], BNO_ROT[0][2],
    vecTarg[0], vecTarg[1], vecTarg[2]);
  GLOBvecErr[2] = vecCross3(
    BNO_ROT[0][0], BNO_ROT[0][1], BNO_ROT[0][2],
    vecTarg[0], vecTarg[1], vecTarg[2]);

  // 2) Body-frame error: rotate global error into body frame
  // Compute inverse (conjugate) of corrected quaternion
  float iqw = BNO_QUAT_W;
  float iqx = -BNO_QUAT_X;
  float iqy = -BNO_QUAT_Y;
  float iqz = -BNO_QUAT_Z;

  BODYvecErr[0] = vecRotation1(
    iqw, iqx, iqy, iqz,
    GLOBvecErr[0], GLOBvecErr[1], GLOBvecErr[2]);
  BODYvecErr[1] = vecRotation2(
    iqw, iqx, iqy, iqz,
    GLOBvecErr[0], GLOBvecErr[1], GLOBvecErr[2]);
  BODYvecErr[2] = vecRotation3(
    iqw, iqx, iqy, iqz,
    GLOBvecErr[0], GLOBvecErr[1], GLOBvecErr[2]);

  // 3) Desired angular acceleration (roll, pitch)
  // Use gyro Z for roll rate, gyro Y for pitch rate
  double gyroZ = BNO_GYRO_Z;
  double gyroY = BNO_GYRO_Y;
  angAccTarg[0] = (GAIN_P * -BODYvecErr[2]) + (GAIN_D * gyroZ);
  angAccTarg[1] = (GAIN_P * BODYvecErr[1]) + (GAIN_D * -gyroY);

  // 4) Torque targets: τ = I * α
  torqueTarg[0] = angAccTarg[0] * MMOI[0];
  torqueTarg[1] = angAccTarg[1] * MMOI[1];

  // 5) Compute thrust vectors if needed (using torqueTarg and geometry)
  // e.g. compute BODYthrustVector[...] and GLOBthrustVector[...] here
}

#endif // CONTROL_H
