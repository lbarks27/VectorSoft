#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "guidance.h"
#include "sensor_fetch.h"
#include "defines.h"

#include <Eigen/Dense>

// --- L1 Adaptive Control state & parameters ---
static Eigen::Vector2f sigma_hat   = Eigen::Vector2f::Zero();
static Eigen::Vector2f eta         = Eigen::Vector2f::Zero();
static Eigen::Vector2f xhatErr     = Eigen::Vector2f::Zero();
static Eigen::Vector2f xhatW       = Eigen::Vector2f::Zero();

// Adaptation gains (Γ) and filter cutoff (ωc) for roll and pitch
static const Eigen::Vector2f L1_Gamma = Eigen::Vector2f(50.0f, 50.0f);
static const Eigen::Vector2f L1_wc    = Eigen::Vector2f(20.0f, 15.0f);

// Control loop period (seconds)
static const float L1_dt = 0.002f;  // 500 Hz

// ----- Error and command vectors -----
static Eigen::Vector3f GLOBvecErr;
static Eigen::Vector3f BODYvecErr;
static Eigen::Vector2f angAccTarg;
static Eigen::Vector2f torqueTarg;

// Dynamics constants
static const Eigen::Vector3f MMOI = Eigen::Vector3f(0.2218606474f, 0.2218606474f, 0.01f); // kg·m²
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
  // 1) Global error: cross product between measured forward axis and target vector
  Eigen::Vector3f forward = BNO_ROT.row(0).transpose();
  Eigen::Vector3f target(vecTarg[0], vecTarg[1], vecTarg[2]);
  GLOBvecErr = forward.cross(target);

  // 2) Body-frame error: rotate global error into body frame using quaternion conjugate
  BODYvecErr = BNO_QUAT.conjugate() * GLOBvecErr;

  // 3) Desired angular acceleration (roll, pitch)
  float gyroZ = BNO_GYRO.z();
  float gyroY = BNO_GYRO.y();
  angAccTarg[0] = (GAIN_P * -BODYvecErr.z()) + (GAIN_D * gyroZ);
  angAccTarg[1] = (GAIN_P * BODYvecErr.y()) + (GAIN_D * -gyroY);

  // 4) Torque targets: τ = I * α
  torqueTarg[0] = angAccTarg[0] * MMOI.x();
  torqueTarg[1] = angAccTarg[1] * MMOI.y();

  // --- L1 adaptive augmentation ---
  // Predictor dynamics (Euler integration)
  Eigen::Vector2f xhatErr_dot = xhatW;
  Eigen::Vector2f xhatW_dot   = (torqueTarg + sigma_hat).cwiseQuotient(MMOI.head<2>());
  xhatErr += xhatErr_dot * L1_dt;
  xhatW   += xhatW_dot   * L1_dt;

  // Compute prediction error on attitude error
  Eigen::Vector2f err2;
  err2 << -BODYvecErr.z(), BODYvecErr.y();
  Eigen::Vector2f predErr = xhatErr - err2;

  // Adaptation law
  Eigen::Vector2f sigma_dot = -L1_Gamma.cwiseProduct(predErr);
  sigma_hat += sigma_dot * L1_dt;

  // Low-pass filter the adaptive term
  Eigen::Vector2f eta_dot = -L1_wc.cwiseProduct(eta) + L1_wc.cwiseProduct(sigma_hat);
  eta += eta_dot * L1_dt;

  // Augment the torque command
  torqueTarg -= eta;

  // 5) Compute thrust vectors if needed...
}

#endif // CONTROL_H
