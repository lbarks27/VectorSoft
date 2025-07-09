// tvc_optimizer.cpp
#include "tvc_optimizer.h"
#include "defines.h"
#include <Arduino.h> // For Serial output if using on Teensy/Arduino
#include <Servo.h>   // Ensure Servo library is available

#ifndef S_microseconds
#define S_microseconds 0
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795f
#endif

#ifndef M_gear_coef
#define M_gear_coef 1.0f
#endif

#ifndef S1_zero
#define S1_zero 90.0f
#endif
#ifndef S2_zero
#define S2_zero 90.0f
#endif
#ifndef S3_zero
#define S3_zero 90.0f
#endif
#ifndef S4_zero
#define S4_zero 90.0f
#endif
#ifndef S5_zero
#define S5_zero 90.0f
#endif
#ifndef S6_zero
#define S6_zero 90.0f
#endif

#ifndef S1_pin
#define S1_pin 2
#endif
#ifndef S2_pin
#define S2_pin 3
#endif
#ifndef S3_pin
#define S3_pin 4
#endif
#ifndef S4_pin
#define S4_pin 5
#endif
#ifndef S5_pin
#define S5_pin 6
#endif
#ifndef S6_pin
#define S6_pin 7
#endif

Servo S1, S2, S3, S4, S5, S6;
float servoCmd[6] = {0};

// Global TVC state
extern TVCState tvc;

void setDynamicTorque(float tx, float ty, float tz) {
  tvc.targetTorque[0] = tx;
  tvc.targetTorque[1] = ty;
  tvc.targetTorque[2] = tz;
}

void TVC_servo_to_vec(float S1, float S2, float S3, float S4, float S5, float S6) {
  // Placeholder: convert servo angles (pitch, yaw) to thrust vectors (update M1_vec, etc.)
}

float torqueCost(const float* x, int n, const TVCState* tvc) {
  float torque[3] = {0};

  for (int i = 0; i < 3; ++i) {
    const TVCMotor* motor = (i == 0) ? &tvc->M1 : (i == 1) ? &tvc->M2 : &tvc->M3;
    float pitch = x[i * 2 + 0];
    float yaw   = x[i * 2 + 1];

    // Compute rotated thrust vector (Z-axis negative thrust, pitched/yawed)
    float thrustVec[3] = {
      -motor->thrust * sinf(yaw),
      -motor->thrust * sinf(pitch),
      -motor->thrust * cosf(yaw) * cosf(pitch)
    };

    torque[0] += vecCross1(motor->position[0], motor->position[1], motor->position[2], thrustVec[0], thrustVec[1], thrustVec[2]);
    torque[1] += vecCross2(motor->position[0], motor->position[1], motor->position[2], thrustVec[0], thrustVec[1], thrustVec[2]);
    torque[2] += vecCross3(motor->position[0], motor->position[1], motor->position[2], thrustVec[0], thrustVec[1], thrustVec[2]);
  }

  float dx = tvc->targetTorque[0] - torque[0];
  float dy = tvc->targetTorque[1] - torque[1];
  float dz = tvc->targetTorque[2] - torque[2];

  return vecLength(dx, dy, dz);
}

void optimizeTVC(TVCState& tvc, float* x, AdamOptimizer& optimizer, int steps, bool debug) {
  for (int step = 0; step < steps; ++step) {
    optimizer.step((AdamOptimizer::CostFunction)torqueCost, x, &tvc);

    // Clamp each servo's pitch/yaw to circular cone limit
    for (int i = 0; i < 3; ++i) {
      clampServoCircle(x[i * 2 + 0], x[i * 2 + 1], SERVO_ANGLE_LIMIT);
    }

    if (debug) {
      Serial.print("Step "); Serial.print(step);
      Serial.print(" | Cost: "); Serial.print(torqueCost(x, 6, &tvc), 6);
      Serial.print(" | Angles: ");
      for (int i = 0; i < 6; ++i) {
        Serial.print(x[i], 4);
        Serial.print(i < 5 ? ", " : "\n");
      }
    }
  }

  // Output commands to servos directly
  servoCmd[0] = x[0] * RAD_TO_DEG * M_gear_coef + S1_zero;
  servoCmd[1] = x[1] * RAD_TO_DEG * M_gear_coef + S2_zero;
  servoCmd[2] = x[2] * RAD_TO_DEG * M_gear_coef + S3_zero;
  servoCmd[3] = x[3] * RAD_TO_DEG * M_gear_coef + S4_zero;
  servoCmd[4] = x[4] * RAD_TO_DEG * M_gear_coef + S5_zero;
  servoCmd[5] = x[5] * RAD_TO_DEG * M_gear_coef + S6_zero;

  if (!S_microseconds) {
    S1.write(servoCmd[0]);
    S2.write(servoCmd[1]);
    S3.write(servoCmd[2]);
    S4.write(servoCmd[3]);
    S5.write(servoCmd[4]);
    S6.write(servoCmd[5]);
  } else {
    // Optionally support microsecond PWM here
  }
}

// Call this in setup() to attach the servos to their pins
void setupServos() {
  S1.attach(S1_pin);
  S2.attach(S2_pin);
  S3.attach(S3_pin);
  S4.attach(S4_pin);
  S5.attach(S5_pin);
  S6.attach(S6_pin);
}

// Send servo angles (in radians) to the physical servos, with clamping and transformation
void TVC_out(float* servo_cmd_rad) {
  const float angle_limit = SERVO_ANGLE_LIMIT;

  // Clamp each servo's pitch/yaw to circular cone limit
  for (int i = 0; i < 3; ++i) {
    clampServoCircle(servo_cmd_rad[i * 2 + 0], servo_cmd_rad[i * 2 + 1], angle_limit);
  }

  float command[6];
  command[0] = servo_cmd_rad[0] * RAD_TO_DEG * M_gear_coef + S1_zero;
  command[1] = servo_cmd_rad[1] * RAD_TO_DEG * M_gear_coef + S2_zero;
  command[2] = servo_cmd_rad[2] * RAD_TO_DEG * M_gear_coef + S3_zero;
  command[3] = servo_cmd_rad[3] * RAD_TO_DEG * M_gear_coef + S4_zero;
  command[4] = servo_cmd_rad[4] * RAD_TO_DEG * M_gear_coef + S5_zero;
  command[5] = servo_cmd_rad[5] * RAD_TO_DEG * M_gear_coef + S6_zero;

  if (!S_microseconds) {
    S1.write(command[0]);
    S2.write(command[1]);
    S3.write(command[2]);
    S4.write(command[3]);
    S5.write(command[4]);
    S6.write(command[5]);
  } else {
    // Optionally support microsecond PWM here
  }
}