#ifndef guidance_h
#define guidance_h



#include <Arduino.h>

#include <guidance.h>
#include <sensor_fetch.h>
#include <control.h>
#include <defines.h>




// Current and target positions
float pos_current[3] = {0, 0, 0};  // To be updated with sensor data
float pos_target[3]  = {0, 0, 10}; // Example target: 10 meters up

// Output vector pointing from current to target position
float vecTarg[3] = {0, 0, 1};  // Initialized pointing up

// Proportional gain for feedback (unitless since we're normalizing)
float kP_position = 1.0f;

// Update target vector based on current and desired positions
inline void updateTargetVector() {
  float dx = pos_target[0] - pos_current[0];
  float dy = pos_target[1] - pos_current[1];
  float dz = pos_target[2] - pos_current[2];

  float mag = sqrtf(dx * dx + dy * dy + dz * dz);
  if (mag > 0.001f) {
    vecTarg[0] = kP_position * dx / mag;
    vecTarg[1] = kP_position * dy / mag;
    vecTarg[2] = kP_position * dz / mag;
  } else {
    vecTarg[0] = 0;
    vecTarg[1] = 0;
    vecTarg[2] = 1;
  }
}



#endif