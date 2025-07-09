// esc_pwm.h
#ifndef ESC_PWM_H
#define ESC_PWM_H

#include <Arduino.h>
#include <Servo.h>

namespace ESC {

// Number of ESCs/props
static constexpr uint8_t NUM_MOTORS = 3;
// Arduino‐style pin numbers (change if you rewired)
static constexpr uint8_t PINS[NUM_MOTORS] = { 8, 9, 17 };

// Thrust range in Newtons
static constexpr float THRUST_MIN = 0.0f;
static constexpr float THRUST_MAX = 16.0f;  // set this above your 1676 g motor max

// Servo‐pulse bounds (µs)
static constexpr uint16_t PULSE_MIN = 1000;  // 1 ms → zero throttle
static constexpr uint16_t PULSE_MAX = 2000;  // 2 ms → full throttle

static Servo motors[NUM_MOTORS];

/**
 * @brief Attach all ESCs and arm them with zero throttle.
 *        Must be called once in setup().
 */
inline void init() {
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    motors[i].attach(PINS[i], PULSE_MIN, PULSE_MAX);
    motors[i].writeMicroseconds(PULSE_MIN);
  }
  // Wait for ESCs to arm (check your ESC manual—2 s is typical)
  delay(2000);
}

/**
 * @brief Map a thrust [N] into the servo pulse [µs].
 */
inline uint16_t thrustToPulse(float thrust) {
  if (thrust <= THRUST_MIN) return PULSE_MIN;
  if (thrust >= THRUST_MAX) return PULSE_MAX;
  float frac = (thrust - THRUST_MIN) / (THRUST_MAX - THRUST_MIN);
  return uint16_t(PULSE_MIN + frac * (PULSE_MAX - PULSE_MIN) + 0.5f);
}

/**
 * @brief Command one motor by thrust [N].
 */
inline void setThrust(uint8_t idx, float thrust) {
  if (idx >= NUM_MOTORS) return;
  motors[idx].writeMicroseconds(thrustToPulse(thrust));
}

/**
 * @brief Command all motors to the same thrust [N].
 */
inline void setAllThrust(float thrust) {
  uint16_t pulse = thrustToPulse(thrust);
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    motors[i].writeMicroseconds(pulse);
  }
}

} // namespace ESC

#endif // ESC_PWM_H