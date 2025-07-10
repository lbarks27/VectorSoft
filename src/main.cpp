
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TeensyThreads.h>

#include "defines.h"
#include "adam_optimizer.h"
#include "control.h"
#include "datalogging.h"
#include "guidance.h"
#include "sensor_fetch.h"
#include "tvc_optimizer.h"
#include "esc_pwm.h"


#include <Eigen/Dense>

TVCState tvc;

// Initialize TVC motors in a circle
void initializeTVCCircle(TVCState& tvc) {
    const float radius = 0.09f;
    const float z = -1.0f;
    const float thrust = 5.0f; // Adjust based on your motor spec

    // Motor angles around the circle in radians (0°, 120°, 240°)
    float angles[3] = { 0.0f, 2.0f * PI / 3.0f, 4.0f * PI / 3.0f };
    TVCMotor* motors[3] = { &tvc.M1, &tvc.M2, &tvc.M3 };

    for (int i = 0; i < 3; ++i) {
        float theta = angles[i];
        motors[i]->position[0] = radius * cosf(theta);  // x
        motors[i]->position[1] = radius * sinf(theta);  // y
        motors[i]->position[2] = z;                     // z (below CoM)
        motors[i]->roll = theta + PI;                   // rolled outward (facing away)
        motors[i]->thrust = thrust;
    }
}

// Cost function for torque error
float torqueCost(const float* x, int n, const void* ctx) {
    const TVCState* tvc = static_cast<const TVCState*>(ctx);
    float torque[3] = {0};
    for (int i = 0; i < 3; ++i) {
        const TVCMotor* motor = (i == 0) ? &tvc->M1 : (i == 1) ? &tvc->M2 : &tvc->M3;
        float pitch = x[i * 2 + 0];
        float yaw   = x[i * 2 + 1];

        // Simplified thrust vector assuming thrust = -Z, rotated by pitch/yaw
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

AdamOptimizer optimizer(6);
float servoAngles[6] = {0};

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Serial to initialize

    Serial.println("Starting setup...");

    sensorsBegin(); // Initialize sensors
    initializeTVCCircle(tvc);
    setupServos();
    setDynamicTorque(0.1f, 0.1f, -0.05f); // Initial torque target

    Serial.println("Setup complete.");
}

void loop() {
    Serial.println("Beginning loop...");

    updateBNO055();  // Get IMU orientation and gyro
    updateBMP390();  // Get barometric altitude
    updateGPS();     // Optionally update GPS

    controlLaw();    // Compute torqueTarg[]

    // Print torqueTarg values to verify they are updated before being passed to setDynamicTorque
    Serial.print("Computed Target Torque: ");
    Serial.print(torqueTarg.x(), 4); Serial.print(", ");
    Serial.println(torqueTarg.y(), 4);

    setDynamicTorque(torqueTarg[0], torqueTarg[1], 0.0f); // Use only roll and pitch

    const float costThreshold = 0.01f;
    const int maxSteps = 100; 
    int step = 0;
    float cost = torqueCost(servoAngles, 6, &tvc);

    while (cost > costThreshold && step < maxSteps) {
        optimizer.step(torqueCost, servoAngles, &tvc, true);  // Debug enabled
        cost = torqueCost(servoAngles, 6, &tvc);
        step++;
    }

    TVC_out(servoAngles); // Write angles to physical servos

    Serial.print("Final Cost: "); Serial.print(cost, 6);
    Serial.print(" | Target Torque: ");
    Serial.print(tvc.targetTorque[0], 4); Serial.print(", ");
    Serial.print(tvc.targetTorque[1], 4); Serial.print(", ");
    Serial.print(tvc.targetTorque[2], 4);
    Serial.print(" | Angles: ");
    for (int i = 0; i < 6; ++i) {
        Serial.print(servoAngles[i], 4);
        Serial.print(i < 5 ? ", " : "\n");
    }

    // Print orientation (roll, pitch, yaw) derived from BNO_ROT
    extern Eigen::Matrix3f BNO_ROT;
    float roll  = atan2(BNO_ROT(2,1), BNO_ROT(2,2)) * RAD_TO_DEG;
    float pitch = asin(-BNO_ROT(2,0)) * RAD_TO_DEG;
    float yaw   = atan2(BNO_ROT(1,0), BNO_ROT(0,0)) * RAD_TO_DEG;
    Serial.print("Orientation (roll, pitch, yaw): ");
    Serial.print(roll, 4); Serial.print(", ");
    Serial.print(pitch, 4); Serial.print(", ");
    Serial.println(yaw, 4);

    delay(100);
}