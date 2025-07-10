#ifndef guidance_h
#define guidance_h

#include <Arduino.h>
#include <guidance.h>
#include <sensor_fetch.h>
#include <control.h>
#include <defines.h>

#include <Eigen/Dense>

// Target direction vector (normalized) using Eigen
static Eigen::Vector3f vecTarg = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

#endif // guidance_h