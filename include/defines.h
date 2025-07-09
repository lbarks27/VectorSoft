#ifndef defines_h
#define defines_h

#include <Arduino.h>

float quatProduct0(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3);
float quatProduct1(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3);
float quatProduct2(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3);
float quatProduct3(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3);
  
float quatLength(float q0, float q1, float q2, float q3);
  
float quatNormalize0(float q0, float q1, float q2, float q3);
float quatNormalize1(float q0, float q1, float q2, float q3);
float quatNormalize2(float q0, float q1, float q2, float q3);
float quatNormalize3(float q0, float q1, float q2, float q3);
  
float quatConjugate0(float q0, float q1, float q2, float q3);
float quatConjugate1(float q0, float q1, float q2, float q3);
float quatConjugate2(float q0, float q1, float q2, float q3);
float quatConjugate3(float q0, float q1, float q2, float q3);
  
float quatInverse0(float q0, float q1, float q2, float q3);
float quatInverse1(float q0, float q1, float q2, float q3);
float quatInverse2(float q0, float q1, float q2, float q3);
float quatInverse3(float q0, float q1, float q2, float q3);
  
float vecRotation1(float q0, float q1, float q2, float q3, float v1, float v2, float v3);
float vecRotation2(float q0, float q1, float q2, float q3, float v1, float v2, float v3);
float vecRotation3(float q0, float q1, float q2, float q3, float v1, float v2, float v3);
  
float eulerToQuat0(float p, float y, float r);
float eulerToQuat1(float p, float y, float r);
float eulerToQuat2(float p, float y, float r);
float eulerToQuat3(float p, float y, float r);

float quatToEuler1(float q0, float q1, float q2, float q3);
float quatToEuler2(float q0, float q1, float q2, float q3);
float quatToEuler3(float q0, float q1, float q2, float q3);


//Vector
float vecLength(float v1, float v2, float v3);
    
float vecDot(float v1, float v2, float v3, float b1, float b2, float b3);
    
float vecAngle(float v1, float v2, float v3, float b1, float b2, float b3);
    
float vecCross1(float v1, float v2, float v3, float b1, float b2, float b3);
float vecCross2(float v1, float v2, float v3, float b1, float b2, float b3);
float vecCross3(float v1, float v2, float v3, float b1, float b2, float b3);

float vecProj1(float v1, float v2, float v3, float b1, float b2, float b3);
float vecProj2(float v1, float v2, float v3, float b1, float b2, float b3);
float vecProj3(float v1, float v2, float v3, float b1, float b2, float b3);


/*
float numericalDerivative(float (*func)(float, void*), float input, float step, void* ctx) {
  return (func(input + step, ctx) - func(input - step, ctx)) / (2.0f * step);
}*/


float rungeKutta4();

#endif