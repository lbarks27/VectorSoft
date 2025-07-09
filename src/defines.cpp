#include <Arduino.h>
#include "defines.h"

//_____________________________________________________________________________________________________________________________________________________________________________________________________
//quat
float quatProduct0(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3) {
    return p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3;
  }
float quatProduct1(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3) {
    return p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2;
  }
float quatProduct2(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3) {
    return p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1;
  }
float quatProduct3(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3) {
    return p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0;
  }
  
float quatLength(float q0, float q1, float q2, float q3) {
    return pow(pow(q0, 2) + pow(q1, 2) + pow(q2, 2) + pow(q3, 2), 0.5);
  }
  
float quatNormalize0(float q0, float q1, float q2, float q3) {
    return q0 / quatLength(q0, q1, q2, q3);
  }
float quatNormalize1(float q0, float q1, float q2, float q3) {
    return q1 / quatLength(q0, q1, q2, q3);
  }
float quatNormalize2(float q0, float q1, float q2, float q3) {
    return q2 / quatLength(q0, q1, q2, q3);
  }
float quatNormalize3(float q0, float q1, float q2, float q3) {
    return q3 / quatLength(q0, q1, q2, q3);
  }
  
float quatConjugate0(float q0, float q1, float q2, float q3) {
    return q0;
  }
float quatConjugate1(float q0, float q1, float q2, float q3) {
    return q1 * -1;
  }
float quatConjugate2(float q0, float q1, float q2, float q3) {
    return q2 * -1;
  }
float quatConjugate3(float q0, float q1, float q2, float q3) {
    return q3 * -1;
  }
  
float quatInverse0(float q0, float q1, float q2, float q3) {
    return quatConjugate0(q0, q1, q2, q3) / pow(quatLength(q0, q1, q2, q3), 2);
  }
float quatInverse1(float q0, float q1, float q2, float q3) {
    return quatConjugate1(q0, q1, q2, q3) / pow(quatLength(q0, q1, q2, q3), 2);
  }
float quatInverse2(float q0, float q1, float q2, float q3) {
    return quatConjugate2(q0, q1, q2, q3) / pow(quatLength(q0, q1, q2, q3), 2);
  }
float quatInverse3(float q0, float q1, float q2, float q3) {
    return quatConjugate3(q0, q1, q2, q3) / pow(quatLength(q0, q1, q2, q3), 2);
  }
  
float vecRotation1(float q0, float q1, float q2, float q3, float v1, float v2, float v3) {
    return quatProduct1(q0, q1, q2, q3, quatProduct0(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct1(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct2(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct3(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)));
  }
float vecRotation2(float q0, float q1, float q2, float q3, float v1, float v2, float v3) {
    return quatProduct2(q0, q1, q2, q3, quatProduct0(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct1(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct2(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct3(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)));
  }
float vecRotation3(float q0, float q1, float q2, float q3, float v1, float v2, float v3) {
    return quatProduct3(q0, q1, q2, q3, quatProduct0(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct1(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct2(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)), quatProduct3(0, v1, v2, v3, quatInverse0(q0, q1, q2, q3), quatInverse1(q0, q1, q2, q3), quatInverse2(q0, q1, q2, q3), quatInverse3(q0, q1, q2, q3)));
  }

//euler/quat conversions
float eulerToQuat0(float p, float y, float r) {
    return (cos(p / 2) * cos(y / 2) * cos(r / 2)) + (sin(p / 2) * sin(y / 2) * sin(r / 2));
}
float eulerToQuat1(float p, float y, float r) {
    return (sin(p / 2) * cos(y / 2) * cos(r / 2)) - (cos(p / 2) * sin(y / 2) * sin(r / 2));
}
float eulerToQuat2(float p, float y, float r) {
    return (cos(p / 2) * sin(y / 2) * cos(r / 2)) + (sin(p / 2) * cos(y / 2) * sin(r / 2));
}
float eulerToQuat3(float p, float y, float r) {
    return (cos(p / 2) * cos(y / 2) * sin(r / 2)) - (sin(p / 2) * sin(y / 2) * cos(r / 2));
}

float quatToEuler1(float q0, float q1, float q2, float q3) {
    return ((pow(q0, 2)) - (pow(q1, 2)) - (pow(q2, 2)) + (pow(q3, 2)));
}
float quatToEuler2(float q0, float q1, float q2, float q3) {
    return asin(2 * ((q0 * q2) + (q3 * q1)));
}
float quatToEuler3(float q0, float q1, float q2, float q3) {
    return ((pow(q0, 2)) + (pow(q1, 2)) - (pow(q2, 2)) - (pow(q3, 2)));
}

  //Vector
float vecLength(float v1, float v2, float v3) {
    return pow(pow(v1, 2) + pow(v2, 2) + pow(v3, 2), 0.5);
}
    
float vecDot(float v1, float v2, float v3, float b1, float b2, float b3) {
  return v1 * b1 + v2 * b2 + v3 * b3;
}
    
float vecAngle(float v1, float v2, float v3, float b1, float b2, float b3) {
  return acos(vecDot(v1, v2, v3, b1, b2, b3) / vecLength(v1, v2, v3) * vecLength(b1, b2, b3));
}
    
float vecCross1(float v1, float v2, float v3, float b1, float b2, float b3) {
  return v2 * b3 - v3 * b2;
}
float vecCross2(float v1, float v2, float v3, float b1, float b2, float b3) {
  return v3 * b1 - v1 * b3;
}
float vecCross3(float v1, float v2, float v3, float b1, float b2, float b3) {
  return v1 * b2 - v2 * b1;
}

//MUST INPUT UNIT VECTOR FOR b
float vecProj1(float v1, float v2, float v3, float b1, float b2, float b3) {
  return v1 - (vecDot(v1, v2, v3, b1, b2, b3) * b1);
}
float vecProj2(float v1, float v2, float v3, float b1, float b2, float b3) {
  return v2 - (vecDot(v1, v2, v3, b1, b2, b3) * b2);
}
float vecProj3(float v1, float v2, float v3, float b1, float b2, float b3) {
  return v3 - (vecDot(v1, v2, v3, b1, b2, b3) * b3);
}

    //derivatives in h file

    //Integration
float rungeKutta4() {

}