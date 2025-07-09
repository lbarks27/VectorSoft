#ifndef ADAM_OPTIMIZER_H
#define ADAM_OPTIMIZER_H

#include <math.h>

class AdamOptimizer {
public:
  typedef float (*CostFunction)(const float* x, int n, const void* ctx);

  AdamOptimizer(int size,
                float lr = 0.001f,
                float b1 = 0.9f,
                float b2 = 0.999f,
                float eps = 1e-8f);

  ~AdamOptimizer();

  void step(CostFunction f, float* x, const void* ctx, bool debug = false);

private:
  int n;
  int t;
  float learningRate;
  float beta1;
  float beta2;
  float epsilon;
  float beta1_power;
  float beta2_power;
  float* m;
  float* v;
  float* grad;

  void computeGradient(CostFunction f, const float* x, int n, float* gradOut, const void* ctx, float h = 1e-4f);
};

#endif