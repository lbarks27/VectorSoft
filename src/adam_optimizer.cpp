#include "adam_optimizer.h"
#include <math.h>

AdamOptimizer::AdamOptimizer(int size, float lr, float b1, float b2, float eps)
    : n(size), t(0), learningRate(lr), beta1(b1), beta2(b2), epsilon(eps),
      beta1_power(1.0f), beta2_power(1.0f)
{
    m = new float[n]();
    v = new float[n]();
    grad = new float[n]();
}

AdamOptimizer::~AdamOptimizer() {
    delete[] m;
    delete[] v;
    delete[] grad;
}

void AdamOptimizer::computeGradient(CostFunction f, const float* x, int n, float* gradOut, const void* ctx, float h) {
    float* temp = new float[n];
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) temp[j] = x[j];
        temp[i] += h;
        float f1 = f(temp, n, ctx);
        temp[i] -= 2 * h;
        float f2 = f(temp, n, ctx);
        gradOut[i] = (f1 - f2) / (2 * h);
    }
    delete[] temp;
}

void AdamOptimizer::step(CostFunction f, float* x, const void* ctx, bool debug) {
    t++;
    beta1_power *= beta1;
    beta2_power *= beta2;

    computeGradient(f, x, n, grad, ctx);

    for (int i = 0; i < n; ++i) {
        m[i] = beta1 * m[i] + (1.0f - beta1) * grad[i];
        v[i] = beta2 * v[i] + (1.0f - beta2) * grad[i] * grad[i];

        float m_hat = m[i] / (1.0f - beta1_power);
        float v_hat = v[i] / (1.0f - beta2_power);

        x[i] -= learningRate * m_hat / (sqrtf(v_hat) + epsilon);
    }
    if (debug) {
        // debug prints can be added here if needed
    }
}