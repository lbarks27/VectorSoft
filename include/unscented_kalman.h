

#ifndef UNSCENTED_KALMAN_H
#define UNSCENTED_KALMAN_H

#include <vector>
#include <functional>
#include <Eigen/Dense>

// Generic Unscented Kalman Filter for N-state, M-process noise, and variable measurements.
template<int N, int L = N>
class UnscentedKalmanFilter {
public:
    using StateVec = Eigen::Matrix<double, N, 1>;
    using CovMatrix = Eigen::Matrix<double, N, N>;
    using SigmaPoints = Eigen::Matrix<double, N, 2 * N + 1>;
    
    // Measurement function: takes state and returns measurement of dimension M
    using MeasFunc = std::function<Eigen::VectorXd(const StateVec&)>;
    // Measurement noise covariance
    using MeasCov = Eigen::MatrixXd;

    UnscentedKalmanFilter(
        const StateVec &x0,
        const CovMatrix &P0,
        const Eigen::Matrix<double, N, N> &Q_in,
        double alpha = 1e-3, double beta = 2.0, double kappa = 0.0
    )
    : x(x0), P(P0), Q(Q_in),
      lam(alpha * alpha * (N + kappa) - N),
      gamma(std::sqrt(N + lam))
    {}

    // Process model: f(x, dt)
    void setProcessModel(std::function<StateVec(const StateVec&, double)> f_in) {
        f = f_in;
    }

    // Add a sensor measurement function
    // Each must be paired with its measurement noise covariance R
    void addSensor(const MeasFunc &h, const MeasCov &R) {
        sensors.push_back({h, R});
    }

    // Predict step
    void predict(double dt) {
        // Generate sigma points
        Eigen::Matrix<double, N, N> A = P.llt().matrixL();
        sigmaPoints.col(0) = x;
        for (int i = 0; i < N; ++i) {
            sigmaPoints.col(i + 1)     = x + gamma * A.col(i);
            sigmaPoints.col(i + 1 + N) = x - gamma * A.col(i);
        }

        // Propagate through process model
        for (int i = 0; i < 2 * N + 1; ++i) {
            sigmaPoints.col(i) = f(sigmaPoints.col(i), dt);
        }

        // Compute predicted mean and covariance
        x.fill(0.0);
        P.fill(0.0);
        computeWeights();
        for (int i = 0; i < 2 * N + 1; ++i) {
            x += wm(i) * sigmaPoints.col(i);
        }
        for (int i = 0; i < 2 * N + 1; ++i) {
            StateVec d = sigmaPoints.col(i) - x;
            P += wc(i) * (d * d.transpose());
        }
        P += Q;
    }

    // Update step for all registered sensors
    void update() {
        for (auto &sensor : sensors) {
            const MeasFunc &h = sensor.first;
            const MeasCov &R = sensor.second;
            int M = R.rows();

            // Transform sigma points into measurement space
            Eigen::MatrixXd Z(M, 2 * N + 1);
            for (int i = 0; i < 2 * N + 1; ++i) {
                Z.col(i) = h(sigmaPoints.col(i));
            }

            // Predicted measurement mean
            Eigen::VectorXd zPred = Eigen::VectorXd::Zero(M);
            for (int i = 0; i < 2 * N + 1; ++i)
                zPred += wm(i) * Z.col(i);

            // Measurement covariance & cross-covariance
            Eigen::MatrixXd S = Eigen::MatrixXd::Zero(M, M);
            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N, M);
            for (int i = 0; i < 2 * N + 1; ++i) {
                Eigen::VectorXd dz = Z.col(i) - zPred;
                Eigen::Matrix<double, N, 1> dx = sigmaPoints.col(i) - x;
                S += wc(i) * dz * dz.transpose();
                C += wc(i) * dx * dz.transpose();
            }
            S += R;

            // Kalman gain
            Eigen::Matrix<double, N, Eigen::Dynamic> K = C * S.inverse();

            // Measurement from sensor (external code should set latest_meas)
            Eigen::VectorXd y = latest_meas[sensor_idx++] - zPred;
            x += K * y;
            P -= K * S * K.transpose();
        }
        sensor_idx = 0;
    }

    // Set the latest measurement for the next update call
    void setMeasurement(const Eigen::VectorXd &z) {
        latest_meas.push_back(z);
    }

    // Get current state estimate
    const StateVec& state() const { return x; }

private:
    StateVec x;
    CovMatrix P;
    Eigen::Matrix<double, N, N> Q;
    double lam, gamma;
    Eigen::VectorXd wm, wc;
    SigmaPoints sigmaPoints;
    std::function<StateVec(const StateVec&, double)> f;
    std::vector<std::pair<MeasFunc, MeasCov>> sensors;
    std::vector<Eigen::VectorXd> latest_meas;
    int sensor_idx = 0;

    void computeWeights() {
        int L = 2 * N + 1;
        wm.resize(L); wc.resize(L);
        wm(0) = lam / (N + lam);
        wc(0) = wm(0) + (1 - 1e-3*1e-3 + 2.0);
        for (int i = 1; i < L; ++i) {
            wm(i) = 1.0 / (2*(N + lam));
            wc(i) = wm(i);
        }
    }
};

#endif // UNSCENTED_KALMAN_H