#pragma once

#include <Eigen/Dense>

#include "measurement_model.hpp"
#include "system_model.hpp"
#include "types.hpp"
#include "utils.hpp"
namespace kf {
template <typename SystemModel>
class UnscentedKalmanFilter {
 public:
  using State = typename SystemModel::StateType;
  using Control = typename SystemModel::ControlType;
  using StateCov = Covariance<State>;
  static constexpr auto kNumSigmaPoints = 2 * SystemModel::kStateNum + 1;
  static constexpr auto kStateNum = SystemModel::kStateNum;
  using SigmaPointsMatrix = Matrix<kStateNum, kNumSigmaPoints>;

  void Init(State x, Covariance<State> p) {
    X_ = std::move(x);
    P_ = std::move(p);
    kappa_ = 3 - kStateNum;
    float w0 = static_cast<float>(kappa_) / (kStateNum + kappa_);
    float w1 = 1.0f / (2.0f * (kStateNum + kappa_));

    sigma_point_weights_ = Vector<kNumSigmaPoints>::Zero();
    sigma_point_weights_(0) = w0;
    for (int i = 1; i < kNumSigmaPoints; ++i) {
      sigma_point_weights_(i) = w1;
    }
  }

  const State& Predict(const SystemModel& sys,
                       const Control& u = Control::Zero()) {
    StateCov factor = (P_ * (kStateNum + kappa_));
    // cholesky decomposition
    factor = factor.llt().matrixL();
    // calculate state sigma points
    std::array<State, kNumSigmaPoints> sigma_points;
    sigma_points[0] = X_;
    for (int i = 0; i < kStateNum; ++i) {
      sigma_points[1 + i] = X_ + factor.col(i);
    }
    for (int i = 0; i < kStateNum; ++i) {
      sigma_points[1 + kStateNum + i] = (X_ - factor.col(i));
    }

    // transfer to the next state
    std::transform(sigma_points.begin(), sigma_points.end(),
                   state_sigma_points_.begin(),
                   [&sys, &u](const State& state) { return sys.f(state, u); });
    state_sigma_matrix_ = HStack(state_sigma_points_);
    // caculate mean and variance after transition
    X_ = state_sigma_matrix_ * sigma_point_weights_;
    P_ = (state_sigma_matrix_.colwise() - X_) * sigma_point_weights_.asDiagonal() *
             (state_sigma_matrix_.colwise() - X_).transpose() +
         sys.Q();
    return GetState();
  }

  template <typename MeasurementModel>
  const State& Update(const MeasurementModel& m,
                      const typename MeasurementModel::MeasurementType& z) {
    // sigma points convert from state space to measurement
    using Measurement = typename MeasurementModel::MeasurementType;
    std::array<Measurement, kNumSigmaPoints> sigma_points;
    std::transform(state_sigma_points_.begin(), state_sigma_points_.end(),
                   sigma_points.begin(),
                   [&m](const State& state) { return m.h(state); });
    auto sigma_matrix = HStack(sigma_points);
    // compute the mean
    Measurement mean = sigma_matrix * sigma_point_weights_;
    // compute the variance
    Covariance<Measurement> cov = (sigma_matrix.colwise() - mean) *
                                      sigma_point_weights_.asDiagonal() *
                                      (sigma_matrix.colwise() - mean).transpose() +
                                  m.R();
    CrossCovariance<State, Measurement> cross_cov =
        (state_sigma_matrix_.colwise() - X_) * sigma_point_weights_.asDiagonal() *
        (sigma_matrix.colwise() - mean).transpose();

    // compute the kalman gain
    KalmanGain<State, Measurement> kalman_gain = cross_cov * cov.inverse();
    // update the state
    X_ += kalman_gain * (z - mean);
    // update the covariance
    P_ -= kalman_gain * cov * kalman_gain.transpose();
    return GetState();
  }
  const State& GetState() const { return X_; }

 protected:
  State X_;
  StateCov P_;
  std::array<State, kNumSigmaPoints> state_sigma_points_;
  Matrix<kStateNum, kNumSigmaPoints> state_sigma_matrix_;
  int kappa_;
  Vector<kNumSigmaPoints> sigma_point_weights_;
};

}  // namespace kf