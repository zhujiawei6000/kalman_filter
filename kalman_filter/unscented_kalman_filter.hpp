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
  template<class Type>
  using SigmaPoints = Matrix<Type::RowsAtCompileTime, kNumSigmaPoints>;

  void Init(State x, Covariance<State> p) {
    X_ = std::move(x);
    P_ = std::move(p);
    kappa_ = static_cast<int>(3 - kStateNum);
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
    SigmaPoints<State> sigma_points;
    sigma_points.col(0) = X_;
    for (int i = 0; i < kStateNum; ++i) {
      sigma_points.col(1 + i) = X_ + factor.col(i);
    }
    for (int i = 0; i < kStateNum; ++i) {
      sigma_points.col(1 + kStateNum + i) = (X_ - factor.col(i));
    }

    // transfer to the next state
    for(int i = 0; i < kNumSigmaPoints; ++i) {
      state_sigma_points_.col(i) = sys.f(sigma_points.col(i), u);
    }

    // caculate mean and variance after transition
    X_ = state_sigma_points_ * sigma_point_weights_;
    P_ = (state_sigma_points_.colwise() - X_) * sigma_point_weights_.asDiagonal() *
             (state_sigma_points_.colwise() - X_).transpose() +
         sys.Q();
    return GetState();
  }

  template <typename MeasurementModel>
  const State& Update(const MeasurementModel& m,
                      const typename MeasurementModel::MeasurementType& z) {
    // sigma points convert from state space to measurement
    using Measurement = typename MeasurementModel::MeasurementType;
    SigmaPoints<Measurement> sigma_points;
    for(int i = 0; i < kNumSigmaPoints; ++i) {
      sigma_points.col(i) = m.h(state_sigma_points_.col(i));
    }
    
    // compute the mean
    Measurement mean = sigma_points * sigma_point_weights_;
    // compute the variance
    Covariance<Measurement> cov = (sigma_points.colwise() - mean) *
                                      sigma_point_weights_.asDiagonal() *
                                      (sigma_points.colwise() - mean).transpose() +
                                  m.R();
    CrossCovariance<State, Measurement> cross_cov =
        (state_sigma_points_.colwise() - X_) * sigma_point_weights_.asDiagonal() *
        (sigma_points.colwise() - mean).transpose();

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
  SigmaPoints<State> state_sigma_points_;
  int kappa_;
  Vector<kNumSigmaPoints> sigma_point_weights_;
};

}  // namespace kf