#pragma once

#include <Eigen/Dense>
#include "types.hpp"
#include "measurement_model.hpp"
#include "linearized/linearized_system_model.hpp"
#include "linearized/linearized_measurement_model.hpp"
#include "system_model.hpp"

namespace kf {
template <typename SystemModel>
class ExtendKalmanFilter {
 public:
  using State = typename SystemModel::StateType;
  using Control = typename SystemModel::ControlType;
  using StateCov = Covariance<State>;
  void Init(State x, Covariance<State> p) {
    X_ = std::move(x);
    P_ = std::move(p);
  }
  
  const State& Predict(const SystemModel& sys, const Control& u = Control::Zero()) {
    if constexpr (std::is_base_of_v<LinearizedSystemModel<State>, SystemModel>) {
      const_cast<SystemModel&>(sys).UpdateJacobian(X_);
    }
    X_ = sys.f(X_, u);
    const auto& F = sys.F();
    P_ = F * P_ * F.transpose() + sys.Q();
    return GetState();
  }

  template <typename MeasurementModel>
  const State& Update(const MeasurementModel& m,
                      const typename MeasurementModel::MeasurementType& z) {
    using Measurement = typename MeasurementModel::MeasurementType;
    if constexpr (std::is_base_of<LinearizedMeasurementModel<State, MeasurementModel::MeasurementType>, MeasurementModel>::value) {
      const_cast<MeasurementModel&>(m).UpdateJacobian(X_);
    }
    auto& H = m.H();
    Covariance<Measurement> innovation_cov =
        H * P_ * H.transpose() + m.R();
    KalmanGain<State, Measurement> K =
        P_ * H.transpose() * innovation_cov.inverse();
    X_ += K * (z - m.h(X_));
    auto I = Covariance<State>::Identity();
    Covariance<State> factor = I - K * H;
    P_ = factor * P_ * factor.transpose() + K * m.R() * K.transpose();
    return GetState();
  }
  const State& GetState() const { return X_; }

 protected:
  State X_;
  StateCov P_;
};

// alias
template <typename SystemModel>
using LinearKalmanFilter = ExtendKalmanFilter<SystemModel>;

}  // namespace kf