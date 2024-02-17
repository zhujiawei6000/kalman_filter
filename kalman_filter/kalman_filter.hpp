#pragma once

#include <Eigen/Dense>
#include "types.hpp"
#include "measurement_model.hpp"
#include "system_model.hpp"

namespace kf {
template <typename SystemModel, typename MeasurementModel>
class KalmanFilter {
 public:
  using State = typename SystemModel::StateType;
  const State& Predict(const SystemModel& sys) {
    X_ = sys.F() * X_;
    P_ = sys.F() * P_ * sys.F().transpose() + sys.Q();
    return GetState();
  }
  template <typename MeasurementModel>
  const State& Update(const MeasurementModel& m,
                      const typename MeasurementModel::MeasurementType& z) {
    using Measurement = MeasurementModel::MeasurementType;
    Covariance<Measurement> innovation_cov =
        m.H() * P_ * m.H().transpose() + m.R();
    KalmanGain<State, Measurement> K =
        P_ * m.H().transpose() * innovation_cov.inverse();
    X_ += K * (z - m.H() * X_);
    auto I = Covariance<State>::Identity();
    Covariance<State> factor = I - K * m.H();
    P_ = factor * P_ * factor.transpose() + K * m.R() * K.transpose();
    return GetState();
  }
  const State& GetState() const { return X_; }

 protected:
  State X_;
  Covariance<State> P_;
};

}  // namespace kf