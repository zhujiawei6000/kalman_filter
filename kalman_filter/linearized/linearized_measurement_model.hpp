#pragma once

#include <Eigen/Dense>

#include "kalman_filter/types.hpp"


namespace kf {

template <typename State, typename Measurement>
struct LinearizedMeasurementModel : MeasurementModel<State, Measurement> {
  using MeasurementType = Measurement;
  void UpdateJacobian(const State& s) {
    UpdateJacobianImpl(jacobian_, s);
  }
  Observation<Measurement, State> H() const override {
    return jacobian_;
  }
protected:
  virtual void UpdateJacobianImpl(Observation<Measurement, State>& jacobian, const State& s) = 0;
  Observation<Measurement, State> jacobian_;

};
}  // namespace kf