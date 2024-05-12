#pragma once

#include <Eigen/Dense>

#include <kalman_filter/types.hpp>
#include <kalman_filter/measurement_model.hpp>

namespace kf {

template <typename State, typename Measurement>
struct LinearMeasurementModel : MeasurementModel<State, Measurement> {
  using MeasurementType = Measurement;
  using MeasurementModel<State, Measurement>::H;
  Measurement h(const State& s) const override{
    return H() * s;
  }
};
}  // namespace kf