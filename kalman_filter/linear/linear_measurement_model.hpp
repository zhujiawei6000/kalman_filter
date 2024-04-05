#pragma once

#include <Eigen/Dense>

#include <kalman_filter/types.hpp>


namespace kf {

template <typename State, typename Measurement>
struct LinearMeasurementModel : MeasurementModel<State, Measurement> {
  using MeasurementType = Measurement;
  Measurement h(const State& s) const override{
    return H() * s;
  }
};
}  // namespace kf