#pragma once

#include <Eigen/Dense>

#include "types.hpp"


namespace kf {

template <typename State, typename Measurement>
struct MeasurementModel {
  using MeasurementType = Measurement;
  virtual Measurement h(const State& s) const = 0;
  virtual Observation<Measurement, State> H() const = 0;
  virtual Covariance<Measurement> R() const = 0;

};
}  // namespace kf