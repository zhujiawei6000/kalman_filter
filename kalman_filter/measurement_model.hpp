#pragma once

#include <Eigen/Dense>

#include "types.hpp"


namespace kf {

template <typename State, typename Measurement>
struct MeasurementModel {
  using MeasurementType = Measurement;
  Observation<Measurement, State> H() const { return H_; }
  Covariance<Measurement> R() const { return R_; }

 protected:
  Observation<Measurement, State> H_;
  Covariance<Measurement> R_;
};
}  // namespace kf