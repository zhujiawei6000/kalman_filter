#pragma once

#include <kalman_filter/linear/linear_measurement_model.hpp>
#include "rocket_altitude_system_model.hpp"

struct RocketAltitudeMeasurement : kf::Vector<1> {
  KF_VECTOR(RocketAltitudeMeasurement, 1);
  enum { ALTITUDE };
  double altitude() const { return (*this)[ALTITUDE]; }
  double& altitude() { return (*this)[ALTITUDE]; }
};


class RocketAltitudeMeasurementModel
    : public kf::LinearMeasurementModel<RocketAltitudeState,
                                  RocketAltitudeMeasurement> {
 public:
  using S = RocketAltitudeState;
  using M = RocketAltitudeMeasurement;
  using Base =
      kf::LinearMeasurementModel<S, M>;
  RocketAltitudeMeasurementModel(double var) : Base{}, H_{}, R_{} {
    H_ << 1, 0;
    R_ << var;
  }
  kf::Observation<M, S> H() const final { return H_; }
  kf::Covariance<M> R() const final { return R_; }

 private:
  kf::Observation<M, S> H_;
  kf::Covariance<M> R_;

};

