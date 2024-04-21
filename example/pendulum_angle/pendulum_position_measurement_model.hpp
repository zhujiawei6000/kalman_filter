#pragma once

#include <kalman_filter/linearized/linearized_measurement_model.hpp>

#include "pendulum_angle_system_model.hpp"

struct PendulumPositionMeasurement : kf::Vector<1> {
  KF_VECTOR(PendulumPositionMeasurement, 1)
  enum { POSITION };
  double position() const { return (*this)[POSITION]; }
  double& position() { return (*this)[POSITION]; }
};

class PendulumPositionMeasurementModel
    : public kf::LinearizedMeasurementModel<PendulumAngleState,
                                            PendulumPositionMeasurement> {
 public:
  using State = PendulumAngleState;
  using Measurement = PendulumPositionMeasurement;
  using Base = kf::LinearizedMeasurementModel<PendulumAngleState,
                                              PendulumPositionMeasurement>;
  PendulumPositionMeasurementModel(double pendulum_length,
                                   double measurement_std)
      : Base{},
        pendulum_length_{pendulum_length},
        measurement_std_{measurement_std} {}
  Measurement h(const State& s) const final {
    Measurement converted_state;
    converted_state.position() = std::sin(s.angle()) * pendulum_length_;
    return converted_state;
  }

  kf::Covariance<Measurement> R() const final {
    kf::Covariance<Measurement> cov;
    cov << measurement_std_ * measurement_std_;
    return cov;
  }
  void UpdateJacobianImpl(kf::Observation<Measurement, State>& jacobian,
                          const State& s) final {
    jacobian << pendulum_length_ * std::cos(s.angle()), 0;
  }

 private:
  double pendulum_length_;
  double measurement_std_;
};