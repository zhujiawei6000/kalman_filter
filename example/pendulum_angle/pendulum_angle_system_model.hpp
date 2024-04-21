#pragma once

#include <kalman_filter/linearized/linearized_system_model.hpp>

struct PendulumAngleState : kf::Vector<2> {
  KF_VECTOR(PendulumAngleState, 2)
  enum { ANGLE, ANGLE_SPEED };
  double angle() const { return (*this)[ANGLE]; }
  double angle_speed() const { return (*this)[ANGLE_SPEED]; }
  double& angle() { return (*this)[ANGLE]; }
  double& angle_speed() { return (*this)[ANGLE_SPEED]; }

  friend std::ostream& operator<<(std::ostream& os, const PendulumAngleState& state) {
    os << "(" << state.angle() << ", " << state.angle_speed() << ")";
    return os;
  }
};

class PendulumAngleSystemModel
    : public kf::LinearizedSystemModel<PendulumAngleState> {
 public:
  PendulumAngleSystemModel(double pendulum_length, double acceleration_variance,
                           double dt)
      : LinearizedSystemModel<PendulumAngleState>{},
        pendulum_length_{pendulum_length},
        acceleration_variance_{acceleration_variance},
        dt_{dt} {}
  void UpdateJacobianImpl(kf::Transition<PendulumAngleState>& jacobian,
                          const PendulumAngleState& s) final {
    jacobian << 1, dt_,
        -kGravitationalAcceleration / pendulum_length_ * std::cos(s.angle()) *
            dt_,
        1;
  }

  PendulumAngleState f(const PendulumAngleState& current_state,
                       const kf::NoControl& u) const {
    PendulumAngleState next_state;
    next_state.angle() =
        current_state.angle() + current_state.angle_speed() * dt_;
    next_state.angle_speed() = current_state.angle_speed() -
                               kGravitationalAcceleration / pendulum_length_ *
                                   std::sin(current_state.angle()) * dt_;
    return next_state;
  }

  kf::Covariance<PendulumAngleState> Q() const {
    kf::Covariance<PendulumAngleState> process_noise;
    process_noise << std::pow(dt_, 4) / 4, std::pow(dt_, 3) / 2,
        std::pow(dt_, 3) / 2, std::pow(dt_, 2);
    process_noise *= acceleration_variance_ * acceleration_variance_;
    return process_noise;
  }

 private:
  double pendulum_length_;
  double acceleration_variance_;
  double dt_;
  static constexpr double kGravitationalAcceleration = 9.78;
};