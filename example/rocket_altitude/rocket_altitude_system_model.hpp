#pragma once

#include <kalman_filter/linear/linear_system_model.hpp>

struct RocketAltitudeState : kf::Vector<2> {
  KF_VECTOR(RocketAltitudeState, 2);
  enum { ALTITUDE, VELOCITY };
  double altitude() const { return (*this)[ALTITUDE]; }
  double velocity() const { return (*this)[VELOCITY]; }
  double& altitude() { return (*this)[ALTITUDE]; }
  double& velocity() { return (*this)[VELOCITY]; }
  friend std::ostream& operator<<(std::ostream& os,
                                  const RocketAltitudeState& s) {
    return os << "altitude: " << s.altitude() << ", "
              << "velocity: " << s.velocity() << ".";
  }
};

struct RocketAltitudeControl : kf::Vector<1> {
  KF_VECTOR(RocketAltitudeControl, 1);
  enum { ACC };
  double acc() const { return (*this)[ACC]; }
  double& acc() { return (*this)[ACC]; }
};


class RocketAltitudeSystemModel
    : public kf::LinearSystemModel<RocketAltitudeState, RocketAltitudeControl> {
 public:

  using S = RocketAltitudeState;
  using C = RocketAltitudeControl;
  explicit RocketAltitudeSystemModel(double acc_var, double dt)
      : acc_var_{acc_var}, dt_(dt) {}

  void UpdateDeltaTime(double dt) { dt_ = dt; }
  kf::Transition<S> F() const final {
    kf::Transition<S> F;
    F << 1, dt_, 0, 1;
    return F;
  }

  kf::Covariance<S> Q() const final {
    kf::Covariance<S> Q;
    Q << std::pow(dt_, 4) / 4 * acc_var_, std::pow(dt_, 3) / 2 * acc_var_,
        std::pow(dt_, 3) / 2 * acc_var_, dt_ * dt_ * acc_var_;
    return Q;
  }

  kf::InputTransition<S, C> G() const final {
    kf::InputTransition<S, C> G;
    G << 0.5 * dt_ * dt_, dt_;
    return G;
  }
 private:
  double acc_var_;
  double dt_;

};
