#pragma once

#include <Eigen/Dense>

namespace kf {

template <typename State>
struct SystemModel {
  using StateType = State;
  const Transition<State>& F() const { return F_; }
  const Covariance<State>& Q() const { return Q_; }

 protected:
  Transition<State> F_;
  Covariance<State> Q_;
};
}  // namespace kf