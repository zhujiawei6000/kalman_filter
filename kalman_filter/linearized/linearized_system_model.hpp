#pragma once

#include <Eigen/Dense>

#include <kalman_filter/system_model.hpp>
#include <kalman_filter/types.hpp>

namespace kf {

template <typename State>
class LinearizedSystemModel : public SystemModelBase<State> {
 public:
  using SystemModelBase<State>::StateType;

  void UpdateJacobian(const State& s) { UpdateJacobianImpl(jacobian_, s); }
  Transition<State> F() const override { return jacobian_; }

 protected:
  virtual void UpdateJacobianImpl(Transition<State>& jacobian,
                                  const State& s) = 0;
  Transition<State> jacobian_;
};

}  // namespace kf