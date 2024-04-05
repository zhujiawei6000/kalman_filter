#pragma once

#include <Eigen/Dense>
#include "types.hpp"
#include "system_model.hpp"
namespace kf {

template <typename State>
struct LinearizedSystemModel : SystemModelBase<State> {
  using SystemModelBase<State>::StateType;
  virtual void UpdateJacobian(const State& s) = 0;
};

}  // namespace kf