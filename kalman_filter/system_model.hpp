#pragma once

#include <Eigen/Dense>
#include "types.hpp"
namespace kf {
using NoControl = Vector<0>;

template <typename State, typename Control = NoControl>
struct SystemModelBase {
  using StateType = State;
  using ControlType = Control;
  static constexpr bool kHasControl = !std::is_same_v<Control, NoControl>;
  virtual State f(const State& s, const Control& u) const = 0;
  virtual Transition<State> F() const = 0;
  virtual Covariance<State> Q() const = 0;

};

using NoneControl = void;

}  // namespace kf