#pragma once

#include <Eigen/Dense>

#include "kalman_filter/types.hpp"
#include "kalman_filter/system_model.hpp"
namespace kf {

template <typename State, typename Control = NoControl>
struct LinearSystemModel : SystemModelBase<State, Control> {
  using SystemModelBase<State, Control>::kHasControl;
  using SystemModelBase<State, Control>::F;
  State f(const State& s, const Control& u) const final {
    if constexpr (kHasControl) {
      return F() * s + G() * u;
    } else {
      return F() * s;
    }
  };

  virtual kf::InputTransition<State, Control> G() const {
    return kf::InputTransition<State, Control>{};
  };
};

}  // namespace kf