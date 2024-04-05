#pragma once

#include <Eigen/Dense>

#include "kalman_filter/types.hpp"
#include "kalman_filter/system_model.hpp"
namespace kf {

template <typename State, typename Control = NoControl>
struct LinearSystemModel : SystemModelBase<State, Control> {

  State f(const State& s, const Control& u) const final {
    if constexpr (kHasControl) {
      return F() * s + G() * u;
    } else {
      return F() * s;
    }
  };
};

}  // namespace kf