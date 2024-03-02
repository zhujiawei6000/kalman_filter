#pragma once

#include <Eigen/Dense>
#include "types.hpp"
namespace kf {

template <typename State, typename Control = Vector<0>>
struct SystemModel {
  using StateType = State;
  using ControlType = Control;
  const auto& F() const { return F_; }
  const auto& Q() const { return Q_; }
  const auto& G() const { return G_; }

 protected:
  Transition<State> F_;
  Covariance<State> Q_;
  InputTransition<State, Control> G_;
};
template <typename SystemModel>
inline constexpr bool HasControl =
    std::is_same<SystemModel::ControlType, Vector<0>>::value;

}  // namespace kf