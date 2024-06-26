#pragma once

#include <Eigen/Dense>

#include "matrix.hpp"

namespace kf {

template <typename State, typename Measurement>
using KalmanGain = Matrix<State::RowsAtCompileTime, Measurement::RowsAtCompileTime>;
template <typename V>
using Covariance = Matrix<V::RowsAtCompileTime, V::RowsAtCompileTime>;
template <typename A, typename B>
using CrossCovariance = Matrix<A::RowsAtCompileTime, B::RowsAtCompileTime>;
template <typename A, typename B>
using Observation = Matrix<A::RowsAtCompileTime, B::RowsAtCompileTime>;
template <typename V>
using Transition = Matrix<V::RowsAtCompileTime, V::RowsAtCompileTime>;

template <typename A, typename B>
using Jacobian = Matrix<A::RowsAtCompileTime, B::RowsAtCompileTime>;

template <typename State, typename Control>
using InputTransition = Matrix<State::RowsAtCompileTime, Control::RowsAtCompileTime>;

}  // namespace kf