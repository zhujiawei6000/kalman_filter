#pragma once

#include <kalman_filter/extend_kalman_filter.hpp>

namespace kf {
// alias
template <typename SystemModel>
using LinearKalmanFilter = ExtendKalmanFilter<SystemModel>;

}  // namespace kf