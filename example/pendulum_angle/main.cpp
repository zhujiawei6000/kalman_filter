
#include <iostream>
#include <kalman_filter/extend_kalman_filter.hpp>
#include <queue>

#include "pendulum_angle_system_model.hpp"
#include "pendulum_position_measurement_model.hpp"

using PendulumAngleFilter = kf::ExtendKalmanFilter<PendulumAngleSystemModel>;

std::queue<PendulumPositionMeasurement> GenerateInputs() {
  std::queue<PendulumPositionMeasurement> inputs;
  std::vector<double> positions = {0.119, 0.113, 0.12,   0.101,  0.099,
                                   0.063, 0.008, -0.017, -0.037, -0.05};

  for (int i = 0; i < positions.size(); i++) {
    PendulumPositionMeasurement m;
    m.position() = positions[i];
    inputs.emplace(m);
  }
  return inputs;
}

int main() {
  // initialize filter
  constexpr double kPendulumLength = 0.5;
  PendulumAngleFilter filter;
  PendulumAngleSystemModel system_model{kPendulumLength, 1.0, 0.05};
  PendulumPositionMeasurementModel measurement_model{kPendulumLength, 0.01};
  filter.Init(
      (PendulumAngleState{} << 0.0873, 0).finished(),  // initial state
      PendulumAngleFilter::StateCov::Identity() * 5.0  // initial state cov
  );
  // prepare input of measurements
  std::queue<PendulumPositionMeasurement> inputs{GenerateInputs()};

  // feed measurements to filter
  while (!inputs.empty()) {
    auto m = inputs.front();
    inputs.pop();
    // predict state t+1 conditioning on state t
    filter.Predict(system_model);
    // correct filter with measurement t+1
    auto state = filter.Update(measurement_model, m);
    // output state t+1
    // std::cout << state << "\n";
    std::cout << std::asin(m.position() / kPendulumLength) << ","
              << state.angle() << ","
              << state.angle_speed()
              << std::endl;
  }
}