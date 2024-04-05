
#include <queue>

#include "kalman_filter/extend_kalman_filter.hpp"
#include "radar_measurement_model.hpp"
#include "vehicle_location_system_model.hpp"
#include <iostream>
using VehicleLocationFilter =
    kf::ExtendKalmanFilter<VehicleLocationSystemModel>;

std::queue<RadarMeasurement> GenerateInputs() {
  std::queue<RadarMeasurement> inputs;

  std::vector<double> input_ranges = {
      502.55, 477.34, 457.21, 442.94, 427.27, 406.05, 400.73, 377.32, 360.27,
      345.93, 333.34, 328.07, 315.48, 301.41, 302.87, 304.25, 294.46, 294.29,
      299.38, 299.37, 300.68, 304.1,  301.96, 300.3,  301.9,  296.7,  297.07,
      295.29, 296.31, 300.62, 292.3,  298.11, 298.07, 298.92, 298.04};
  std::vector<double> input_angles = {
      -0.9316, -0.8977, -0.8512, -0.8114, -0.7853, -0.7392, -0.7052,
      -0.6478, -0.59,   -0.5183, -0.4698, -0.3952, -0.3026, -0.2445,
      -0.1626, -0.0937, 0.0085,  0.0856,  0.1675,  0.2467,  0.329,
      0.4149,  0.504,   0.5934,  0.667,   0.7537,  0.8354,  0.9195,
      1.0039,  1.0923,  1.1546,  1.2564,  1.3274,  1.409,   1.5011};
  for (int i = 0; i < input_ranges.size(); i++) {
    RadarMeasurement m;
    m.range() = input_ranges[i];
    m.angle() = input_angles[i];
    inputs.emplace(m);
  }
  return inputs;
}

int main() {
  // initialize filter
  VehicleLocationFilter filter;
  VehicleLocationSystemModel system_model{
      0.2,  // acc std
      1.0   // dt
  };
  RadarMeasurementModel measurement_model{
      5.0,    // range std
      0.0087  // angle std
  };
  filter.Init(
      (VehicleLocationState{} << 400, 0, 0, -300, 0, 0)
          .finished(),                                     // initial state
      VehicleLocationFilter::StateCov::Identity() * 500.0  // initial state cov
  );
  // prepare input of measurements
  std::queue<RadarMeasurement> inputs{GenerateInputs()};

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
    std::cout << std::cos(m.angle()) * m.range() << "," << std::sin(m.angle()) * m.range() << ","
              << state.x() << "," << state.y() << ","
              << state.vx() << "," << state.vy() << std::endl;
  }
}