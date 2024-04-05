
#include <iostream>
#include <queue>

#include "kalman_filter/linear_kalman_filter.hpp"
#include "vehicle_location_measurement_model.hpp"
#include "vehicle_location_system_model.hpp"


using VehicleLocationKalmanFilter =
    kf::LinearKalmanFilter<VehicleLocationSystemModel>;

std::queue<VehicleLocationMeasurement> GenerateInputs() {
  // simulate test measurements
  std::queue<VehicleLocationMeasurement> measurements;
  std::vector<double> input_x{
      {301.5,  298.23, 297.83, 300.42, 301.94, 299.5,  305.98, 301.25, 299.73,
       299.2,  298.62, 301.84, 299.6,  295.3,  299.3,  301.95, 296.3,  295.11,
       295.12, 289.9,  283.51, 276.42, 264.22, 250.25, 236.66, 217.47, 199.75,
       179.7,  160,    140.92, 113.53, 93.68,  69.71,  45.93,  20.87}};

  std::vector<double> input_y{
      {-401.46, -375.44, -346.15, -320.2,  -300.08, -274.12, -253.45,
       -226.4,  -200.65, -171.62, -152.11, -125.19, -93.4,   -74.79,
       -49.12,  -28.73,  2.99,    25.65,   49.86,   72.87,   96.34,
       120.4,   144.69,  168.06,  184.99,  205.11,  221.82,  238.3,
       253.02,  267.19,  270.71,  285.86,  288.48,  292.9,   298.77}};

  //   measurements.reserve(input_x.size());
  for (int i = 0; i < input_x.size(); ++i) {
    VehicleLocationMeasurement m;
    m.x() = input_x[i];
    m.y() = input_y[i];
    measurements.emplace(m);
  }
  return measurements;
}

int main() {
  // The random acceleration standard deviation: σa = 0.2 m/s2
  VehicleLocationSystemModel system_model{
      0.2,  // acc std
      1.0   // dt
  };
  // The measurement error standard deviation: σx = σy = 3m
  VehicleLocationMeasurementModel measurement_model{3.0, 3.0};
  VehicleLocationKalmanFilter filter;
  filter.Init(VehicleLocationState::Zero(),
              VehicleLocationKalmanFilter::StateCov::Identity() *
                  500.0  // initial state cov
  );
  // prepare input of measurements
  std::queue<VehicleLocationMeasurement> inputs{GenerateInputs()};

  while (!inputs.empty()) {
    auto measure = inputs.front();
    inputs.pop();
    // predict N+1 condition on N
    filter.Predict(system_model);
    // estimate N+1 with both measure & prediction
    auto state = filter.Update(measurement_model, measure);
    // std::cout << state << std::endl;
    std::cout << measure.x() << "," << measure.y() << "," << state.x() << ","
              << state.y() << "," << state.vx() << "," << state.vy()
              << std::endl;
  }

  return 0;
}
