#include <iostream>
#include <map>
#include <queue>

#include <kalman_filter/linear_kalman_filter.hpp>
#include "rocket_altitude_measurement_model.hpp"
#include "rocket_altitude_system_model.hpp"



using RocketAltitudeKalmanFilter = kf::LinearKalmanFilter<RocketAltitudeSystemModel>;
// class RocketAltitudeKalmanFilter
//     : public kf::KalmanFilter<RocketAltitudeSystemModel> {
//  public:
//   void Init(const RocketAltitudeState& state, double var) {
//     X_ = state;
//     P_ << var, 0, 0, var;
//   }
// };

std::queue<std::pair<RocketAltitudeMeasurement, RocketAltitudeControl>>
GetInputs() {
  std::queue<std::pair<RocketAltitudeMeasurement, RocketAltitudeControl>> q;
  std::vector<double> h{6.43,   1.3,    39.43,  45.89,  41.44,  48.7,
                        78.06,  80.08,  61.77,  75.15,  110.39, 127.83,
                        158.75, 156.55, 213.32, 229.82, 262.8,  297.57,
                        335.69, 367.92, 377.19, 411.18, 460.7,  468.39,
                        553.9,  583.97, 655.15, 723.09, 736.85, 787.22};
  std::vector<double> a{39.81, 39.67, 39.81, 39.84, 40.05, 39.85, 39.78, 39.65,
                        39.67, 39.78, 39.59, 39.87, 39.85, 39.59, 39.84, 39.9,
                        39.63, 39.59, 39.76, 39.79, 39.73, 39.93, 39.83, 39.85,
                        39.94, 39.86, 39.76, 39.86, 39.74, 39.94};
  assert(h.size() == a.size());
  constexpr double G = 9.8;
  for (size_t i = 0u; i < h.size(); i++) {
    RocketAltitudeMeasurement m;
    RocketAltitudeControl c;
    m.altitude() = h[i];
    c.acc() = a[i] - G;
    q.emplace(m, c);
  }
  return q;
}

int main() {
  std::queue<std::pair<RocketAltitudeMeasurement, RocketAltitudeControl>>
      measurements_and_controls{GetInputs()};
  RocketAltitudeKalmanFilter filter;
  RocketAltitudeMeasurementModel msmodel{
    400.0
  };
  RocketAltitudeSystemModel sysmodel{
    0.01,
    0.25
  };
  filter.Init(RocketAltitudeState::Zero(), 
  RocketAltitudeKalmanFilter::StateCov::Identity() * 500.0);
  RocketAltitudeControl last_u = RocketAltitudeControl::Zero();
  while (!measurements_and_controls.empty()) {
    auto [m, u] = measurements_and_controls.front();
    measurements_and_controls.pop();
    filter.Predict(sysmodel, last_u);
    const auto& state = filter.Update(msmodel, m);
    last_u = u;
    std::cout << m.altitude() << "," << state.altitude() << "," << state.velocity() << std::endl;
  }
}