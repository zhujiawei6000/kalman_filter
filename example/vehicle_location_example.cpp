
#include <iostream>
#include <queue>

#include "kalman_filter.hpp"

struct VehicleLocationMeasurement : kf::Vector<2> {
  KF_VECTOR(VehicleLocationMeasurement, 2)
  enum { POS_X, POS_Y };
  float x() const { return (*this)[POS_X]; };
  float y() const { return (*this)[POS_Y]; };
  float& x() { return (*this)[POS_X]; };
  float& y() { return (*this)[POS_Y]; };
};

struct VehicleLocationState : kf::Vector<6> {
  KF_VECTOR(VehicleLocationState, 6)
  enum { POS_X, VEL_X, ACC_X, POS_Y, VEL_Y, ACC_Y };
  float x() const { return (*this)[POS_X]; };
  float vx() const { return (*this)[VEL_X]; };
  float ax() const { return (*this)[ACC_X]; };
  float y() const { return (*this)[POS_Y]; };
  float vy() const { return (*this)[VEL_Y]; };
  float ay() const { return (*this)[ACC_Y]; };
  friend std::ostream& operator<<(std::ostream& os,
                                  const VehicleLocationState& s) {
    return os << "x:" << s.x() << ", "
              << "vx:" << s.vx() << ", "
              << "ax:" << s.ax() << ", "
              << "y:" << s.y() << ", "
              << "vy:" << s.vy() << ", "
              << "ay:" << s.ay() << ". ";
  }
};

struct VehicleLocationMeasurementModel
    : kf::MeasurementModel<VehicleLocationState, VehicleLocationMeasurement> {
 public:
  using S = VehicleLocationState;
  using M = VehicleLocationMeasurement;
  using Base =
      kf::MeasurementModel<VehicleLocationState, VehicleLocationMeasurement>;
  VehicleLocationMeasurementModel() : Base{} {}
  void Init(float sigma_x, float sigma_y) {
    H_.setZero();
    H_(M::POS_X, S::POS_X) = 1;
    H_(M::POS_Y, S::POS_Y) = 1;
    R_.setZero();
    // set R matrix with measurement SNR
    R_(M::POS_X, M::POS_X) = sigma_x * sigma_x;
    R_(M::POS_Y, M::POS_Y) = sigma_y * sigma_y;
  }
};
class VehicleLocationSystemModel
    : public kf::SystemModel<VehicleLocationState> {
 public:
  using S = VehicleLocationState;
  explicit VehicleLocationSystemModel(float acc_var) : acc_var_{acc_var} {}
  void UpdateProcess(float dt) {
    F_.setIdentity();
    F_(S::POS_X, S::VEL_X) = dt;
    F_(S::POS_X, S::ACC_X) = 0.5 * dt * dt;
    F_(S::VEL_X, S::ACC_X) = dt;
    F_(S::POS_Y, S::VEL_Y) = dt;
    F_(S::POS_Y, S::ACC_Y) = 0.5 * dt * dt;
    F_(S::VEL_Y, S::ACC_Y) = dt;
    Q_.setIdentity();
    Q_.block(0, 0, 3, 3) << std::pow(dt, 4) / 4, std::pow(dt, 3) / 2,
        std::pow(dt, 2) / 2, std::pow(dt, 3) / 2, std::pow(dt, 2), dt,
        std::pow(dt, 2) / 2, dt, 1;
    Q_.block(3, 3, 3, 3) << std::pow(dt, 4) / 4, std::pow(dt, 3) / 2,
        std::pow(dt, 2) / 2, std::pow(dt, 3) / 2, std::pow(dt, 2), dt,
        std::pow(dt, 2) / 2, dt, 1;
    Q_ *= acc_var_;
  }

 private:
  float acc_var_;
};

struct VehicleLocationKalmanFilter
    : kf::KalmanFilter<VehicleLocationSystemModel> {
  void Init(const VehicleLocationState& initialState, float initialNoisy) {
    X_ = initialState;
    P_.setIdentity();
    P_ *= initialNoisy;
  }
};

int main() {
  // The random acceleration standard deviation: σa = 0.2 m/s2
  VehicleLocationSystemModel system_model{
      0.2f,
  };
  VehicleLocationMeasurementModel measurement_model;
  // The measurement error standard deviation: σx = σy = 3m
  measurement_model.Init(3.f, 3.f);
  VehicleLocationKalmanFilter filter;
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

  filter.Init(VehicleLocationState::Zero(), 500.0f);
  while (!measurements.empty()) {
    auto measure = measurements.front();
    measurements.pop();
    // update system model with dt=1.0s
    system_model.UpdateProcess(1.0f);
    // predict N+1 condition on N
    filter.Predict(system_model);
    // estimate N+1 with both measure & prediction
    auto state = filter.Update(measurement_model, measure);
    std::cout << state << std::endl;
  }
}
