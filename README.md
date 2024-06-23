# Kalman Filter C++ Library
A general-purpose kalman filter library implemented by Jiawei Zhu. It includes all of the variants of the kalman filter such as KF, EKF, UKF and so on. The usage of the kalman filter library is very simple, just following the steps below.

## 1.  Define Your System Model
```c++
class VehicleLocationSystemModel
    : public kf::LinearSystemModel<VehicleLocationState> {
 public:
  kf::Transition<VehicleLocationState> F() const final {
    ...
  }
  kf::Covariance<VehicleLocationState> Q() const final {
    ...
  }

};
```

## 2. Define Your Measurement Model

```c++

struct RadarMeasurementModel
    : kf::LinearizedMeasurementModel<VehicleLocationState, RadarMeasurement> {
  RadarMeasurement h(const VehicleLocationState& s) const final {
    ...
  }
  kf::Covariance<RadarMeasurement> R() const final {
    ...
  }
  
};
```

## 3. Declare Your Filter
```c++
using VehicleLocationFilter =
    kf::UnscentedKalmanFilter<VehicleLocationSystemModel>;
```

## 4. Put It All Together
```c++
  filter.Init(...);
  // feed measurements to filter
  while (!inputs.empty()) {
    auto m = inputs.front();
    inputs.pop();
    // predict state t+1 conditioning on state t
    filter.Predict(system_model);
    // correct filter with measurement t+1
    auto state = filter.Update(measurement_model, m);
    ...
  }
```

That's all you need.