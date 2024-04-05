#pragma once
#include "kalman_filter/linearized/linearized_measurement_model.hpp"
#include "vehicle_location_system_model.hpp"
#include <cmath>

struct RadarMeasurement : kf::Vector<2> {
  KF_VECTOR(RadarMeasurement, 2)
  enum { RANGE, ANGLE };
  double range() const { return (*this)[RANGE]; };
  double angle() const { return (*this)[ANGLE]; };
  double& range() { return (*this)[RANGE]; };
  double& angle() { return (*this)[ANGLE]; };
};

struct RadarMeasurementModel
    : kf::LinearizedMeasurementModel<VehicleLocationState, RadarMeasurement> {
  RadarMeasurementModel(double sigma_range, double sigma_angle)
      : sigma_range_{sigma_range}, sigma_angle_{sigma_angle} {}
  RadarMeasurement h(const VehicleLocationState& s) const final {
    RadarMeasurement m;
    m.range() = std::sqrt(s.x() * s.x() + s.y() * s.y());
    m.angle() = atan2(s.y(), s.x());
    return m;
  }
  kf::Covariance<RadarMeasurement> R() const final {
    return (kf::Covariance<RadarMeasurement>{} << sigma_range_ * sigma_range_, 0, 0, sigma_angle_ * sigma_angle_).finished();
  }
  void UpdateJacobianImpl(kf::Observation<RadarMeasurement, VehicleLocationState>& jacobian,
                          const VehicleLocationState& s) final {
    double x_2_y_2 = s.x() * s.x() + s.y() * s.y();
    double sqrt_x_2_y_2 = std::sqrt(x_2_y_2);
    jacobian << s.x() / sqrt_x_2_y_2, 0, 0, s.y() / sqrt_x_2_y_2, 0, 0,
        -s.y() / x_2_y_2, 0, 0, s.x() / x_2_y_2, 0, 0;
  }

 private:
  double sigma_range_;
  double sigma_angle_;
};