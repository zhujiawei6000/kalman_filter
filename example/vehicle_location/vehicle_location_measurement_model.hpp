#pragma once
#include "kalman_filter/linear/linear_measurement_model.hpp"
#include "vehicle_location_system_model.hpp"

struct VehicleLocationMeasurement : kf::Vector<2> {
  KF_VECTOR(VehicleLocationMeasurement, 2)
  enum { POS_X, POS_Y };
  double x() const { return (*this)[POS_X]; };
  double y() const { return (*this)[POS_Y]; };
  double& x() { return (*this)[POS_X]; };
  double& y() { return (*this)[POS_Y]; };
};

struct VehicleLocationMeasurementModel
    : kf::LinearMeasurementModel<VehicleLocationState,
                                 VehicleLocationMeasurement> {
 public:
  using S = VehicleLocationState;
  using M = VehicleLocationMeasurement;
  using Base =
      kf::LinearMeasurementModel<S, M>;
  VehicleLocationMeasurementModel(double sigma_x, double sigma_y) : Base{}, H_{}, R_{} {
    H_.setZero();
    H_(M::POS_X, S::POS_X) = 1;
    H_(M::POS_Y, S::POS_Y) = 1;
    R_.setZero();
    // set R matrix with measurement SNR
    R_(M::POS_X, M::POS_X) = sigma_x * sigma_x;
    R_(M::POS_Y, M::POS_Y) = sigma_y * sigma_y;
  }
  kf::Observation<M, S> H() const final { return H_; }
  kf::Covariance<M> R() const final { return R_; }

 private:
  kf::Observation<M, S> H_;
  kf::Covariance<M> R_;
};