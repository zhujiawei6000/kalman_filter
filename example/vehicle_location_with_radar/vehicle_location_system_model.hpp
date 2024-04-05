#pragma once
#include "kalman_filter/linear/linear_system_model.hpp"

struct VehicleLocationState : kf::Vector<6> {
  KF_VECTOR(VehicleLocationState, 6)
  enum { POS_X, VEL_X, ACC_X, POS_Y, VEL_Y, ACC_Y };
  double x() const { return (*this)[POS_X]; };
  double vx() const { return (*this)[VEL_X]; };
  double ax() const { return (*this)[ACC_X]; };
  double y() const { return (*this)[POS_Y]; };
  double vy() const { return (*this)[VEL_Y]; };
  double ay() const { return (*this)[ACC_Y]; };
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

class VehicleLocationSystemModel
    : public kf::LinearSystemModel<VehicleLocationState> {
 public:
  using S = VehicleLocationState;
  explicit VehicleLocationSystemModel(double acc_var, double dt)
      : acc_var_{acc_var}, dt_(dt) {}
  void UpdateDeltaTime(double dt) { dt_ = dt; }
  kf::Transition<VehicleLocationState> F() const final {
    kf::Transition<VehicleLocationState> F;
    F.setIdentity();
    F(S::POS_X, S::VEL_X) = dt_;
    F(S::POS_X, S::ACC_X) = 0.5 * dt_ * dt_;
    F(S::VEL_X, S::ACC_X) = dt_;
    F(S::POS_Y, S::VEL_Y) = dt_;
    F(S::POS_Y, S::ACC_Y) = 0.5 * dt_ * dt_;
    F(S::VEL_Y, S::ACC_Y) = dt_;
    return F;
  }
  kf::Covariance<VehicleLocationState> Q() const final {
    kf::Covariance<VehicleLocationState> Q;
    Q.setIdentity();
    Q.block(0, 0, 3, 3) << std::pow(dt_, 4) / 4, std::pow(dt_, 3) / 2,
        std::pow(dt_, 2) / 2, std::pow(dt_, 3) / 2, std::pow(dt_, 2), dt_,
        std::pow(dt_, 2) / 2, dt_, 1;
    Q.block(3, 3, 3, 3) << std::pow(dt_, 4) / 4, std::pow(dt_, 3) / 2,
        std::pow(dt_, 2) / 2, std::pow(dt_, 3) / 2, std::pow(dt_, 2), dt_,
        std::pow(dt_, 2) / 2, dt_, 1;
    Q *= acc_var_;
    return Q;
  }

 private:
  double acc_var_;
  double dt_;
};
