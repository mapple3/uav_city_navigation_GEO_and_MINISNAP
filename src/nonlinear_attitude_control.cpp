#include "uav_city_navigation/nonlinear_attitude_control.h"

NonlinearAttitudeControl::NonlinearAttitudeControl(double attctrl_tau) : Control() { attctrl_tau_ = attctrl_tau; }

NonlinearAttitudeControl::~NonlinearAttitudeControl() {}

void NonlinearAttitudeControl::Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                      const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
  // Geometric attitude controller
  // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
  // attitude control: Technical report. ETH Zurich, 2013.

  const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
  const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
  const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);
  desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  const Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att);
  const Eigen::Vector3d zb = rotmat.col(2);
  desired_thrust_(0) = 0.0;
  desired_thrust_(1) = 0.0;
  desired_thrust_(2) = ref_acc.dot(zb);
}
