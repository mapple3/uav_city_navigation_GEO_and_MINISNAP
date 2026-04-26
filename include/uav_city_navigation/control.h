#ifndef CONTROL_H
#define CONTROL_H

#include <Eigen/Dense>

class Control {
 public:
  Control(){};
  virtual ~Control(){};
  virtual void Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                      const Eigen::Vector3d &ref_jerk){};
  Eigen::Vector3d getDesiredThrust() { return desired_thrust_; };
  Eigen::Vector3d getDesiredRate() { return desired_rate_; };
  Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_thrust_{Eigen::Vector3d::Zero()};

 protected:
 private:
};

#endif
