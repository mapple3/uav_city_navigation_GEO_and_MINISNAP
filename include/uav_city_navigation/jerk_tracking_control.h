#ifndef JERK_TRACKING_CONTROL_H
#define JERK_TRACKING_CONTROL_H

#include "uav_city_navigation/common.h"
#include "uav_city_navigation/control.h"

class JerkTrackingControl : public Control {
 public:
  JerkTrackingControl();
  virtual ~JerkTrackingControl();
  void Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
              const Eigen::Vector3d &ref_jerk) override;

 private:
  Eigen::Vector3d last_ref_acc_{Eigen::Vector3d::Zero()};
};

#endif
