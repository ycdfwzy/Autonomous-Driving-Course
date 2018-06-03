// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework3/rotation/rotation.h"

namespace homework3 {

Eigen::Vector3d ToRollPitchYaw(Eigen::Matrix3d rotation) {
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::AngleAxisd ToAngleAxis(Eigen::Matrix3d rotation) {
  return Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
}
}  // namespace homework3
