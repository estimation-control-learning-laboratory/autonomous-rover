#pragma once
#include "Types.h"
#include <utility/imumaths.h>

namespace IMU {
  void init();
  void calibrate();
  Types::IMUFrame read();        // returns bias-corrected ax, ay, wz (world-projected) + yaw
  double varianceTheta();
  imu::Vector<3> lastEuler();    // for display
}