#pragma once
#include <utility/imumaths.h>

namespace Display {
  void init();
  void update(const imu::Vector<3>& euler_deg, double north_m, double east_m);
}