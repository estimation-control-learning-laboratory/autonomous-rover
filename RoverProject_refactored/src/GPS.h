#pragma once
#include <Arduino.h>

namespace GPS {
  void init();
  void calibrate();
  void update();
  double north();     // corrected (bias removed)
  double east();
  double varianceNorth();
  double varianceEast();
}