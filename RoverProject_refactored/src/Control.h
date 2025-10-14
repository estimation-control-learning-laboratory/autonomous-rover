#pragma once
#include "Types.h"

namespace Control {
  void outerLoop(const Types::IMUFrame& imu,
                 double north_meas, double east_meas,
                 unsigned long tSinceStart_ms, double dt);

  void innerLoopPID(); // reads encoder pulses, drives motors
}