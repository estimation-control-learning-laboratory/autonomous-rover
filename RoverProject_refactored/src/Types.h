#pragma once
#include <Arduino.h>

namespace Types {
  struct ECEF { double x{}, y{}, z{}; };
  struct NED  { double n{}, e{}, d{}; };

  struct IMUFrame {
    double ax{};     // world-projected longitudinal accel (bias-corrected)
    double ay{};     // world-projected lateral accel (bias-corrected)
    double wz{};     // yaw rate (bias-corrected)
    double yaw{};    // radians
  };
}