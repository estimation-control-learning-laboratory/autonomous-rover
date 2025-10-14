#pragma once
#include <Arduino.h>

namespace Config {
  // Timing (ms)
  constexpr unsigned long OUTER_DT_MS   = 30;
  constexpr unsigned long INNER_DT_MS   = 20;
  constexpr unsigned long DISPLAY_DT_MS = 200;

  // GPS serial
  constexpr uint32_t GPS_BAUD = 38400;

  // Geometry / conversions
  constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;

  // Controller (outer circle ref)
  constexpr double CIRC_RATE_RAD_S = 0.01;

  // Encoders / motors
  constexpr int ENCODER_PPR  = 12;
  constexpr int GEAR_RATIO   = 120;

  // PID (inner)
  constexpr double PID_KP = 0.6;
  constexpr double PID_KI = 0.2;
  constexpr double PID_KD = 0.0;

  // Calibration samples
  constexpr int CAL_SAMPLES_IMU = 1000;
  constexpr int CAL_SAMPLES_GPS = 50;

  // Display colors (GIGA)
  constexpr uint16_t BLACK = 0x0000;
  constexpr uint16_t WHITE = 0xFFFF;
}