#pragma once
#include <Arduino.h>
#include <ArduinoEigenDense.h>

namespace Logging {
  void initUSB();
  void openLogs();
  void writeOpenClosed(const Eigen::Matrix<double,5,1>& x,
                       const Eigen::Matrix<double,5,1>& xhat);
}