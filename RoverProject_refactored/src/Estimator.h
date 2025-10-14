#pragma once
#include <ArduinoEigenDense.h>

namespace Estimator {
  void init(double varNorth, double varEast, double varTheta);
  void step(double r1dd, double r2dd, double wz, double yaw,
            double north_meas, double east_meas, double dt);
  const Eigen::Matrix<double,5,1>& x();
  const Eigen::Matrix<double,5,1>& xhat();
}