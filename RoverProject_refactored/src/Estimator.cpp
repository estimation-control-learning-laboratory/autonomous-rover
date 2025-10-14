#include "Estimator.h"
using namespace Eigen;

namespace {
  Matrix<double,5,5> A, Ad, I5, Pk, Pk1, Qk;
  Matrix<double,5,3> Bx, Bd;
  Matrix<double,3,5> C;
  Matrix<double,3,3> Rk, Rbar;
  Matrix<double,5,3> Kest;
  Matrix<double,5,1> xk, xhatk;
}

void Estimator::init(double varN, double varE, double varTheta) {
  I5.setIdentity();
  A.setZero(); Bx.setZero(); C.setZero();

  // x = [r1, r1dot, r2, r2dot, theta]
  A(0,1) = 1.0; A(2,3) = 1.0;
  Bx(1,0) = 1.0; Bx(3,1) = 1.0; Bx(4,2) = 1.0;
  C(0,0) = 1.0; C(1,2) = 1.0; C(2,4) = 1.0;

  Qk = 0.1 * I5;
  Rk.setZero(); Rk(0,0)=varN; Rk(1,1)=varE; Rk(2,2)=varTheta;

  Pk = 100.0 * I5;
  xhatk.setZero(); xk.setZero();
  xhatk(0) = -1.0; // initial condition from original
}

void Estimator::step(double r1dd, double r2dd, double wz, double yaw,
                     double north_meas, double east_meas, double dt) {
  Ad = I5 + A * dt;
  Bd = Bx * dt;

  Vector3d u; u << r1dd, r2dd, wz;
  xk = Ad * xhatk + Bd * u;

  // covariance predict
  Pk1 = Ad * Pk * Ad.transpose() + Qk;

  // measurement update
  Vector3d y; y << north_meas, east_meas, yaw;
  Rbar = C * Pk1 * C.transpose() + Rk;
  Kest = Pk1 * C.transpose() * Rbar.inverse();
  xhatk = xk + Kest * (y - C * xk);
  Pk    = Pk1 - Kest * C * Pk1;
}

const Matrix<double,5,1>& Estimator::x()    { return xk; }
const Matrix<double,5,1>& Estimator::xhat() { return xhatk; }