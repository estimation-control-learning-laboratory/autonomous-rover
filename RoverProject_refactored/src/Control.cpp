#include "Control.h"
#include "Config.h"
#include "Estimator.h"
#include "Hardware.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

namespace {
  // Allocation matrix K (as in your original)
  Matrix<double,4,3> K;
  Matrix<double,3,4> B;
  Matrix<double,3,3> B_K;
  Matrix<double, 4,3> sigma;
  Matrix<double, 4,3> K_scaled;
  // PID state
  double pidKp=Config::PID_KP, pidKi=Config::PID_KI, pidKd=Config::PID_KD;
  double pidErr[4]{}, pidLast[4]{}, pidInt[4]{};
  double speedCmd[4]{};
  double pidSetpoint[4]{};
}

void Control::outerLoop(const Types::IMUFrame& imu,
                        double north_meas, double east_meas,
                        unsigned long tSinceStart_ms, double dt) {
  // estimator step
  Estimator::step(imu.ax, imu.ay, imu.wz, imu.yaw, north_meas, east_meas, dt);

  // init K once
  static bool inited=false;
  if (!inited) {
K << -50.0,  50.0,  12.5,
       -50.0, -50.0, -12.5,
       -50.0, -50.0,  12.5,
        50.0, -50.0, -12.5;
B << .015, .015,   .015,  0.015,
      -.015, .015,   0.015, -0.015,
      -0.06,  0.06,  -0.06,  0.06;
sigma <<16.6667,  -16.6667,  -4.1667,
       16.6667, 16.6667, 4.1667,
       16.6667, 16.6667,  -4.1667,
        16.6667, -16.6667, 4.1667;
    //Matrix<double,3,3> 
    K_scaled = K/scaling_Factor;
    B_K = B*K_scaled;
    inited=true;
  }

  const auto& xhat = Estimator::xhat();
  Vector3d xi; xi << xhat(0), xhat(2), xhat(4);

  const double tsec = tSinceStart_ms / 1000.0;
  Vector3d xi_ref;
  xi_ref << -cos(Config::CIRC_RATE_RAD_S * tsec),
             sin(Config::CIRC_RATE_RAD_S * tsec),
            (-(Config::CIRC_RATE_RAD_S * tsec) + (M_PI/2.0));

  // wheel angular velocity targets
  Vector3d U = B_K * (xi - xi_ref);
  Vector4d w = sigma*U;
  speedCmd[0] = w(1);
  speedCmd[1] = w(0);
  speedCmd[2] = w(2);
  speedCmd[3] = w(3);
}

void Control::innerLoopPID() {
  const double dt = Config::INNER_DT_MS / 1000.0;
  for (int i=0;i<4;++i) {
    const long pulses = Hardware::swapEncoderPulses(i);
    const double revs = double(pulses) / (Config::ENCODER_PPR * Config::GEAR_RATIO);
    const double rpm  = (revs / dt) * 60.0;
    const double omega = rpm * (M_PI / 30.0);

    pidErr[i]  = speedCmd[i] - omega;
    pidInt[i] += pidErr[i] * dt;
    const double der = (pidErr[i] - pidLast[i]) / dt;
    pidLast[i] = pidErr[i];

    const double cmd = pidKp*pidErr[i] + pidKi*pidInt[i] + pidKd*der;
    const int    pwm = constrain(int(fabs(cmd)), 0, 255);
    const int    dir = (cmd>0) ? +1 : (cmd<0 ? -1 : 0);

    Hardware::setMotorPWM(i, pwm, dir);
  }
}
