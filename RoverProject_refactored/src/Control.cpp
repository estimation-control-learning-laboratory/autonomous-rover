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
  Matrix<double,3,3> K_diag;
  Matrix<double, 4,3> sigma;
  Matrix<double, 3,3> U;
  // PID state
  //double pidKp=Config::PID_KP, pidKi=Config::PID_KI, pidKd=Config::PID_KD;
  double pidKp, pidKi, pidKd; // individual PID gains
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

    // These Matrices were computed in Matlab
K_diag << -25.0,  0.0,  0.0,
               0.0, -25.0,  0.0,
               0.0,  0.0, -25.0;
    
B << .015, .015,   .015,  0.015,
      -.015, .015,   0.015, -0.015,
      -0.06,  0.06,  -0.06,  0.06;
    
sigma <<16.6667,  -16.6667,  -4.1667,
       16.6667, 16.6667, 4.1667,
       16.6667, 16.6667,  -4.1667,
        16.6667, -16.6667, 4.1667;

    inited=true;
  }

  const auto& xhat = Estimator::xhat();
  Vector3d xi; xi << xhat(0), xhat(2), xhat(4);

  const double tsec = tSinceStart_ms / 1000.0;
  Vector3d xi_ref;
  xi_ref << -cos(Config::CIRC_RATE_RAD_S * tsec),
             sin(Config::CIRC_RATE_RAD_S * tsec),
            (-(Config::CIRC_RATE_RAD_S * tsec) + (M_PI/2.0));

  Vector3d error_vec = xi - xi_ref;
  error_vec(2) = atan2(sin(error_vec(2)), cos(error_vec(2))); //normalize the heading angle, before when it went from 0 to 359, it saw it as a big error even if the angles were close.
  Vector4d w; //hold the target wheel speeds
    
  // set a tolerance to stop the rover when it gets within that tolerance
  bool at_position = (abs(error_vec(0)) < Config::POSITION_TOLERANCE_M) &&
                     (abs(error_vec(1)) < Config::POSITION_TOLERANCE_M);
                      
  bool at_heading  = (abs(error_vec(2)) < Config::HEADING_TOLERANCE_RAD);

if (at_position && at_heading) {
    w.setZero();
  } else {
    // if outside the tolerance, keep moving
    Vector3d U_vel = K_diag * error_vec;
    w = sigma * U_vel; // This is the fixed control law
  }
  speedCmd[0] = w(1);
  speedCmd[1] = w(0);
  speedCmd[2] = w(2);
  speedCmd[3] = w(3);
}

void Control::innerLoopPID() {
  const double dt = Config::INNER_DT_MS / 1000.0;
  for (int i=0;i<4;++i) {
    pidKp = Config::PID_KP[i];
    pidKi = Config::PID_KI[i];
    pidKd = Config::PID_KD[i];
    const long pulses = Hardware::swapEncoderPulses(i);
    const double revs = double(pulses) / (Config::ENCODER_PPR * Config::GEAR_RATIO);
    const double rpm  = (revs / dt) * 60.0;
    const double omega = rpm * (M_PI / 30.0);

    pidErr[i]  = speedCmd[i] - omega;
    pidInt[i] += pidErr[i] * dt;
    const double der = (pidErr[i] - pidLast[i]) / dt;
    pidLast[i] = pidErr[i];

    const double cmd = pidKp*pidErr[i] + pidKi*pidInt[i] + pidKd*der;
    const int    pwm = constrain(int(fabs(cmd)), 0, 150); // can cahnge the PWM to saturate the max wheel speed
    const int    dir = (cmd>0) ? +1 : (cmd<0 ? -1 : 0);

    Hardware::setMotorPWM(i, pwm, dir);
  }
}
