#include "IMU.h"
#include "Config.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

namespace {
  Adafruit_BNO055 bno(55, 0x28);
  imu::Vector<3> acc, gyro, euler;
  double bias_ax{}, bias_ay{}, bias_wz{}, mean_theta{}, var_theta{};
}

void IMU::init() {
  if (!bno.begin()) { while (true) { delay(1000); } }
}

void IMU::calibrate() {
  double sx=0, sy=0, so=0, st=0, vst=0;
  for (int i=0;i<Config::CAL_SAMPLES_IMU;++i) {
    acc   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    sx += acc.x(); sy += acc.y();
    so += -gyro.z();                    // yaw rate sign as before
    const double th = euler.x() * Config::DEG2RAD;
    st += th; vst += th*th;
    delay(5);
  }
  bias_ax  = sx / Config::CAL_SAMPLES_IMU;
  bias_ay  = sy / Config::CAL_SAMPLES_IMU;
  bias_wz  = so / Config::CAL_SAMPLES_IMU;
  mean_theta = st / Config::CAL_SAMPLES_IMU;
  var_theta  = vst / Config::CAL_SAMPLES_IMU - mean_theta*mean_theta;
}

Types::IMUFrame IMU::read() {
  acc   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  const double ax_b = acc.x() - bias_ax;
  const double ay_b = acc.y() - bias_ay;
  const double wz   = -gyro.z() - bias_wz;
  const double yaw  = euler.x() * Config::DEG2RAD;

  // rotate body accel to world (r1=r_north, r2=r_east)
  const double c = cos(yaw), s = sin(yaw);
  Types::IMUFrame f;
  f.ax  = (-ax_b * c) + ( ay_b * s);
  f.ay  = ( ax_b * s) + ( ay_b * c);
  f.wz  = wz;
  f.yaw = yaw;
  return f;
}

double IMU::varianceTheta() { return var_theta; }

imu::Vector<3> IMU::lastEuler() { return euler; }