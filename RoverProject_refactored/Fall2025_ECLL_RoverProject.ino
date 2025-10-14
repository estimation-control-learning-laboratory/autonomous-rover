#include "src/Config.h"
#include "src/Hardware.h"
#include "src/Logging.h"
#include "src/IMU.h"
#include "src/GPS.h"
#include "src/Estimator.h"
#include "src/Control.h"
#include "src/Display.h"

static unsigned long tStart, tPrev, tOuter, tPID, tDisp;

void setup() {
  Serial.begin(115200);

  Hardware::initPins();
  Logging::initUSB();
  Logging::openLogs();

  IMU::init();
  IMU::calibrate();

  GPS::init();
  GPS::calibrate();

  Estimator::init(
    GPS::varianceNorth(), GPS::varianceEast(), IMU::varianceTheta()
  );

  Display::init();

  tStart = tPrev = tOuter = tPID = tDisp = millis();
}

void loop() {
  const unsigned long t = millis();

  if (t - tOuter >= Config::OUTER_DT_MS) {
    const double dt = (t - tPrev) / 1000.0; tPrev = t;

    // sensor updates
    const auto imu  = IMU::read();   // bias-corrected ax, ay, wz, yaw
    GPS::update();                   // updates corrected N/E internally

    // estimation + ref + wheel allocation
    Control::outerLoop(imu, GPS::north(), GPS::east(), t - tStart, dt);

    // logging (x, xhat)
    Logging::writeOpenClosed(Estimator::x(), Estimator::xhat());
    tOuter = t;
  }

  if (t - tPID >= Config::INNER_DT_MS) {
    Control::innerLoopPID();         // uses encoder pulses & wheel targets
    tPID = t;
  }

  if (t - tDisp >= Config::DISPLAY_DT_MS) {
    Display::update(IMU::lastEuler(), GPS::north(), GPS::east());
    tDisp = t;
  }

  // optional: I2C overrides are polled inside Hardware if needed
  Hardware::pollI2COverride();
}