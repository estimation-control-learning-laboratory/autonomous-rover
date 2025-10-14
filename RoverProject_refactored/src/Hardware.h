#pragma once
#include <Arduino.h>

namespace Hardware {
  void initPins();
  void pollI2COverride(); // optional manual override
  void setMotorPWM(int idx, int pwmVal, int dir); // dir: -1,0,1
  long swapEncoderPulses(int idx); // atomically get & zero

  // ISRs
  void encoderISR0(); void encoderISR1(); void encoderISR2(); void encoderISR3();
}