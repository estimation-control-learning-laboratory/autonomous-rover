#include "Hardware.h"
#include "MotorData.h"
#include <Wire.h>

namespace {
  // Motor pin map
  const int MOTOR_PWM[4]  = { 7, 6, 9, 8 };
  const int MOTOR_DIRA[4] = { 35, 32, 11, 38 };
  const int MOTOR_DIRB[4] = { 33, 34, 10, 39 };

  const int ENCODER_A[4]  = { A0, 27, A2, 2 };
  const int ENCODER_B[4]  = { A1, 26, A3, 3 };

  volatile long encPulses[4] = {0,0,0,0};

  void setHBridge(int i, int dir) {
    if (dir > 0)      { digitalWrite(MOTOR_DIRA[i], LOW);  digitalWrite(MOTOR_DIRB[i], HIGH); }
    else if (dir < 0) { digitalWrite(MOTOR_DIRA[i], HIGH); digitalWrite(MOTOR_DIRB[i], LOW);  }
    else              { digitalWrite(MOTOR_DIRA[i], LOW);  digitalWrite(MOTOR_DIRB[i], LOW);  }
  }
}

void Hardware::initPins() {
  for (int i=0;i<4;++i) {
    pinMode(MOTOR_PWM[i], OUTPUT);
    pinMode(MOTOR_DIRA[i], OUTPUT);
    pinMode(MOTOR_DIRB[i], OUTPUT);
    pinMode(ENCODER_A[i], INPUT_PULLUP);
    pinMode(ENCODER_B[i], INPUT_PULLUP);
    analogWrite(MOTOR_PWM[i], 0);
  }
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[0]), Hardware::encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[1]), Hardware::encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[2]), Hardware::encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[3]), Hardware::encoderISR3, CHANGE);
  Wire.begin();
}

void Hardware::setMotorPWM(int idx, int pwmVal, int dir) {
  setHBridge(idx, dir);
  analogWrite(((int[]){7,6,9,8})[idx], constrain(abs(pwmVal), 0, 255));
}

long Hardware::swapEncoderPulses(int idx) {
  noInterrupts(); long p = encPulses[idx]; encPulses[idx]=0; interrupts(); return p;
}

// ISRs (quadrature sign patterns preserved)
void Hardware::encoderISR0(){ int A=digitalRead(A0), B=digitalRead(A1); encPulses[0]+= (A==B)?-1:+1; }
void Hardware::encoderISR1(){ int A=digitalRead(27), B=digitalRead(26); encPulses[1]+= (A==B)?+1:-1; }
void Hardware::encoderISR2(){ int A=digitalRead(A2), B=digitalRead(A3); encPulses[2]+= (A==B)?-1:+1; }
void Hardware::encoderISR3(){ int A=digitalRead(2 ), B=digitalRead(3 ); encPulses[3]+= (A==B)?-1:+1; }

// Optional I2C override (address 8) using your MotorDataPacket
void Hardware::pollI2COverride() {
  if (Wire.requestFrom(8, (int)sizeof(MotorDataPacket)) == sizeof(MotorDataPacket)) {
    MotorDataPacket cmd; Wire.readBytes((byte*)&cmd, sizeof(MotorDataPacket));
    setMotorPWM(0, abs(cmd.fl), (cmd.fl>=0)?+1:(cmd.fl==0?0:-1));
    setMotorPWM(1, abs(cmd.fr), (cmd.fr>=0)?+1:(cmd.fr==0?0:-1));
    setMotorPWM(2, abs(cmd.bl), (cmd.bl>=0)?+1:(cmd.bl==0?0:-1));
    setMotorPWM(3, abs(cmd.br), (cmd.br>=0)?+1:(cmd.br==0?0:-1));
  }
}
