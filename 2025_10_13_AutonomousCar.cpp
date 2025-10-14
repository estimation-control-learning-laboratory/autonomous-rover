// ====== Includes ======
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Arduino_GigaDisplay.h>
#include <Arduino_GigaDisplay_GFX.h>
#include <TinyGPS++.h>
#include <ArduinoEigenDense.h>
#include <Arduino_USBHostMbed5.h>
#include <DigitalOut.h>
#include <FATFileSystem.h>
#include <cmath>

#include "src/MotorData.h"

// ====== Namespaces & Aliases ======
using namespace Eigen;

// ====== Display Colors ======
#define BLACK 0x0000
#define WHITE 0xFFFF

// ====== Hardware / IO ======
#define GPS_SERIAL Serial1
USBHostMSD msd;
mbed::FATFileSystem usb("usb");

// ====== Timing Helpers ======
static inline unsigned long nowMs() { return millis(); }
static inline double msToSec(unsigned long dt_ms){ return dt_ms / 1000.0; }

// ====== GPS/WGS84 ======
static const uint32_t GPS_BAUD = 38400;
static const double   DEG2RAD  = M_PI / 180.0;
static const double   WGS84_a  = 6378137.0;              // semi-major axis (m)
static const double   WGS84_e2 = 6.69437999014e-3;       // ecc^2

struct ECEF { double x{}, y{}, z{}; };
struct NED  { double n{}, e{}, d{}; };

static inline ECEF  llhToECEF(double lat_rad, double lon_rad, double alt_m) {
  ECEF e;
  const double sinL = sin(lat_rad), cosL = cos(lat_rad);
  const double sinLON = sin(lon_rad), cosLON = cos(lon_rad);
  const double N = WGS84_a / sqrt(1.0 - WGS84_e2 * sinL * sinL);
  e.x = (N + alt_m) * cosL * cosLON;
  e.y = (N + alt_m) * cosL * sinLON;
  e.z = ((1.0 - WGS84_e2) * N + alt_m) * sinL;
  return e;
}
static inline NED ecefToNed(const ECEF& p, const ECEF& p0, double lat0, double lon0) {
  const double dx = p.x - p0.x, dy = p.y - p0.y, dz = p.z - p0.z;
  const double sL = sin(lat0), cL = cos(lat0), sO = sin(lon0), cO = cos(lon0);
  NED n;
  n.n = -sL * cO * dx - sL * sO * dy + cL * dz;
  n.e =        -sO * dx +        cO * dy;
  n.d = -cL * cO * dx - cL * sO * dy - sL * dz;
  return n;
}

// ====== Global State ======

// USB logging
int   g_err{0};
int   g_count{0};
FILE *g_logClosed{nullptr};
FILE *g_logOpen{nullptr};

// Comms from I2C master (optional manual speed override)
MotorDataPacket motorCommands;

// GPS
TinyGPSPlus gps;
bool   originSet{false};
ECEF   ecefOrigin{};
double lat0_rad{0.0}, lon0_rad{0.0}, alt0_m{0.0};
double correctedEast{0.0}, correctedNorth{0.0};
NED    nedPos{};
bool   gpsHasNewData{false};

// IMU
Adafruit_BNO055 bno(55, 0x28);
imu::Vector<3> acc, euler, gyro;
uint8_t calSystem{}, calGyro{}, calAccel{}, calMag{};

// Calibration (IMU + GPS)
constexpr int CAL_SAMPLES_IMU  = 1000;
constexpr int CAL_SAMPLES_GPS  = 50;
double bias_ax{0.0}, bias_ay{0.0}, bias_omega{0.0}, mean_theta{0.0};
double var_north{0.0}, var_east{0.0}, var_theta{0.0};
double var_ax{0.0}, var_ay{0.0}, var_omega{0.0};
double biasNorth{0.0}, biasEast{0.0};

// Display
GigaDisplay_GFX display;
unsigned long lastDisplayMs{0};
constexpr unsigned int DISPLAY_DT_MS = 200;

// Encoders / Motors
// Keep a single consistent set (removed duplicates).
const int MOTOR_PWM[4]  = { 7, 6, 9, 8 };
const int MOTOR_DIRA[4] = { 35, 32, 11, 38 };
const int MOTOR_DIRB[4] = { 33, 34, 10, 39 };

const int ENCODER_A[4]  = { A0, 27, A2, 2 };  // A* pins must be interrupt-capable
const int ENCODER_B[4]  = { A1, 26, A3, 3 };

volatile long encPulses[4] = {0,0,0,0};
double        currentRPM[4]{};
double        outputPWM[4]{};

constexpr int ENCODER_PPR   = 12;
constexpr int GEAR_RATIO    = 120;
constexpr int CONTROL_DT_MS = 20;

double pidKp = 0.6, pidKi = 0.2, pidKd = 0.0;
double pidErr[4]{}, pidLast[4]{}, pidInt[4]{}, pidDer[4]{};

// Control timing
unsigned long tStart{0}, tPrev{0}, tPID{0}, tOuter{0};
constexpr unsigned long OUTER_DT_MS = 30;

// Outer-loop references
const double circRate = 0.01; // rad/s
float speedCmd[4]{};          // wheel angular rate refs (rad/s)

// Estimator (Eigen)
MatrixXd K(4,3), Bm(3,4);
MatrixXd A(5,5), Bx(5,3), Ad(5,5), Bd(5,3), I5(5,5), C(3,5);
MatrixXd Pk(5,5), Qk(5,5), Rk(3,3), Pk1(5,5), Pk1k1(5,5), Rbar(3,3), Kest(5,3);
MatrixXd x(5,1), xhat(5,1);
Vector3d yMeas;               // [north, east, theta]
Vector4d wheelVel;            // controller output
Vector3d xi, xi_ref;          // states used by K

// ====== Forward Declarations ======
void setupUSB();
void setupLogFiles();
void writeToUSB();
void setupIMU();
void calibrateIMU();
void setupGPS();
void calibrateGPS();
void updateGPS();
void setupPins();
void setupEstimator();
void controllerOuter();
void controllerPID();
void setMotorSpeedPins(int dira, int dirb, int pwm, int spd);
void updateDisplay();

// ISRs
void encoderISR0();
void encoderISR1();
void encoderISR2();
void encoderISR3();

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  pinMode(PA_15, OUTPUT);  // power/LED latch?
  digitalWrite(PA_15, HIGH);

  setupUSB();
  setupLogFiles();

  setupPins();
  setupIMU();
  calibrateIMU();

  setupGPS();
  calibrateGPS();

  setupEstimator();

  tStart = tPrev = tPID = tOuter = nowMs();

  // (Optional) zero outputs
  for (int i = 0; i < 4; ++i) {
    analogWrite(MOTOR_PWM[i], 0);
    digitalWrite(MOTOR_DIRA[i], LOW);
    digitalWrite(MOTOR_DIRB[i], LOW);
  }

  display.begin();
  display.fillScreen(BLACK);
  display.setRotation(1);
  display.setTextSize(3);
  display.setTextColor(WHITE, BLACK);
}

// ====== Main Loop ======
void loop() {
  // Optional manual motor command via I2C
  if (Wire.requestFrom(8, sizeof(MotorDataPacket)) == sizeof(MotorDataPacket)) {
    Wire.readBytes((byte*)&motorCommands, sizeof(MotorDataPacket));
    setMotorSpeedPins(MOTOR_DIRA[0], MOTOR_DIRB[0], MOTOR_PWM[0], motorCommands.fl);
    setMotorSpeedPins(MOTOR_DIRA[1], MOTOR_DIRB[1], MOTOR_PWM[1], motorCommands.fr);
    setMotorSpeedPins(MOTOR_DIRA[2], MOTOR_DIRB[2], MOTOR_PWM[2], motorCommands.bl);
    setMotorSpeedPins(MOTOR_DIRA[3], MOTOR_DIRB[3], MOTOR_PWM[3], motorCommands.br);
  }

  const unsigned long t = nowMs();

  // Outer-loop: estimation + reference generation + wheel velocity allocation
  if (t - tOuter >= OUTER_DT_MS) {
    tOuter = t;
    controllerOuter();
  }

  // Inner-loop PID on wheel speeds
  if (t - tPID >= CONTROL_DT_MS) {
    tPID = t;
    controllerPID();
  }

  // Display (slow)
  if (t - lastDisplayMs >= DISPLAY_DT_MS) {
    lastDisplayMs = t;
    updateDisplay();
  }
}

// ====== Subsystems ======

// USB & logging ---------------------------------------------------------------
void setupUSB() {
  msd.connect();
  Serial.println("Waiting for USB drive...");
  while (!msd.connect()) {
    Serial.println("MSD not found. Please insert a USB drive.");
    delay(1000);
  }
  g_err = usb.mount(&msd);
  if (g_err) {
    Serial.print("Error mounting USB device: "); Serial.println(g_err);
    while (true) { delay(1000); }
  }
  Serial.println("USB Mounted Successfully.");
}

void setupLogFiles() {
  char fn1[40], fn2[40];
  int idx = 0;

  // Find next available index by probing the open-loop name you used earlier
  while (true) {
    sprintf(fn1, "/usb/open_loop_est_%d.csv", idx);
    FILE* test = fopen(fn1, "r");
    if (!test) break;
    fclose(test);
    ++idx;
  }

  // Final names (consistent with headers below)
  sprintf(fn1, "/usb/closed_loop_est_%d.csv", idx);
  sprintf(fn2, "/usb/open_loop_est_%d.csv",   idx);

  Serial.print("Logging to: "); Serial.println(fn1);
  Serial.print("And: ");        Serial.println(fn2);

  g_logClosed = fopen(fn1, "w+");
  if (!g_logClosed) {
    Serial.println("Failed to create closed_loop_est log file!");
  } else {
    // closed-loop = estimate xhat
    fprintf(g_logClosed, "Count,x_pos_est,y_pos_est\n");
  }

  g_logOpen = fopen(fn2, "w+");
  if (!g_logOpen) {
    Serial.println("Failed to create open_loop_est log file!");
  } else {
    // open-loop = prediction x
    fprintf(g_logOpen, "Count,x_pos,y_pos\n");
  }
}

void writeToUSB() {
  if (!g_logClosed || !g_logOpen) return;

  ++g_count;

  if (isnan(x(0)) || isnan(x(2)) || isnan(xhat(0)) || isnan(xhat(2))) {
    Serial.println("ERROR: NaN in state vector. Skipping file write.");
    return;
  }

  // Log closed-loop estimate (xhat)
  fprintf(g_logClosed, "%d,%.4f,%.4f\n", g_count, xhat(0), xhat(2));
  fflush(g_logClosed);

  // Log open-loop prediction (x)
  fprintf(g_logOpen, "%d,%.4f,%.4f\n", g_count, x(0), x(2));
  fflush(g_logOpen);

  if (g_count % 100 == 0) {
    Serial.print("Wrote reading Nr: "); Serial.println(g_count);
  }

  // Optional rollover/close to ensure FS integrity
  if (g_count >= 1000) {
    Serial.println("Closing log files to finalize write...");
    fclose(g_logClosed); fclose(g_logOpen);
    g_logClosed = g_logOpen = nullptr;
    Serial.println("Files closed. Further logging disabled.");
  }
}

// IMU -------------------------------------------------------------------------
void setupIMU() {
  Wire.begin();
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected!");
    while (true) { delay(1000); }
  }
  // bno.setMode(OPERATION_MODE_NDOF); // NDOF is default in many libs; uncomment if needed.
}

void calibrateIMU() {
  Serial.println("Starting IMU calibration. Keep the sensor still...");

  double sx=0, sy=0, so=0, st=0;
  double vsx=0, vsy=0, vso=0, vst=0;

  for (int i = 0; i < CAL_SAMPLES_IMU; ++i) {
    acc   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    const double ax = acc.x();
    const double ay = acc.y();
    const double om = -gyro.z();         // yaw rate (sign like your original)
    const double th = euler.x() * DEG2RAD;

    sx += ax; sy += ay; so += om; st += th;
    vsx += ax*ax; vsy += ay*ay; vso += om*om; vst += th*th;

    delay(5);
  }

  bias_ax    = sx / CAL_SAMPLES_IMU;
  bias_ay    = sy / CAL_SAMPLES_IMU;
  bias_omega = so / CAL_SAMPLES_IMU;
  mean_theta = st / CAL_SAMPLES_IMU;

  var_ax     = vsx / CAL_SAMPLES_IMU - bias_ax * bias_ax;
  var_ay     = vsy / CAL_SAMPLES_IMU - bias_ay * bias_ay;
  var_omega  = vso / CAL_SAMPLES_IMU - bias_omega * bias_omega;
  var_theta  = vst / CAL_SAMPLES_IMU - mean_theta * mean_theta;

  Serial.println("IMU Calibration Results:");
  Serial.print("Accel X Bias: "); Serial.println(bias_ax, 6);
  Serial.print("Accel X Var : "); Serial.println(var_ax, 6);
  Serial.print("Accel Y Bias: "); Serial.println(bias_ay, 6);
  Serial.print("Accel Y Var : "); Serial.println(var_ay, 6);
  Serial.print("Omega  Bias : "); Serial.println(bias_omega, 6);
  Serial.print("Omega  Var  : "); Serial.println(var_omega, 6);
}

// GPS -------------------------------------------------------------------------
void setupGPS() {
  GPS_SERIAL.begin(GPS_BAUD);
}

void calibrateGPS() {
  Serial.println("Calibrating GPS bias/variance...");

  double sumN=0.0, sumE=0.0;
  double samplesN[CAL_SAMPLES_GPS]{}, samplesE[CAL_SAMPLES_GPS]{};
  int n = 0;

  while (n < CAL_SAMPLES_GPS) {
    if (GPS_SERIAL.available() > 0 && gps.encode(GPS_SERIAL.read())) {
      if (gps.location.isUpdated() && gps.location.isValid()) {
        const double lat_deg = gps.location.lat();
        const double lon_deg = gps.location.lng();
        const double alt_m   = gps.altitude.meters();

        const double lat_r = lat_deg * DEG2RAD;
        const double lon_r = lon_deg * DEG2RAD;

        const ECEF ecefCur = llhToECEF(lat_r, lon_r, alt_m);

        if (!originSet) {
          ecefOrigin = ecefCur;
          lat0_rad   = lat_r;
          lon0_rad   = lon_r;
          alt0_m     = alt_m;
          originSet  = true;
          Serial.println("GPS origin set.");
        }

        if (originSet) {
          NED ned = ecefToNed(ecefCur, ecefOrigin, lat0_rad, lon0_rad);
          sumN += ned.n; sumE += ned.e;
          samplesN[n] = ned.n; samplesE[n] = ned.e;
          ++n;
        }
      }
    }
  }

  biasNorth = sumN / CAL_SAMPLES_GPS;
  biasEast  = sumE / CAL_SAMPLES_GPS;

  double sn=0.0, se=0.0;
  for (int i=0;i<CAL_SAMPLES_GPS;++i) {
    sn += (samplesN[i] - biasNorth) * (samplesN[i] - biasNorth);
    se += (samplesE[i] - biasEast)  * (samplesE[i] - biasEast);
  }
  var_north = sn / CAL_SAMPLES_GPS;
  var_east  = se / CAL_SAMPLES_GPS;

  Serial.print("GPS North Bias: "); Serial.println(biasNorth, 6);
  Serial.print("GPS East  Bias: "); Serial.println(biasEast, 6);
}

void updateGPS() {
  if (GPS_SERIAL.available() > 0 && gps.encode(GPS_SERIAL.read())) {
    if (gps.location.isUpdated() && gps.location.isValid()) {
      const double lat_r = gps.location.lat() * DEG2RAD;
      const double lon_r = gps.location.lng() * DEG2RAD;
      const double alt_m = gps.altitude.meters();

      const ECEF ecefCur = llhToECEF(lat_r, lon_r, alt_m);

      if (originSet) {
        nedPos = ecefToNed(ecefCur, ecefOrigin, lat0_rad, lon0_rad);
        correctedEast  = nedPos.e - biasEast;
        correctedNorth = nedPos.n - biasNorth;

        Serial.print("Corrected North (m): "); Serial.println(correctedNorth, 2);
        Serial.print("Corrected East  (m): "); Serial.println(correctedEast, 2);
      }
    }
  }
}

// Pins & interrupts -----------------------------------------------------------
void setupPins() {
  // Motors
  for (int i = 0; i < 4; ++i) {
    pinMode(MOTOR_PWM[i],  OUTPUT);
    pinMode(MOTOR_DIRA[i], OUTPUT);
    pinMode(MOTOR_DIRB[i], OUTPUT);
  }
  // Encoders
  for (int i = 0; i < 4; ++i) {
    pinMode(ENCODER_A[i], INPUT_PULLUP);
    pinMode(ENCODER_B[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[0]), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[1]), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[2]), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[3]), encoderISR3, CHANGE);
}

// Estimator / Controller ------------------------------------------------------
void setupEstimator() {
  // Controller allocation matrix K (as in your original)
  K.resize(4,3);
  K << -50.0,  50.0,  12.5,
       -50.0, -50.0, -12.5,
       -50.0, -50.0,  12.5,
       -50.0,  50.0, -12.5;

  // Bm (unused in controllerOuter but keep to mirror original)
  Bm.resize(3,4);
  Bm <<
    0.0150, 0.0150, 0.0150, 0.0150,
   -0.0150, 0.0150, 0.0150,-0.0150,
   -0.0600, 0.0600,-0.0600, 0.0600;

  // State-space structures
  I5.setZero(5,5);
  A.setZero(5,5);
  Bx.setZero(5,3);
  C.setZero(3,5);

  I5.diagonal().setOnes();

  // x = [r1, r1dot, r2, r2dot, theta]
  A(0,1) = 1.0;
  A(2,3) = 1.0;

  Bx(1,0) = 1.0;   // r1ddot input
  Bx(3,1) = 1.0;   // r2ddot input
  Bx(4,2) = 1.0;   // theta_dot input

  C(0,0) = 1.0;    // measure r1 (north)
  C(1,2) = 1.0;    // measure r2 (east)
  C(2,4) = 1.0;    // measure theta

  // KF init
  Pk   = 100.0 * I5;
  Qk   =   0.1 * I5;

  Rk.setZero(3,3);
  Rk(0,0) = var_north;
  Rk(1,1) = var_east;
  Rk(2,2) = var_theta;

  xhat.setZero(5,1);
  x.setZero(5,1);
  xhat(0) = -1.0;  // your original initial condition
}

// Outer loop: integrate IMU, update KF, set wheel targets
void controllerOuter() {
  const unsigned long t = nowMs();
  const double tsec = msToSec(t - tStart);
  const double dt   = msToSec(t - tPrev);
  tPrev = t;

  // Read sensors
  acc   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  bno.getCalibration(&calSystem, &calGyro, &calAccel, &calMag);

  // Bias-correct body-frame accelerations and yaw rate
  const double ax = acc.x()  - bias_ax;
  const double ay = acc.y()  - bias_ay;
  const double wz = -gyro.z() - bias_omega;

  const double yaw = euler.x() * DEG2RAD; // BNO yaw in degrees about Z (+CW in your sign)

  // Project to world r1ddot/r2ddot (reuse trig once)
  const double c = cos(yaw), s = sin(yaw);
  const double r1dd = (-ax * c) + ( ay * s);
  const double r2dd = ( ax * s) + ( ay * c);

  // Discretize & predict
  Ad = I5 + A * dt;
  Bd = Bx * dt;

  Vector3d u; u << r1dd, r2dd, wz;
  x = Ad * xhat + Bd * u;

  // Time-update covariance
  Pk1 = Ad * Pk * Ad.transpose() + Qk;
  Rbar = C * Pk1 * C.transpose() + Rk;
  Kest = Pk1 * C.transpose() * Rbar.inverse();
  Pk   = Pk1 - Kest * C * Pk1;

  // GPS update
  updateGPS();
  yMeas << correctedNorth, correctedEast, yaw; // yaw measured directly

  // Measurement update
  xhat = x + Kest * (yMeas - C * x);

  // Log
  writeToUSB();

  // Reference (circle)
  xi     << xhat(0), xhat(2), xhat(4);
  xi_ref << -cos(circRate * tsec),  // north
             sin(circRate * tsec),  // east
            (-(circRate * tsec) + (M_PI / 2.0)); // yaw

  // Wheel rates
  wheelVel = K * (xi - xi_ref);

  // Map to speed command array (rad/s)
  speedCmd[0] = wheelVel(1);
  speedCmd[1] = wheelVel(0);
  speedCmd[2] = wheelVel(2);
  speedCmd[3] = wheelVel(3);
}

// Inner loop: PID on wheel angular speed
void controllerPID() {
  const unsigned long t = nowMs();
  const double dt = msToSec(t - tPID); // tPID updated by caller

  for (int i = 0; i < 4; ++i) {
    noInterrupts();
    long pulses = encPulses[i];
    encPulses[i] = 0;
    interrupts();

    // pulses -> RPM -> rad/s
    const double revs = double(pulses) / (ENCODER_PPR * GEAR_RATIO);
    currentRPM[i] = (revs / dt) * 60.0;
    const double omega = currentRPM[i] * (M_PI / 30.0);

    // PID
    pidErr[i]  = speedCmd[i] - omega;
    pidInt[i] += pidErr[i] * dt;
    pidDer[i]  = (pidErr[i] - pidLast[i]) / dt;
    pidLast[i] = pidErr[i];

    outputPWM[i] = pidKp * pidErr[i] + pidKi * pidInt[i] + pidKd * pidDer[i];

    // Motor command
    if (outputPWM[i] > 0) {
      digitalWrite(MOTOR_DIRA[i], LOW);
      digitalWrite(MOTOR_DIRB[i], HIGH);
    } else if (outputPWM[i] < 0) {
      digitalWrite(MOTOR_DIRA[i], HIGH);
      digitalWrite(MOTOR_DIRB[i], LOW);
    } else {
      digitalWrite(MOTOR_DIRA[i], LOW);
      digitalWrite(MOTOR_DIRB[i], LOW);
    }

    if (outputPWM[i] != 0) {
      analogWrite(MOTOR_PWM[i], constrain((int)fabs(outputPWM[i]), 40, 255));
    } else {
      analogWrite(MOTOR_PWM[i], 0);
    }
  }
}

// Manual motor control helper
void setMotorSpeedPins(int dira, int dirb, int pwm, int spd) {
  if (spd == 0) {
    digitalWrite(dira, LOW); digitalWrite(dirb, LOW); analogWrite(pwm, 0);
  } else if (spd > 0) {
    digitalWrite(dira, HIGH); digitalWrite(dirb, LOW);  analogWrite(pwm, constrain(spd, 0, 255));
  } else {
    digitalWrite(dira, LOW);  digitalWrite(dirb, HIGH); analogWrite(pwm, constrain(-spd, 0, 255));
  }
}

// Display ---------------------------------------------------------------------
void updateDisplay() {
  display.setCursor(0, 0);
  display.print("Calib: ");
  display.print(calSystem); display.print(calGyro);
  display.print(calAccel);  display.print(calMag);

  display.setCursor(0, 30);  display.print("AccX: "); display.print(acc.x());    display.print("   ");
  display.setCursor(0, 60);  display.print("AccY: "); display.print(acc.y());    display.print("   ");
  display.setCursor(0, 90);  display.print("AccZ: "); display.print(acc.z());    display.print("   ");

  display.setCursor(0, 120); display.print("Roll : "); display.print(euler.z()); display.print("   ");
  display.setCursor(0, 150); display.print("Pitch: "); display.print(euler.y()); display.print("   ");
  display.setCursor(0, 180); display.print("Yaw  : "); display.print(euler.x()); display.print("   ");

  display.setCursor(0, 210); display.print("GyroX: "); display.print(gyro.x()); display.print("   ");
  display.setCursor(0, 240); display.print("GyroY: "); display.print(gyro.y()); display.print("   ");
  display.setCursor(0, 270); display.print("GyroZ: "); display.print(-gyro.z());display.print("   ");

  display.setCursor(0, 300); display.print("Lat : ");
  display.print(gps.location.isValid() ? gps.location.lat() : 0.0, 4); display.print("   ");
  display.setCursor(0, 330); display.print("Lon : ");
  display.print(gps.location.isValid() ? gps.location.lng() : 0.0, 4); display.print("   ");

  display.setCursor(0, 360);
  display.print("Pos (m): N=");
  display.print(nedPos.n, 2);
  display.print("  E=");
  display.print(nedPos.e, 2);
  display.print("   ");
}

// ====== ISRs ======
void encoderISR0() { int A = digitalRead(ENCODER_A[0]); int B = digitalRead(ENCODER_B[0]); encPulses[0] += (A == B) ? -1 : +1; }
void encoderISR1() { int A = digitalRead(ENCODER_A[1]); int B = digitalRead(ENCODER_B[1]); encPulses[1] += (A == B) ? +1 : -1; }
void encoderISR2() { int A = digitalRead(ENCODER_A[2]); int B = digitalRead(ENCODER_B[2]); encPulses[2] += (A == B) ? -1 : +1; }
void encoderISR3() { int A = digitalRead(ENCODER_A[3]); int B = digitalRead(ENCODER_B[3]); encPulses[3] += (A == B) ? -1 : +1; }
