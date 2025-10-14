#include "Display.h"
#include "Config.h"
#include <Arduino_GigaDisplay.h>
#include <Arduino_GigaDisplay_GFX.h>

namespace {
  GigaDisplay_GFX dsp;
}

void Display::init() {
  dsp.begin();
  dsp.fillScreen(Config::BLACK);
  dsp.setRotation(1);
  dsp.setTextSize(3);
  dsp.setTextColor(Config::WHITE, Config::BLACK);
}

void Display::update(const imu::Vector<3>& euler, double north_m, double east_m) {
  dsp.setCursor(0, 0);
  dsp.print("Yaw: "); dsp.print(euler.x()); dsp.print("   ");

  dsp.setCursor(0, 40);
  dsp.print("Pitch: "); dsp.print(euler.y()); dsp.print("   ");

  dsp.setCursor(0, 80);
  dsp.print("Roll: "); dsp.print(euler.z()); dsp.print("   ");

  dsp.setCursor(0, 140);
  dsp.print("N (m): "); dsp.print(north_m, 2); dsp.print("   ");

  dsp.setCursor(0, 180);
  dsp.print("E (m): "); dsp.print(east_m, 2); dsp.print("   ");
}