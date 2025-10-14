#include "Logging.h"
#include <Arduino_USBHostMbed5.h>
#include <FATFileSystem.h>

namespace {
  USBHostMSD msd;
  mbed::FATFileSystem usb("usb");
  int   err{0}, count{0};
  FILE *logClosed{nullptr}, *logOpen{nullptr};

  void findNextIndex(char* path, const char* base) {
    int idx=0;
    while (true) {
      sprintf(path, "/usb/%s_%d.csv", base, idx);
      FILE* t = fopen(path, "r"); if (!t) break; fclose(t); ++idx;
    }
  }
}

void Logging::initUSB() {
  msd.connect();
  while (!msd.connect()) { delay(500); }
  err = usb.mount(&msd);
  if (err) while (true) { delay(1000); }
}

void Logging::openLogs() {
  char fnClosed[48], fnOpen[48];
  findNextIndex(fnOpen,   "open_loop_est");
  int idx=0; sscanf(fnOpen, "/usb/open_loop_est_%d.csv", &idx);
  sprintf(fnClosed, "/usb/closed_loop_est_%d.csv", idx);

  logClosed = fopen(fnClosed, "w+");
  logOpen   = fopen(fnOpen,   "w+");
  if (logClosed) fprintf(logClosed, "Count,x_pos_est,y_pos_est\n");
  if (logOpen)   fprintf(logOpen,   "Count,x_pos,y_pos\n");
}

void Logging::writeOpenClosed(const Eigen::Matrix<double,5,1>& x,
                              const Eigen::Matrix<double,5,1>& xhat) {
  if (!logClosed || !logOpen) return;
  ++count;
  fprintf(logClosed, "%d,%.4f,%.4f\n", count, xhat(0), xhat(2)); fflush(logClosed);
  fprintf(logOpen,   "%d,%.4f,%.4f\n", count, x(0),   x(2));     fflush(logOpen);
  if (count >= 1000) { fclose(logClosed); fclose(logOpen); logClosed=logOpen=nullptr; }
}