#include "GPS.h"
#include "Config.h"
#include "Types.h"
#include <TinyGPS++.h>

namespace {
  TinyGPSPlus gps;
  bool originSet{false};
  Types::ECEF ecef0{};
  double lat0{}, lon0{}, alt0{};
  double biasN{}, biasE{}, varN{}, varE{};
  double north_c{}, east_c{};

  // WGS84
  constexpr double a = 6378137.0;
  constexpr double e2 = 6.69437999014e-3;

  Types::ECEF llh2ecef(double lat, double lon, double alt) {
    const double sL=sin(lat), cL=cos(lat), sO=sin(lon), cO=cos(lon);
    const double N = a / sqrt(1.0 - e2*sL*sL);
    return {(N+alt)*cL*cO,(N+alt)*cL*sO,((1.0-e2)*N+alt)*sL};
  }
  Types::NED ecef2ned(const Types::ECEF& p, const Types::ECEF& p0, double lat0, double lon0) {
    const double dx=p.x-p0.x, dy=p.y-p0.y, dz=p.z-p0.z;
    const double sL=sin(lat0), cL=cos(lat0), sO=sin(lon0), cO=cos(lon0);
    return {
      -sL*cO*dx - sL*sO*dy + cL*dz,
            -sO*dx + cO*dy,
      -cL*cO*dx - cL*sO*dy - sL*dz
    };
  }
}

void GPS::init() { Serial1.begin(Config::GPS_BAUD); }

void GPS::calibrate() {
  int n=0; double sumN=0, sumE=0;
  double sN[Config::CAL_SAMPLES_GPS]{}, sE[Config::CAL_SAMPLES_GPS]{};

  while (n < Config::CAL_SAMPLES_GPS) {
    while (Serial1.available()) gps.encode(Serial1.read());
    if (gps.location.isUpdated() && gps.location.isValid()) {
      const double lat = gps.location.lat() * Config::DEG2RAD;
      const double lon = gps.location.lng() * Config::DEG2RAD;
      const double alt = gps.altitude.meters();

      const auto ecef = llh2ecef(lat, lon, alt);
      if (!originSet) { ecef0=ecef; lat0=lat; lon0=lon; alt0=alt; originSet=true; }

      if (originSet) {
        const auto ned = ecef2ned(ecef, ecef0, lat0, lon0);
        sN[n]=ned.n; sE[n]=ned.e; sumN+=ned.n; sumE+=ned.e; ++n;
      }
    }
  }
  biasN = sumN / n;
  biasE = sumE / n;
  double sn=0,se=0;
  for (int i=0;i<n;++i){ sn+=(sN[i]-biasN)*(sN[i]-biasN); se+=(sE[i]-biasE)*(sE[i]-biasE); }
  varN = sn/n; varE = se/n;
}

void GPS::update() {
  while (Serial1.available()) gps.encode(Serial1.read());
  if (gps.location.isUpdated() && gps.location.isValid() && originSet) {
    const double lat = gps.location.lat() * Config::DEG2RAD;
    const double lon = gps.location.lng() * Config::DEG2RAD;
    const double alt = gps.altitude.meters();
    const auto ecef = llh2ecef(lat, lon, alt);
    const auto ned  = ecef2ned(ecef, ecef0, lat0, lon0);
    north_c = ned.n - biasN;
    east_c  = ned.e - biasE;
  }
}

double GPS::north() { return north_c; }
double GPS::east()  { return east_c;  }
double GPS::varianceNorth() { return varN; }
double GPS::varianceEast()  { return varE; }