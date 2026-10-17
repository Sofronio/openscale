#ifndef FUEL_GAUGE_VALIDATE_H
#define FUEL_GAUGE_VALIDATE_H

#include <stdint.h>

inline bool fuelGaugeSocValid(long raw) {
  return raw >= 0 && raw <= 100;
}

inline bool fuelGaugeVoltageValid(long mv) {
  return mv >= 2000 && mv <= 5000;
}

inline bool fuelGaugeCurrentValid(long ma) {
  return ma >= -5000 && ma <= 5000;
}

inline bool fuelGaugePowerValid(long mw) {
  return mw >= -30000 && mw <= 30000;
}

inline bool fuelGaugeCapacityValid(long mah) {
  return mah >= 0 && mah <= 10000;
}

inline bool fuelGaugeFullCapacityValid(long mah) {
  return mah >= 0 && mah <= 20000;
}

inline bool fuelGaugeSohValid(long soh) {
  return soh > 0 && soh <= 100;
}

inline bool fuelGaugeTempValid(long k10) {
  return k10 >= 2200 && k10 <= 3600;
}

#endif
