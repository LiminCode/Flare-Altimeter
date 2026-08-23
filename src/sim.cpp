#include "sim.h"

#include <Arduino.h>

#include "hw_console.h"

namespace sim {
namespace {

bool activeFlag = false;
uint32_t startMs = 0;
uint32_t durationMs = 0;

}  // namespace

bool active() { return activeFlag; }

uint32_t elapsedMs(uint32_t nowMs) { return nowMs - startMs; }

void start(uint32_t dur, const char *label) {
  activeFlag = true;
  startMs = millis();
  durationMs = dur;
  Out.printf("\n[sim] descent %.1f m -> %.1f m in %.1f s (%s)\n",
             FROM_CM / 100.0f, TO_CM / 100.0f, dur / 1000.0f, label);
}

void startSlow() {
  start((uint32_t)SLOW_DURATION_MS, "slow -- hear every stage");
}

void startReal() {
  start((uint32_t)((FROM_CM - TO_CM) / REAL_CM_PER_S * 1000.0f),
        "SLOWEST normal canopy 4.5 m/s = 10 mph");
}

bool update(uint32_t nowMs, float &heightCm, bool &valid) {
  if (!activeFlag) return false;
  const uint32_t t = nowMs - startMs;
  if (t >= durationMs + GROUND_HOLD_MS) {
    activeFlag = false;
    valid = false;                 // hand the ladder back to the laser
    Out.println(F("[sim] done -- back to live laser data"));
    return false;
  }
  heightCm = (t >= durationMs)
                 ? GROUND_CM
                 : FROM_CM + (TO_CM - FROM_CM) * (float)t / (float)durationMs;
  valid = true;
  return true;
}

}  // namespace sim
