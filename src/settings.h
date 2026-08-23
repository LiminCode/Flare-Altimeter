// Persisted settings: the mounting reference and the two tunables.
//
// Validation lives in lib/flare_config so it can be tested; this file is only the
// NVS plumbing around it.
#pragma once

#include "flare_math.h"

namespace settings {

struct Settings {
  flare::Quat mountingRef{1.0f, 0.0f, 0.0f, 0.0f};
  bool refStored = false;
  bool tiltCorrection = true;
  float continuousAtCm = 200.0f;
  float beepStepCm = 50.0f;
};

extern Settings s;

void load();
void save();

}  // namespace settings
