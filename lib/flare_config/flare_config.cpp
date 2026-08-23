#include "flare_config.h"

#include <math.h>

namespace flare {

float clampContinuousAtCm(float value) {
  if (!(value == value)) return kContMinCm;   // NaN: comparisons below all fail
  if (value < kContMinCm) return kContMinCm;
  if (value > kContMaxCm) return kContMaxCm;
  return value;
}

float adjustContinuousAtCm(float value, bool up) {
  return clampContinuousAtCm(value + (up ? kContStepCm : -kContStepCm));
}

float snapBeepStepCm(float stored) {
  for (int i = 0; i < kBeepStepChoiceCount; i++) {
    if (fabsf(stored - kBeepStepChoices[i]) < 0.5f) return kBeepStepChoices[i];
  }
  return kBeepStepDefaultCm;
}

float cycleBeepStepCm(float current) {
  for (int i = 0; i < kBeepStepChoiceCount; i++) {
    if (fabsf(current - kBeepStepChoices[i]) < 0.5f) {
      return kBeepStepChoices[(i + 1) % kBeepStepChoiceCount];
    }
  }
  return kBeepStepDefaultCm;
}

float beepsPerSecond(float stepCm, float descentCmPerS) {
  if (stepCm <= 0.0f) return 0.0f;
  return descentCmPerS / stepCm;
}

}  // namespace flare
