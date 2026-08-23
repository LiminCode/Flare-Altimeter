// BNO08x attitude source, with the stored mounting reference applied.
#pragma once

#include <stdint.h>

#include "flare_math.h"

namespace hw {

void imuBegin();

// Reset, connect and enable the rotation vector. Returns false if the chip never
// answered on either address.
bool imuStart(uint8_t hintAddress);

bool imuReady();
uint8_t imuAddress();

// True when a fresh rotation vector arrived. `q` comes back with the mounting
// reference already divided out, so "level" means the calibrated pose.
bool imuPoll(flare::Quat &q, uint8_t &accuracy);

// Capture the current pose as the mounting reference. Returns false (and does
// nothing) if the fusion is not yet trustworthy enough to define one.
bool imuCaptureReference(const char *&why);

uint8_t imuAccuracy();

}  // namespace hw
