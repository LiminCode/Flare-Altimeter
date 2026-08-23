// Wiring diagnostics: GPIO line probe, I2C bus recovery and scan.
//
// This answers what a bus scan cannot: is the module wired at all, which pins is
// it on, and is anything jamming the bus. It is what found both the unconnected
// TF02 power wire and the I2C pin-detach bug.
#pragma once

#include <stdint.h>

namespace hw {

// Probe every safely probeable GPIO and report anything that is not floating,
// then hand the bus pins back to their peripherals.
void probeLines();

// Returns the I2C address of the BNO08x if the scan found one, else 0.
uint8_t scanI2C();

void i2cBegin();
void i2cReattach();

}  // namespace hw
