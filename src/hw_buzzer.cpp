#include "hw_buzzer.h"

#include <Arduino.h>

#include "esp_rom_gpio.h"
#include "pins.h"
#include "soc/gpio_sig_map.h"

namespace hw {
namespace {

bool muted = false;
uint16_t currentHz = 0;

void routeB(bool inverted) {
  if (PIN_BUZZ_B < 0) return;
  esp_rom_gpio_connect_out_signal(PIN_BUZZ_B, LEDC_LS_SIG_OUT0_IDX + BUZZ_CH,
                                  inverted, false);
}

}  // namespace

void buzzBegin() {
  ledcSetup(BUZZ_CH, 2000, BUZZ_RES_BITS);
  ledcAttachPin(PIN_BUZZ_A, BUZZ_CH);
  if (PIN_BUZZ_B >= 0) {
    pinMode(PIN_BUZZ_B, OUTPUT);
    ledcAttachPin(PIN_BUZZ_B, BUZZ_CH);
  }
  currentHz = 1;        // force the next call to take effect
  buzzTone(0);
}

void buzzReattach() { buzzBegin(); }

void buzzTone(uint16_t hz) {
  if (muted) hz = 0;
  if (hz == currentHz) return;     // ledcWriteTone reprograms the timer
  currentHz = hz;
  if (hz == 0) {
    ledcWrite(BUZZ_CH, 0);
    routeB(false);                 // both pins low => 0 V across the element
  } else {
    routeB(true);
    ledcWriteTone(BUZZ_CH, hz);    // sets 50% duty for us
  }
}

void buzzSetMuted(bool m) {
  muted = m;
  if (muted) buzzTone(0);
}

bool buzzMuted() { return muted; }

}  // namespace hw
