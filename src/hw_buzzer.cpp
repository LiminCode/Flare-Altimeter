#include "hw_buzzer.h"

#include <Arduino.h>

#include "driver/gpio.h"
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

// A bare piezo disc is a capacitor (tens of nF). The series resistor in the
// wiring notes exists only to blunt the current spike on each PWM edge, when the
// pin sees that capacitor as a short. The pad's own drive strength does the same
// job from the inside: the weakest setting raises the driver's output impedance to
// roughly 100-150 ohm, so a disc can hang directly on the pin with no resistor.
// At 20 nF that is a ~3 us edge, invisible against a 200 us period at 5 kHz.
// 中文说明（没有电阻时怎么办）：
//   裸压电片在电路上就是一个几十 nF 的电容。串 100 Ω 的唯一目的是在 PWM 每个
//   跳变沿上限制瞬间电流 —— 跳变那一刻电容对引脚相当于短路。
//   ESP32 每个引脚的驱动能力可以设成四档，最弱一档等效输出阻抗约 100~150 Ω，
//   和外接 100 Ω 电阻的效果一样，所以压电片可以直接挂在脚上不串任何东西。
//   代价是边沿变慢到约 3 µs，相对 5 kHz 的 200 µs 周期完全可以忽略，音量不受影响。
//   两片压电片串联**不能**代替电阻：电容串联只是分压，每片只剩一半电压，更小声，
//   而且对限制边沿电流没有帮助。
void buzzBegin() {
  ledcSetup(BUZZ_CH, 2000, BUZZ_RES_BITS);
  ledcAttachPin(PIN_BUZZ_A, BUZZ_CH);
  gpio_set_drive_capability((gpio_num_t)PIN_BUZZ_A, GPIO_DRIVE_CAP_0);
  if (PIN_BUZZ_B >= 0) {
    pinMode(PIN_BUZZ_B, OUTPUT);
    ledcAttachPin(PIN_BUZZ_B, BUZZ_CH);
    gpio_set_drive_capability((gpio_num_t)PIN_BUZZ_B, GPIO_DRIVE_CAP_0);
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
    // Silence is a DC level, and which level depends on what is wired: a bare
    // disc wants both leads low, a low-trigger module wants its I/O held high.
    // ledcWrite() treats full-scale duty as "pin solid high".
    // 中文：静音就是一个直流电平，高还是低取决于接的是什么：裸压电片两脚都拉低，
    //       低电平触发模块则要把 I/O 保持在高。ledcWrite 满占空比 = 引脚恒高。
    ledcWrite(BUZZ_CH, BUZZ_IDLE_HIGH ? (1u << BUZZ_RES_BITS) - 1 : 0);
    routeB(false);                 // B follows A => 0 V across a two-lead element
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
