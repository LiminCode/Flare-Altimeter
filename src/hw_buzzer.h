// Tone output for a passive buzzer module or a bare piezo disc.
//
// A bare disc is a capacitor (~15 nF), not a buzzer: it makes no sound of its own
// and needs an AC waveform. One LEDC channel drives lead A directly and lead B
// through the GPIO matrix with the invert bit set, so the two pins are opposite by
// construction. Two independent channels would not be -- nothing keeps their
// phases locked, and a drifting phase means a drifting voltage across the disc.
//
// Silence flips lead B back to the NON-inverted routing. With duty 0 that leaves
// both pins low, i.e. no volts across the element. Merely stopping the tone while
// B stays inverted would park 3.3 V DC across it: no sound, but a permanently
// deflected disc and a click on every start and stop.
//
// 中文说明：
//   裸压电片本身不会发声，它就是个约 15 nF 的电容，必须外面喂交流波形。
//   用一个 LEDC 通道，A 脚直出、B 脚经 GPIO 交换矩阵取反输出，两脚天然永远反相。
//   不用两个通道，是因为没有任何机制保证两个通道相位锁死，相位一漂声音就忽大忽小。
//
//   静音这里有个坑：只把占空比设成 0 而 B 脚还挂着反相信号的话，A 低 B 高，
//   压电片两端就一直压着 3.3 V 直流 —— 不出声，但陶瓷片被持续掰弯，
//   而且每次开关都会「啪」一声。所以静音时要把 B 脚改回不反相，两脚同时为低。
#pragma once

#include <stdint.h>

namespace hw {

void buzzBegin();

// Re-run the LEDC setup. The wiring diagnostics steal these pins exactly the way
// they steal the I2C and UART ones; this hands them back.
void buzzReattach();

void buzzTone(uint16_t hz);      // 0 = silent
void buzzSetMuted(bool muted);
bool buzzMuted();

}  // namespace hw
