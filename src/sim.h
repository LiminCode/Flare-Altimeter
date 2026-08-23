// Descent simulator: drives the alert engine from a synthetic height ramp.
//
// Only the alert INPUT is faked. The laser and IMU keep running and keep printing
// real numbers throughout, so a simulated value can never be mistaken for a
// reading. Two speeds, because they answer different questions: the slow one asks
// "is the sound right", the real one asks "is there time to act on it".
//
// 中文说明：
//   用合成的高度曲线代替激光去喂警报状态机，这样不用真把传感器举到 13 米
//   就能在桌面上听完整段音序。注意只有「警报的输入」是假的 ——
//   激光和 IMU 照常工作、照常打印真实数值，不会出现假数据被当成真读数的情况。
//   两种速度回答两个不同的问题：慢速问「声音对不对」，真实速度问「来不来得及反应」。
#pragma once

#include <stdint.h>

namespace sim {

// 13 m is the sensor's worst-case usable range over grass; the ramp runs past the
// changeover height to the ground, then sits on the ground.
//
// Ending the ramp at the changeover height, as an earlier version did at 2 m,
// finishes at the instant that sound begins and never plays it. A demo has to
// cross a threshold to demonstrate it -- and the ground hold is what lets the
// auto-stop rule ("below 1 m for 2 s") fire at all.
// 中文：起点取传感器可用量程上沿，终点穿过转换高度一直到地面，之后在地面上停留。
//       之前的版本把终点定在 2 m —— 正好是转换高度 —— 结果是在那个声音刚要开始的
//       瞬间就结束，永远放不出来。而落地后的停留，是「低于 1 m 满 2 秒自动停」
//       这条规则唯一有机会触发的地方。
constexpr float FROM_CM = 1300.0f;
constexpr float TO_CM = 0.0f;
constexpr float GROUND_CM = 10.0f;
constexpr uint32_t GROUND_HOLD_MS = 4000;
constexpr float SLOW_DURATION_MS = 12000.0f;

// Deliberately the SLOW end of the normal range: 10 mph = 4.5 m/s. Canopies in
// straight flight are quoted at 10-15 mph / ~1000 ft/min, i.e. 4.5-6.7 m/s.
// Demonstrating the most favourable case makes the timing argument impossible to
// dismiss as a pessimistic assumption.
constexpr float REAL_CM_PER_S = 450.0f;

void start(uint32_t durationMs, const char *label);
void startSlow();
void startReal();

bool active();

// Returns true while the simulator owns the alert input, writing the synthetic
// height into `heightCm` / `valid`.
bool update(uint32_t nowMs, float &heightCm, bool &valid);

uint32_t elapsedMs(uint32_t nowMs);

}  // namespace sim
