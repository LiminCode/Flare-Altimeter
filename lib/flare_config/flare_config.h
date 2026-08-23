// Validation for the two settings that are adjustable at runtime and persisted.
//
// Separated out and tested because a stored value that survives a power cycle is
// a value that can outlive the code that wrote it: a firmware update, a partly
// written NVS record, or a stale key from an older build all deliver a float that
// the running code never chose. Clamping and snapping on the way IN is the only
// place that can be defended.
//
// 中文说明（为什么这几行值得单独成模块并测试）：
//   存进 NVS 的值会活得比写它的代码更久 —— 固件升级、写到一半掉电、
//   旧版本残留的键，都会让运行中的代码读到一个它自己从没选过的浮点数。
//   唯一能守住的地方就是**读入的那一刻**：钳位和吸附。
//   尤其是步长：一个脏值会让整条响声阶梯按一个荒谬的间隔切分，而且不会报错。
#pragma once

namespace flare {

constexpr float kContMinCm = 100.0f;
constexpr float kContMaxCm = 600.0f;
constexpr float kContStepCm = 25.0f;

constexpr float kBeepStepChoices[] = {25.0f, 50.0f, 100.0f};
constexpr int kBeepStepChoiceCount = 3;
constexpr float kBeepStepDefaultCm = 50.0f;

// Clamp the changeover height into the adjustable range.
float clampContinuousAtCm(float value);

// Nudge it by one adjustment step, clamped.
float adjustContinuousAtCm(float value, bool up);

// Snap a stored step to a known choice; anything unrecognised falls back to the
// default rather than being used as-is.
float snapBeepStepCm(float stored);

// Next choice in the cycle, wrapping.
float cycleBeepStepCm(float current);

// Beeps per second at a given descent rate -- the number that decides whether a
// step size is countable or reads as a trill.
float beepsPerSecond(float stepCm, float descentCmPerS);

}  // namespace flare
