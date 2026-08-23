// Flare alert engine: turns a stream of (time, height, valid) into a tone.
//
// One beep per `beepStepCm` of height lost, pitch rising geometrically all the way
// down, continuous top pitch at and below `continuousAtCm`, auto-stop once it has
// been on the ground for a while.
//
// The property that makes the model work: beeping every fixed drop in height makes
// the beep RATE equal v / beepStepCm. Descent rate falls out of the geometry for
// free -- no differentiation, no filter, no tau. The ear gets both facts at once:
//     pitch = how far from the flare point,  rate = how fast it is closing.
//
// Pure: no Arduino, no millis(), no hardware. Time is an argument. That is what
// makes it possible to test a two-second landing sequence in microseconds, and to
// test the cases that are almost impossible to stage physically -- a laser dropout
// at 8 m, a thermal, a fast canopy.
//
// 中文说明：
//   每下降 beepStepCm 响一声，音高一路等比升高，到 continuousAtCm 转连续最高音，
//   落地一段时间后自动停止。
//
//   这个模型巧妙的地方：每掉固定高度响一声，意味着响声频率恰好等于
//   下降速率 / 步长。下降速率是从几何关系里白捡的，不需要求导、滤波或算 tau。
//
//   本模块是纯逻辑：不碰 Arduino，不调 millis()，时间由参数传入。
//   这正是能在微秒内测完一次两秒落地全过程的原因，也是能测那些
//   物理上几乎没法复现的场景的原因 —— 8 米高度上的激光丢帧、上升气流、快伞。
#pragma once

#include <stdint.h>

namespace flare {

enum class Stage : uint8_t {
  Silent = 0,
  Far,
  Closing,
  Near,
  FlareNow,
};

const char *stageName(Stage s);

struct AlertConfig {
  float continuousAtCm = 200.0f;   // beeps give way to a continuous tone here
  float beepStepCm = 50.0f;        // height lost per beep
  float alertTopCm = 1300.0f;      // pitch reference top (max usable laser range)
  uint16_t beepMs = 35;            // must fit inside the gap at the fastest rate
  uint16_t pitchLowHz = 700;
  uint16_t pitchHighHz = 3500;

  // Arming contract: continuous data for armConfirmMs, starting above armMinCm,
  // with a real drop across the window. All three, or it stays silent.
  float armMinCm = 800.0f;
  uint32_t armConfirmMs = 300;
  float armMinDropCm = 15.0f;      // 15 cm in 300 ms = 0.5 m/s

  float stopBelowCm = 100.0f;      // auto-stop threshold
  uint32_t stopHoldMs = 2000;      // ...after this long below it
  uint32_t dropoutGraceMs = 300;   // ride out a short loss of signal
};

struct AlertOutput {
  uint16_t toneHz = 0;             // 0 = silent
  Stage stage = Stage::Silent;
  bool armed = false;
  bool beepFired = false;          // a new beep started on this tick
  float beepAtCm = 0.0f;           // height it fired at (valid if beepFired)
  uint16_t beepHz = 0;             // pitch of the most recent beep
};

uint16_t pitchForHeightHz(const AlertConfig &cfg, float heightCm);
Stage stageForHeight(const AlertConfig &cfg, float heightCm);

class AlertEngine {
 public:
  explicit AlertEngine(const AlertConfig &cfg) : cfg_(cfg) {}

  AlertConfig &config() { return cfg_; }
  const AlertConfig &config() const { return cfg_; }

  // heightCm is only consulted when `valid` is true. During the dropout grace the
  // engine reuses the last VALID height rather than whatever the caller last
  // wrote -- see the note in the .cpp, this distinction is a real bug's worth.
  // 中文：valid 为 false 时不使用传入的 heightCm，而是沿用最后一个**有效**高度。
  //       这个区别不是洁癖，是一个真实 bug 的分界线，理由写在 .cpp 里。
  AlertOutput update(uint32_t nowMs, float heightCm, bool valid);

  // Re-align the beep ladder to the current height. Called after the step size
  // changes so the next beep uses the new spacing immediately.
  void reseatLadder(float heightCm);

  void disarm();

  bool armed() const { return armed_; }
  Stage stage() const { return stage_; }
  float nextBeepAtCm() const { return nextBeepAtCm_; }

 private:
  AlertConfig cfg_;

  bool armed_ = false;
  Stage stage_ = Stage::Silent;

  bool haveValidEver_ = false;
  float lastValidCm_ = -1.0f;
  uint32_t lastValidMs_ = 0;

  bool armWindowOpen_ = false;
  uint32_t armWindowMs_ = 0;
  float armWindowCm_ = 0.0f;

  float nextBeepAtCm_ = 0.0f;
  uint32_t beepUntilMs_ = 0;
  uint16_t beepHz_ = 0;
  uint32_t belowStopSinceMs_ = 0;
};

}  // namespace flare
