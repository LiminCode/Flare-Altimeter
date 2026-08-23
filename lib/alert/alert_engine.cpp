#include "alert_engine.h"

#include <math.h>

namespace flare {
namespace {

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

const char *stageName(Stage s) {
  switch (s) {
    case Stage::Far:      return "far";
    case Stage::Closing:  return "closing";
    case Stage::Near:     return "near";
    case Stage::FlareNow: return "FLARE NOW";
    default:              return "silent";
  }
}

uint16_t pitchForHeightHz(const AlertConfig &cfg, float heightCm) {
  // Geometric, not linear. Pitch is perceived logarithmically, so equal height
  // steps must be equal frequency RATIOS to sound like equal musical steps. A
  // linear map crowds the top of the range into an indistinguishable smear
  // exactly where the information matters most.
  // 中文：等比而不是等差。人对音高的感知是对数的，所以「每 50 cm 一步」
  //       要听起来均匀，频率必须按固定比例递增。线性映射会把高频那一段
  //       挤成一团分辨不出来 —— 而那正是最接近拉飘点、最需要分辨的一段。
  const float span = cfg.alertTopCm - cfg.continuousAtCm;
  if (span <= 0.0f) return cfg.pitchHighHz;
  const float p = clampf((cfg.alertTopCm - heightCm) / span, 0.0f, 1.0f);
  return (uint16_t)((float)cfg.pitchLowHz *
                    powf((float)cfg.pitchHighHz / (float)cfg.pitchLowHz, p));
}

Stage stageForHeight(const AlertConfig &cfg, float heightCm) {
  if (heightCm <= cfg.continuousAtCm) return Stage::FlareNow;
  if (heightCm <= 500.0f) return Stage::Near;
  if (heightCm <= 1000.0f) return Stage::Closing;
  return Stage::Far;
}

void AlertEngine::reseatLadder(float heightCm) {
  if (heightCm <= 0.0f || cfg_.beepStepCm <= 0.0f) return;
  nextBeepAtCm_ = floorf(heightCm / cfg_.beepStepCm) * cfg_.beepStepCm;
}

void AlertEngine::disarm() {
  armed_ = false;
  armWindowOpen_ = false;
  belowStopSinceMs_ = 0;
  beepUntilMs_ = 0;
  stage_ = Stage::Silent;
}

AlertOutput AlertEngine::update(uint32_t nowMs, float heightCm, bool valid) {
  AlertOutput out;

  if (valid) {
    lastValidCm_ = heightCm;
    lastValidMs_ = nowMs;
    haveValidEver_ = true;
  }

  // Dropout handling. One blade of grass, one insect, one weak return must not
  // chop the sound into fragments; a real loss of signal must not leave a stale
  // alarm sounding. dropoutGraceMs is the line between those two.
  //
  // Crucially, the grace period reuses the last VALID height, not whatever the
  // caller passed alongside valid=false. The earlier monolithic version used the
  // caller's variable, which the caller sets to -1 when the laser has no echo --
  // so a brief dropout at 8 m altitude fed h = -1 into the ladder, which is below
  // continuousAtCm, and produced a burst of full-volume "FLARE NOW" tone. The
  // grace period, meant to smooth over a lost frame, instead turned it into the
  // single most dangerous sound the device can make.
  //
  // 中文说明（这里修掉了一个真实存在的严重 bug）：
  //   丢帧宽限的用意是：一片草叶、一只虫子、一次弱回波不该把声音切碎；
  //   但真的丢失信号时也不能让过期警报继续响。dropoutGraceMs 是这两者的分界。
  //
  //   关键在于宽限期内用的是「最后一个**有效**高度」，而不是调用方在 valid=false
  //   时一并传进来的那个值。之前的单文件版本用的是调用方的变量 ——
  //   而调用方在激光没有回波时会把它设成 -1。于是 8 米高度上一次短暂丢帧，
  //   喂进阶梯的是 h = -1，它低于 continuousAtCm，结果是一串满音量的
  //   「FLARE NOW」长鸣。
  //   本意是「平滑掉一帧丢失」的宽限期，反而把丢帧变成了这台设备能发出的
  //   最危险的一个声音。
  if (!valid) {
    if (!haveValidEver_ || (nowMs - lastValidMs_) > cfg_.dropoutGraceMs) {
      disarm();
      out.armed = false;
      out.stage = Stage::Silent;
      out.toneHz = 0;
      return out;
    }
  }

  const float h = lastValidCm_;

  // Arming. A foot-mounted sensor reads the floor at ~10 cm from the car park to
  // the aircraft door, which on height alone is indistinguishable from "below the
  // changeover height" -- without this the device howls from boarding to exit.
  //
  // The height test applies to the START of the confirmation window, not the end:
  // at 6.7 m/s a unit that acquires at 13 m is already below 8 m one second later,
  // so an end-of-window test would never arm on a fast canopy, which is exactly
  // the case that needs the warning most.
  //
  // 中文：布防的意义是让设备在地面上闭嘴。装在脚上的传感器从停车场到机舱门
  //       一直读到 10 cm 左右的地面，光看高度和「已经低于转换高度」无法区分。
  //       「够高」判的是确认窗口**开始**时的高度：6.7 m/s 下 13 m 捕获、
  //       一秒后已经掉到 6.3 m，拿窗口结束时去比 8 m 的话快伞永远布防不上 ——
  //       而快伞恰恰最需要提前警告。
  if (!armed_) {
    // Reopen the window whenever the current height is ABOVE where the window
    // started. Nothing that is higher than it was can have been descending since
    // then, so the window is measuring a descent that did not happen, and a stale
    // low starting height fails the "high enough" test for up to a full
    // confirmation window before sliding -- burning the first step of a real
    // descent for no reason.
    //
    // 中文：只要当前高度比窗口起点还高，就重开窗口。比开始时更高的东西不可能
    //       「自那时起一直在下降」，这个窗口量的是一段不存在的下降。
    //       不重开的话，一个陈旧的低起点会让「够高」这条判据一直失败，
    //       最多要等满一整个确认窗口才会滑走 —— 白白吃掉真实下降的第一步。
    if (!armWindowOpen_ || h > armWindowCm_) {
      armWindowOpen_ = true;
      armWindowMs_ = nowMs;
      armWindowCm_ = h;
    }
    if ((nowMs - armWindowMs_) >= cfg_.armConfirmMs) {
      const bool highEnough = armWindowCm_ > cfg_.armMinCm;
      const bool descending = (armWindowCm_ - h) >= cfg_.armMinDropCm;
      if (highEnough && descending) {
        armed_ = true;
        reseatLadder(h);
        belowStopSinceMs_ = 0;
      } else {
        armWindowOpen_ = false;   // slide the window and keep watching
      }
    }
    if (!armed_) {
      stage_ = Stage::Silent;
      out.stage = stage_;
      out.toneHz = 0;
      out.armed = false;
      return out;
    }
  }

  // Auto-stop, checked before the tone so silence wins over sound.
  // 中文：自动停止判断放在发声之前，让「停」优先于「响」。
  if (h <= cfg_.stopBelowCm) {
    if (belowStopSinceMs_ == 0) belowStopSinceMs_ = nowMs;
    if ((nowMs - belowStopSinceMs_) > cfg_.stopHoldMs) {
      disarm();
      out.armed = false;
      out.stage = Stage::Silent;
      out.toneHz = 0;
      return out;
    }
  } else {
    belowStopSinceMs_ = 0;
  }

  if (h <= cfg_.continuousAtCm) {
    stage_ = Stage::FlareNow;
    out.stage = stage_;
    out.armed = true;
    out.toneHz = cfg_.pitchHighHz;
    return out;
  }

  if (h <= nextBeepAtCm_) {
    beepHz_ = pitchForHeightHz(cfg_, h);
    beepUntilMs_ = nowMs + cfg_.beepMs;
    out.beepFired = true;
    out.beepAtCm = h;

    // Seat the next threshold relative to the CURRENT height, not one step below
    // the old threshold. While descending smoothly the two are identical -- a
    // 4.5-6.7 cm tick never skips a 50 cm step. They diverge only on a
    // discontinuity, e.g. signal lost at 10 m and reacquired at 4 m, where
    // decrementing would fire one beep per tick until the ladder caught up:
    // a burst of beeps claiming six 50 cm descents that were never observed.
    // One jump is one beep.
    //
    // 中文：下一个阈值以**当前高度**为基准落位，而不是拿旧阈值减一步。
    //       平滑下降时两者完全等价 —— 每个 tick 只掉 4.5-6.7 cm，不可能跨过 50 cm 的格子。
    //       只有高度发生跳变时才不同：比如 10 m 丢信号、4 m 才重新捕获，
    //       用「减一步」的写法会一个 tick 响一声地追下去，
    //       发出一串声称「掉了六个 50 cm」的爆音 —— 而那六段下降根本没有被观测到。
    //       一次跳变就该只响一声。
    nextBeepAtCm_ = floorf(h / cfg_.beepStepCm) * cfg_.beepStepCm;
    if (nextBeepAtCm_ >= h) nextBeepAtCm_ -= cfg_.beepStepCm;
  } else if (h > nextBeepAtCm_ + 2.0f * cfg_.beepStepCm) {
    // Climbed back up -- a thermal, a bad frame, or the foot swinging over a
    // different patch of ground. Re-seat rather than staying latched low, which
    // would silently swallow every beep on the way back down.
    // 中文：高度又涨回去了。重新对齐刻度，否则刻度一直卡在低处，
    //       再降下来时所有响声都会被无声吞掉 —— 不报错，只是不响。
    reseatLadder(h);
  }

  out.toneHz = (nowMs < beepUntilMs_) ? beepHz_ : 0;
  out.beepHz = beepHz_;
  stage_ = stageForHeight(cfg_, h);
  out.stage = stage_;
  out.armed = true;
  return out;
}

}  // namespace flare
