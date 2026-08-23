// Flare Altimeter -- foot-mounted laser altimeter for parachute flare timing.
//
// A DIY *validation* device, NOT a life-critical instrument. It cross-checks flare
// timing near the ground; nothing here should ever be relied on for a safety
// decision. Keep that framing.
//
// This file is only wiring: setup, the main loop, and the serial console. The
// decisions live in testable modules --
//   lib/flare_math    quaternions, tilt, slant-range -> height
//   lib/tf02          TF02-Pro frame decoding and the validity gate
//   lib/alert         the beep ladder, arming, auto-stop
//   lib/flare_config  settings validation
//   src/hw_*          the parts that need real hardware
//
// Wiring: see src/pins.h.
//
// Serial commands (115200, accepted from either USB route):
//   h  human view (10 Hz)          c  CSV stream (50 Hz) for tools/imu_viewer.py
//   r  re-run all diagnostics      l  re-check the laser
//   t  store the current pose as the mounting reference (writes NVS)
//   z  clear that reference        1/0  tilt correction on/off
//   b  buzzer sweep 500-5000 Hz    m  mute toggle          s  status
//   +/-  changeover height +/-25 cm       k  beep step 25/50/100 cm
//   d  simulated descent, slow     D  simulated descent, real canopy speed
//   X<dist_cm>,<strength>  inject one laser frame (replay)
//   Y<cm>                  set the changeover height outright
//
// 中文说明：
//   这是跳伞拉飘高度计。定位是 DIY 验证装置，不是性命攸关的仪表 ——
//   它只用来事后核对拉飘时机，任何时候都不该拿它做安全决策。这个前提请一直保留。
//
//   本文件只负责接线：初始化、主循环、串口命令。真正的判断逻辑都在可测试的模块里，
//   lib/ 下面的四个模块不依赖 Arduino，可以在电脑上直接跑单元测试。
#include <Arduino.h>

#include "alert_engine.h"
#include "flare_config.h"
#include "flare_math.h"
#include "hw_buzzer.h"
#include "hw_console.h"
#include "hw_diag.h"
#include "hw_imu.h"
#include "hw_lidar.h"
#include "pins.h"
#include "settings.h"
#include "sim.h"

TeeStream Out;

namespace {

flare::AlertEngine alerts{flare::AlertConfig{}};

bool csvMode = false;
uint32_t lastPrintMs = 0;
uint8_t imuHintAddress = 0;

// Latest attitude/height, refreshed at sensor rate rather than print rate.
flare::Quat attitude{1.0f, 0.0f, 0.0f, 0.0f};
uint8_t fusionAccuracy = 0;
float tiltDeg = 0.0f;
float heightCm = -1.0f;
int32_t distCm = -1;

flare::Stage lastSimStage = flare::Stage::Silent;

void syncAlertConfig() {
  alerts.config().continuousAtCm = settings::s.continuousAtCm;
  alerts.config().beepStepCm = settings::s.beepStepCm;
}

void runDiagnostics() {
  hw::probeLines();
  imuHintAddress = hw::scanI2C();
  hw::lidarDiagnose();
  const bool ok = hw::imuStart(imuHintAddress);
  if (ok) {
    Out.printf("\nStreaming rotation vector @ %u ms. Commands: h c r t s b d D\n",
               IMU_REPORT_INTERVAL_MS);
    Out.println(F("Rotate the board slowly on all 3 axes to settle the fusion.\n"));
  } else {
    Out.println(F("\nIMU not found. Fix wiring; retrying every 2 s (or press 'r').\n"));
  }
}

void printStatus() {
  const flare::AlertConfig &cfg = alerts.config();
  Out.printf("\n[status] reference: %s   tilt correction: %s   buzzer: %s\n",
             settings::s.refStored ? "stored" : "none",
             settings::s.tiltCorrection ? "on" : "OFF",
             hw::buzzMuted() ? "muted" : "unmuted");
  Out.printf("         alert: %s   armed: %s   h=%.1f cm (%s)   fusion acc=%u\n",
             flare::stageName(alerts.stage()), alerts.armed() ? "YES" : "no",
             heightCm, distCm >= 0 ? "valid" : "no valid height",
             hw::imuAccuracy());
  Out.printf("         beep every %.0f cm, %u->%u Hz over %.0f..%.0f cm;"
             " continuous below %.0f cm; auto-stop %.0f s under %.0f cm\n",
             cfg.beepStepCm, cfg.pitchLowHz, cfg.pitchHighHz, cfg.alertTopCm,
             cfg.continuousAtCm, cfg.continuousAtCm, cfg.stopHoldMs / 1000.0f,
             cfg.stopBelowCm);
  Out.printf("         arming needs: >%.0f cm, descending >=%.0f cm,"
             " %lu ms of continuous data\n",
             cfg.armMinCm, cfg.armMinDropCm, (unsigned long)cfg.armConfirmMs);
}

// A bare piezo disc has a sharp mechanical resonance and is many times louder
// there than 200 Hz either side of it. Where that peak sits depends on the disc
// AND on how it ends up mounted, so it has to be found by ear rather than assumed.
// 中文：发声器件有很尖锐的机械谐振点，谐振频率上的声压比偏开 200 Hz 大好几倍。
//       这个峰在哪取决于器件本身和最后怎么装，抄数据手册没用，只能扫一遍用耳朵找。
void buzzerSweep() {
  Out.println(F("\n[buzz] sweeping 500..5000 Hz -- note where it is loudest"));
  for (uint16_t hz = 500; hz <= 5000 && !hw::buzzMuted(); hz += 100) {
    hw::buzzTone(hz);
    if (hz % 500 == 0) Out.printf("  %u Hz\n", hz);
    const uint32_t until = millis() + 60;
    while ((int32_t)(millis() - until) < 0) hw::lidarPoll();  // never stall the laser
  }
  hw::buzzTone(0);
  Out.println(F("[buzz] sweep done"));
}

void handleCommand(char c) {
  switch (c) {
    case 'c':
      csvMode = true;
      Out.println(F("t_ms,qw,qi,qj,qk,roll,pitch,yaw,tilt,acc,dist_cm,strength,"
                    "h_cm,corr,stage"));
      break;
    case 'h':
      csvMode = false;
      Out.println(F("\n[human view]"));
      break;
    case 'l':
      Out.println(F("\n[lidar]"));
      hw::lidarDiagnose();
      break;
    case 'r':
      Out.println(F("\n[rescan]"));
      runDiagnostics();
      break;
    case 't': {
      const char *why = "";
      if (hw::imuCaptureReference(why)) {
        Out.printf("\n[ref] %s\n", why);
      } else {
        Out.printf("\n[ref] REFUSED: %s\n", why);
      }
      break;
    }
    case 'z':
      settings::s.mountingRef = {1.0f, 0.0f, 0.0f, 0.0f};
      settings::s.refStored = false;
      settings::save();
      Out.println(F("\n[ref] cleared -- back to the sensor's own frame"));
      break;
    case '1':
    case '0':
      settings::s.tiltCorrection = (c == '1');
      settings::save();
      Out.printf("\n[corr] tilt correction %s\n",
                 settings::s.tiltCorrection ? "ON" : "OFF");
      break;
    case '+':
    case '-':
      settings::s.continuousAtCm =
          flare::adjustContinuousAtCm(settings::s.continuousAtCm, c == '+');
      syncAlertConfig();
      settings::save();
      Out.printf("\n[cfg] continuous tone starts at %.0f cm at the device"
                 "  (~%.0f-%.0f cm at eye level)\n",
                 settings::s.continuousAtCm, settings::s.continuousAtCm + 50.0f,
                 settings::s.continuousAtCm + 100.0f);
      break;
    case 'k':
      // Re-seat the ladder immediately so the change is audible on the very next
      // beep instead of after the current step finishes -- that delay is very
      // misleading when comparing step sizes by ear.
      settings::s.beepStepCm = flare::cycleBeepStepCm(settings::s.beepStepCm);
      syncAlertConfig();
      if (alerts.armed() && heightCm > 0) alerts.reseatLadder(heightCm);
      settings::save();
      Out.printf("\n[cfg] one beep per %.0f cm"
                 "  (at 4.5 m/s that is %.1f beeps/s; at 6.7 m/s, %.1f)\n",
                 settings::s.beepStepCm,
                 flare::beepsPerSecond(settings::s.beepStepCm, 450.0f),
                 flare::beepsPerSecond(settings::s.beepStepCm, 670.0f));
      break;
    case 'm':
      hw::buzzSetMuted(!hw::buzzMuted());
      Out.printf("\n[buzz] %s\n", hw::buzzMuted() ? "MUTED" : "unmuted");
      break;
    case 'b':
      buzzerSweep();
      break;
    case 's':
      printStatus();
      break;
    case 'd':
      lastSimStage = alerts.stage();
      sim::startSlow();
      break;
    case 'D':
      lastSimStage = alerts.stage();
      sim::startReal();
      break;
    default:
      break;
  }
}

// Console input is single characters, except for replay frames, which need two
// numbers. 'X' switches to collecting a line; everything else stays a one-shot
// command, so nothing that worked before changes.
//
//   X<dist_cm>,<strength>\n     e.g.  X1832,1450
//
// 中文：串口输入本来是单字符命令，只有回放帧需要带两个数字。
//       'X' 切进整行收集模式，其它字符仍然是一按即执行 —— 原有命令行为完全不变。
char injectBuf[24];
uint8_t injectLen = 0;
char collectingCmd = 0;      // 0 = not collecting; otherwise the leading letter

// Y<cm> sets the changeover height outright, which '+' / '-' cannot: they step by
// 25 cm, so reaching an arbitrary value from a UI text box would mean sending a
// computed number of keystrokes and hoping none were dropped.
// 中文：Y<cm> 直接设定转换高度。'+' / '-' 做不到这件事 —— 它们每次走 25 cm，
//       要从界面输入框到达一个任意值，就得算出该按几下再祈祷一下都没丢。
void parseSetChangeover(const char *s, uint8_t len) {
  char tmp[24];
  if (len == 0 || len >= sizeof(tmp)) return;
  memcpy(tmp, s, len);
  tmp[len] = '\0';
  settings::s.continuousAtCm = flare::clampContinuousAtCm((float)atol(tmp));
  syncAlertConfig();
  settings::save();
  Out.printf("\n[cfg] continuous tone starts at %.0f cm at the device"
             "  (~%.0f-%.0f cm at eye level)\n",
             settings::s.continuousAtCm, settings::s.continuousAtCm + 50.0f,
             settings::s.continuousAtCm + 100.0f);
}

void parseInject(const char *s, uint8_t len) {
  int32_t dist = -1;
  long strength = 0;
  char tmp[24];
  if (len >= sizeof(tmp)) return;
  memcpy(tmp, s, len);
  tmp[len] = '\0';
  char *comma = strchr(tmp, ',');
  if (!comma) return;
  *comma = '\0';
  dist = (int32_t)atol(tmp);
  strength = atol(comma + 1);
  if (strength < 0) strength = 0;
  if (strength > 65535) strength = 65535;
  hw::lidarInject(dist, (uint16_t)strength);
}

void consoleChar(char c) {
  if (collectingCmd) {
    if (c == '\n' || c == '\r') {
      if (collectingCmd == 'X') parseInject(injectBuf, injectLen);
      else if (collectingCmd == 'Y') parseSetChangeover(injectBuf, injectLen);
      collectingCmd = 0;
      injectLen = 0;
    } else if (injectLen < sizeof(injectBuf) - 1) {
      injectBuf[injectLen++] = c;
    } else {
      collectingCmd = 0;           // overlong line: drop it rather than truncate
      injectLen = 0;
    }
    return;
  }
  if (c == 'X' || c == 'Y') {
    collectingCmd = c;
    injectLen = 0;
    return;
  }
  handleCommand(c);
}

void drawGauge(char *buf, size_t len, float value, float fullScale) {
  const int width = 21;             // odd => a true centre cell
  const int centre = width / 2;
  const int pos =
      centre + (int)roundf(constrain(value / fullScale, -1.0f, 1.0f) * centre);
  int i = 0;
  for (; i < width && (size_t)i < len - 1; i++) {
    buf[i] = (i == pos) ? '#' : (i == centre ? '|' : '-');
  }
  buf[i] = '\0';
}

}  // namespace

void setup() {
  Out.begin(115200);
  delay(300);
  Out.println(F("\n\n=== Flare Altimeter :: IMU bring-up ==="));

  hw::i2cBegin();
  hw::lidarBegin();
  hw::buzzBegin();
  hw::imuBegin();

  settings::load();
  syncAlertConfig();
  Out.printf("mounting reference: %s   tilt correction: %s\n",
             settings::s.refStored ? "stored (loaded from NVS)"
                                   : "none (raw sensor frame)",
             settings::s.tiltCorrection ? "on" : "OFF");

  runDiagnostics();
}

void loop() {
  while (Out.available()) consoleChar((char)Out.read());

  // Before any early return below: at 100 Hz the laser fills the 256-byte RX FIFO
  // in under three seconds of not being read.
  hw::lidarPoll();

  const uint32_t now = millis();

  // Fresh attitude, at the sensor's rate rather than the console's.
  if (hw::imuReady()) {
    flare::Quat q;
    uint8_t acc;
    if (hw::imuPoll(q, acc)) {
      attitude = q;
      fusionAccuracy = acc;
      tiltDeg = flare::tiltFromVerticalDeg(q);
    }
  }

  // A stale reading is worse than none: report -1 once the stream stops rather
  // than freezing on the last distance the sensor happened to see.
  distCm = hw::lidarOnline() ? hw::lidarDistCm() : -1;
  heightCm = flare::heightFromSlantCm(
      distCm, settings::s.tiltCorrection ? tiltDeg : 0.0f);

  // Alert input: the simulator when it is running, the sensors otherwise. The
  // validity gate is the project's -- no echo, or a beam more than 40 deg off
  // vertical, means there is no height this frame.
  float alertHeight = heightCm;
  bool alertValid = hw::imuReady() && distCm >= 0 &&
                    (!settings::s.tiltCorrection || tiltDeg <= TILT_GATE_DEG);
  const bool simRunning = sim::update(now, alertHeight, alertValid);

  const flare::AlertOutput out = alerts.update(now, alertHeight, alertValid);
  hw::buzzTone(out.toneHz);

  // Beep events are echoed during a replay as well as a simulation. Height alone
  // cannot show what the ear actually receives: the whole design rests on beep
  // RATE carrying descent rate, and a rate can only be checked by counting real
  // beep events against the clock. Without these lines a test can confirm the
  // heights were right while the sound was silent or machine-gunning.
  // 中文：回放时也把 beep 事件打出来。光看高度看不出耳朵实际收到了什么 ——
  //       整个设计的立足点是「响声频率携带下降速率」，而频率只能靠
  //       数真实的响声事件除以时间来验证。没有这几行，测试可以确认高度全对，
  //       而声音其实是哑的、或者在扫射。
  if (simRunning || hw::lidarReplayActive()) {
    if (out.beepFired) {
      const float tSec = simRunning ? sim::elapsedMs(now) / 1000.0f
                                    : now / 1000.0f;
      Out.printf("  beep  %6.0f cm  %4u Hz   (t+%.2f s)\n", out.beepAtCm,
                 out.beepHz, tSec);
    }
  }

  if (simRunning) {
    // Report the stage the alarm is ACTUALLY in, not what the height alone
    // implies. Those differ exactly where it matters -- while disarmed, and after
    // the auto-stop -- so printing the height-derived stage would hide both.
    if (out.stage != lastSimStage) {
      lastSimStage = out.stage;
      Out.printf("  %6.0f cm  ->  %s   (t+%.2f s)\n", alertHeight,
                 flare::stageName(out.stage), sim::elapsedMs(now) / 1000.0f);
    }
  }

  if (!hw::imuReady()) {
    static uint32_t lastRetry = 0;
    if (now - lastRetry > 2000) {
      lastRetry = now;
      runDiagnostics();
    }
    return;
  }

  if (now - lastPrintMs < (csvMode ? PRINT_CSV_MS : PRINT_HUMAN_MS)) return;
  lastPrintMs = now;

  float rollRad, pitchRad, yawRad;
  flare::quatToRPY(attitude, rollRad, pitchRad, yawRad);
  const float roll = degrees(rollRad);
  const float pitch = degrees(pitchRad);
  const float yaw = degrees(yawRad);

  if (csvMode) {
    Out.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%u,%ld,%u,%.1f,%u,%u\n",
               (unsigned long)now, attitude.w, attitude.x, attitude.y, attitude.z,
               roll, pitch, yaw, tiltDeg, fusionAccuracy, (long)distCm,
               hw::lidarOnline() ? hw::lidarStrength() : 0, heightCm,
               settings::s.tiltCorrection ? 1u : 0u, (unsigned)out.stage);
  } else {
    static const char *accName[4] = {"unreliable", "low", "medium", "high"};
    char rollBar[24], pitchBar[24];
    drawGauge(rollBar, sizeof(rollBar), roll, 90.0f);
    drawGauge(pitchBar, sizeof(pitchBar), pitch, 90.0f);
    Out.printf("R%+7.1f %s  P%+7.1f %s  Y%+7.1f  tilt%6.1f  [%s]\n", roll,
               rollBar, pitch, pitchBar, yaw, tiltDeg,
               accName[fusionAccuracy & 0x03]);
    if (distCm < 0) {
      Out.printf("    laser: %s\n",
                 hw::lidarOnline() ? "no valid echo" : "OFFLINE (no frames)");
    } else {
      Out.printf("    laser: d%6ld cm  S%6u  ->  h%7.1f cm  (%.2f m)  [corr %s]"
                 "  alert: %s%s\n",
                 (long)distCm, hw::lidarStrength(), heightCm, heightCm / 100.0f,
                 settings::s.tiltCorrection ? "on" : "OFF",
                 flare::stageName(out.stage), hw::buzzMuted() ? " (MUTED)" : "");
    }
  }
}
