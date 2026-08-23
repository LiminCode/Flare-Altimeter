#include "hw_lidar.h"

#include <Arduino.h>

#include "hw_console.h"
#include "pins.h"
#include "tf02_parser.h"

namespace hw {
namespace {

flare::Tf02Parser parser;
flare::LidarReading latest{-1, 0, 0.0f};
uint32_t lastFrameMs = 0;

uint32_t lastInjectMs = 0;
bool replaying = false;

}  // namespace

bool lidarReplayActive() {
  if (replaying && (millis() - lastInjectMs) > LIDAR_REPLAY_TIMEOUT_MS) {
    replaying = false;
    Out.println(F("[replay] feeder stopped -- back to live laser data"));
  }
  return replaying;
}

void lidarInject(int32_t distCm, uint16_t strength) {
  if (!replaying) {
    replaying = true;
    Out.println(F("[replay] injecting laser frames -- live sensor ignored"));
  }
  lastInjectMs = millis();
  // Apply the same validity gate the parser applies, so an injected weak return
  // is rejected for exactly the reason a real one would be. Injecting a
  // pre-decided distCm would quietly bypass the gate under test.
  // 中文：注入的帧也要过和真实帧完全相同的有效性判据 ——
  //       直接塞一个已经决定好的距离，等于把正要被测试的那道判据绕过去了。
  latest.strength = strength;
  latest.distCm = (strength < flare::kStrengthMin ||
                   strength == flare::kStrengthSaturated || distCm < 0)
                      ? -1
                      : distCm;
  lastFrameMs = millis();
}

void lidarBegin() {
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);
}

void lidarReattach() {
  Serial1.end();
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);
}

void lidarPoll() {
  const bool discard = lidarReplayActive();
  flare::LidarReading r{};
  while (Serial1.available()) {
    // Drained even while replaying: leaving the bytes in the FIFO would overflow
    // it in under three seconds and corrupt the first real frames afterwards.
    // 中文：即使在回放中也要照常把字节读掉 —— 留在 FIFO 里不到三秒就会溢出，
    //       回放结束后最先到的那几帧真实数据会跟着一起坏掉。
    if (parser.feed((uint8_t)Serial1.read(), r) && !discard) {
      latest = r;
      lastFrameMs = millis();
    }
  }
}

uint32_t lidarListen(uint32_t ms) {
  const uint32_t before = parser.frames();
  const uint32_t until = millis() + ms;
  while ((int32_t)(millis() - until) < 0) lidarPoll();
  return parser.frames() - before;
}

bool lidarOnline() {
  return parser.frames() > 0 && (millis() - lastFrameMs) < LIDAR_STALE_MS;
}

int32_t lidarDistCm() { return latest.distCm; }
uint16_t lidarStrength() { return latest.strength; }
float lidarTempC() { return latest.tempC; }
uint32_t lidarFrames() { return parser.frames(); }
uint32_t lidarBadChecksums() { return parser.badChecksums(); }

// The sensor needs no handshake -- it either streams or it does not. 300 ms is
// ~30 frames at its default 100 Hz, so silence here is a real wiring answer and
// not bad luck with timing.
//
// 中文说明：
//   TF02-Pro 不需要任何握手，上电就自己吐数据，所以「有没有帧」直接等价于
//   「线接对没接对」。等 300 ms 是因为默认 100 Hz，这段时间足够收到约 30 帧，
//   一帧都没有就不可能是时序凑巧，而是真的没接通。
//   「收到帧但校验全错」和「完全没帧」是两种不同的病，分开报。
void lidarDiagnose() {
  Out.println(F("--- TF02-Pro ---"));
  const uint32_t badBefore = parser.badChecksums();
  const uint32_t got = lidarListen(300);
  const uint32_t bad = parser.badChecksums() - badBefore;

  if (got > 0) {
    Out.printf("  %lu frames in 300 ms (%lu checksum errors)\n",
               (unsigned long)got, (unsigned long)bad);
    if (latest.distCm >= 0) {
      Out.printf("  dist %ld cm  strength %u  temp %.1f C\n",
                 (long)latest.distCm, latest.strength, latest.tempC);
    } else {
      Out.printf("  streaming, but no valid range (strength %u -- aim at a"
                 " surface 0.5-5 m away)\n", latest.strength);
    }
    return;
  }

  if (bad > 0) {
    Out.println(F("  bytes arriving but every checksum fails -- wrong baud rate?"));
    return;
  }

  // Silent. Rather than hand back a checklist, listen on the other wire: a
  // swapped green/white pair is the classic mistake and the board can rule it in
  // or out by itself, because an RX pin is a pure input and moving the listener
  // in software needs no re-wiring at all.
  //
  // 中文说明：
  //   一帧都没有的时候，与其甩给用户一张检查清单，不如让板子自己去另一根线上听。
  //   绿白接反是最常见的错误，而 UART 的 RX 是纯输入 ——
  //   换个引脚监听不需要动任何硬件，也不会损坏任何东西，几百毫秒就有确定答案。
  //   听完必须还原引脚，否则后面的正常读取会挂在错误的脚上。
  Out.printf("  nothing on GPIO%d -- listening on GPIO%d instead...\n",
             PIN_LIDAR_RX, PIN_LIDAR_TX);
  Serial1.end();
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, PIN_LIDAR_TX, PIN_LIDAR_RX);
  parser.reset();
  const uint32_t swapped = lidarListen(300);
  lidarReattach();
  parser.reset();

  if (swapped > 0) {
    Out.printf("  FOUND IT: %lu frames on GPIO%d. Green and white are swapped"
               " -- move green to GPIO%d.\n",
               (unsigned long)swapped, PIN_LIDAR_TX, PIN_LIDAR_RX);
    return;
  }

  Out.println(F("  silent on both pins, so this is power or ground, not the"
                " signal pair:"));
  Out.println(F("    - a powered TF02-Pro drives its TX high when idle; the"
                " line probe above showing"));
  Out.println(F("      'stuck low' or 'floating' on the green wire means the"
                " sensor is not running."));
  Out.println(F("    - red must reach 5V (VBUS), not 3V3 -- it will not start"
                " on 3.3 V."));
  Out.println(F("    - black must share ground with the board."));
  Out.println(F("    - some clone boards only feed the 5V pin from the NATIVE"
                " USB port; if red is"));
  Out.println(F("      on such a pin, plugging the second USB cable in for"
                " power is the quick test."));
}

}  // namespace hw
