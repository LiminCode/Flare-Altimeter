// Benewake TF02-Pro frame parser.
//
// Frame: 59 59 DistL DistH StrL StrH TempL TempH Checksum   (9 bytes, 100 Hz)
// Checksum is the low byte of the sum of the first eight.
//
// Hand-rolled rather than using TFMPlus so the validity gate and the re-sync
// behaviour are ours -- both are safety-relevant and both are things a library
// would decide differently.
//
// 中文说明：
//   TF02-Pro 的 9 字节定长帧，帧头两个 0x59，校验和 = 前 8 字节之和的低 8 位。
//   自己写而不用 TFMPlus 库，是因为「什么算有效读数」和「丢字节后怎么重新对齐」
//   这两件事都和安全相关，而库会按它自己的想法决定。
#pragma once

#include <stdint.h>

namespace flare {

struct LidarReading {
  int32_t distCm;     // -1 = frame received but the range is not trustworthy
  uint16_t strength;
  float tempC;
};

// Strength outside this band means the frame's distance is meaningless. The sensor
// keeps emitting frames in both cases, with numbers that look ordinary -- so this
// gate is the only thing standing between a dark or oblique target and a confident
// wrong altitude.
// 中文：强度落在这个区间之外，这一帧的距离就没有意义。传感器在这两种情况下
//       照样吐帧，数值看上去也很正常 —— 这道判据是「目标太暗/太斜」和
//       「一个自信的错误高度」之间唯一的东西。
constexpr uint16_t kStrengthMin = 100;      // below: too far, too dark, too oblique
constexpr uint16_t kStrengthSaturated = 65535;  // above range: too close/reflective

class Tf02Parser {
 public:
  // Feed one byte. Returns true when a checksum-valid frame just completed, with
  // `out` filled in. A frame that fails checksum returns false and bumps the
  // bad-checksum counter.
  bool feed(uint8_t byte, LidarReading &out);

  void reset();

  uint32_t frames() const { return frames_; }
  uint32_t badChecksums() const { return badChecksums_; }

 private:
  uint8_t buf_[9] = {0};
  uint8_t len_ = 0;
  uint32_t frames_ = 0;
  uint32_t badChecksums_ = 0;
};

}  // namespace flare
