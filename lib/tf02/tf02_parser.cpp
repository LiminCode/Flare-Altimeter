#include "tf02_parser.h"

namespace flare {

void Tf02Parser::reset() {
  len_ = 0;
}

bool Tf02Parser::feed(uint8_t byte, LidarReading &out) {
  // Header sync. While the two header bytes are still being collected, anything
  // that is not 0x59 throws the partial frame away. That is what lets the parser
  // recover by itself after a dropped byte instead of staying permanently one
  // byte out of phase and rejecting every frame from then on.
  // 中文：帧头同步。还没凑齐两个 0x59 之前，遇到别的字节就整帧丢弃重来。
  //       这正是掉一个字节之后解析器能自己对回来的原因 ——
  //       否则它会永久错位一个字节，之后每一帧都校验失败。
  if (len_ < 2) {
    if (byte == 0x59) {
      buf_[len_++] = byte;
    } else {
      len_ = 0;
    }
    return false;
  }

  buf_[len_++] = byte;
  if (len_ < sizeof(buf_)) return false;
  len_ = 0;

  uint16_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += buf_[i];
  if ((sum & 0xFF) != buf_[8]) {
    badChecksums_++;
    return false;
  }

  const uint16_t dist = (uint16_t)buf_[2] | ((uint16_t)buf_[3] << 8);
  const uint16_t strength = (uint16_t)buf_[4] | ((uint16_t)buf_[5] << 8);
  const uint16_t rawTemp = (uint16_t)buf_[6] | ((uint16_t)buf_[7] << 8);

  out.strength = strength;
  out.tempC = rawTemp / 8.0f - 256.0f;
  out.distCm = (strength < kStrengthMin || strength == kStrengthSaturated ||
                dist == 0xFFFF)
                   ? -1
                   : (int32_t)dist;

  // Counted even when the range is rejected: this counter answers "is the sensor
  // talking to us", which is a different question from "can we see the ground",
  // and conflating them makes a wiring fault look like a reflectivity problem.
  // 中文：即使距离被判为无效也照样计数。这个计数器回答的是「传感器还在不在说话」，
  //       和「能不能看到地面」是两个不同的问题 ——
  //       混在一起会让一个接线故障看起来像是地面反射率不够。
  frames_++;
  return true;
}

}  // namespace flare
