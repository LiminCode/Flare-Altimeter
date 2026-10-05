// Attitude and geometry maths for the flare altimeter.
//
// Deliberately free of Arduino.h so it compiles and runs on the host: this is the
// layer where a sign error or a swapped multiplication order produces an answer
// that is wrong by a few percent and looks entirely plausible on a serial log.
// That class of bug is only caught by asserting against known values, which needs
// a test runner, which needs the code to build off-target.
//
// 中文说明（这个模块为什么必须能脱离硬件编译）：
//   这里是最容易出「看起来完全合理但就是错了几个百分点」的地方 ——
//   一个符号写反、四元数乘法顺序颠倒，串口上打出来的数字依然像模像样。
//   这种错误只能靠「拿已知输入断言已知输出」来抓，而那需要一个测试框架，
//   也就要求这些代码能在电脑上直接编译运行，不依赖 Arduino。
#pragma once

#include <stdint.h>

namespace flare {

struct Quat {
  float w, x, y, z;
};

Quat qConj(const Quat &q);
Quat qMul(const Quat &a, const Quat &b);

// Remove a stored mounting pose from a live one: q_rel = q ⊗ q_ref⁻¹.
//
// The order matters and fails quietly if reversed -- a 15° swing reads 14.71°
// instead of 15.00°, small enough to never look like a bug and permanent enough
// to bias every height the device ever reports.
// 中文：把安装基准从当前姿态里除掉。乘法顺序写反不会报错，只会让 15° 读成
//       14.71° —— 小到永远不会被当成 bug，却会永久地把每一次高度都算偏。
Quat applyReference(const Quat &raw, const Quat &ref);

// Euler angles in radians, matching the SparkFun BNO08x library's own formulas so
// that switching to this code changes no printed number.
void quatToRPY(const Quat &q, float &rollRad, float &pitchRad, float &yawRad);

// Angle between the body +Z axis and world vertical, in degrees. This is the theta
// in h = d * cos(theta).
float tiltFromVerticalDeg(const Quat &q);

// Slant range -> vertical height. Returns -1 for "no reading", never 0: downstream,
// 0 means "on the ground", which is the most dangerous possible default.
// A beam at or above horizontal (tilt >= 90 deg) is also "no reading".
// 中文：斜距换算竖直高度。没有读数时返回 -1 而不是 0 ——
//       下游把 0 理解成「已经贴地」，那是最危险的默认值。
float heightFromSlantCm(int32_t distCm, float tiltDeg);

}  // namespace flare
