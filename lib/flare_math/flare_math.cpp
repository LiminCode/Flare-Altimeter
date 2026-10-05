#include "flare_math.h"

#include <math.h>

namespace flare {
namespace {

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

constexpr float kRadToDeg = 57.29577951308232f;

}  // namespace

Quat qConj(const Quat &q) {
  return {q.w, -q.x, -q.y, -q.z};
}

Quat qMul(const Quat &a, const Quat &b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Quat applyReference(const Quat &raw, const Quat &ref) {
  return qMul(raw, qConj(ref));
}

void quatToRPY(const Quat &q, float &rollRad, float &pitchRad, float &yawRad) {
  const float ysqr = q.y * q.y;
  rollRad = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                   1.0f - 2.0f * (q.x * q.x + ysqr));
  const float t2 = clampf(2.0f * (q.w * q.y - q.z * q.x), -1.0f, 1.0f);
  pitchRad = asinf(t2);
  yawRad = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                  1.0f - 2.0f * (ysqr + q.z * q.z));
}

float tiltFromVerticalDeg(const Quat &q) {
  // R33 of the rotation matrix is the dot product of body +Z with world +Z, i.e.
  // the cosine of the angle between them -- no need to go via Euler angles, which
  // would drag in gimbal lock and wrap-around for no gain.
  //
  // The clamp is not cosmetic: float error can push R33 to 1.0000001, and acosf()
  // outside its domain returns NaN, which then silently poisons every downstream
  // calculation instead of failing loudly.
  // 中文：clamp 不是装饰。浮点误差可能让 R33 变成 1.0000001，
  //       acosf() 拿到定义域外的输入会返回 NaN，而 NaN 会一路无声地污染
  //       后面所有计算 —— 不会报错，只会让结果全变成 NaN。
  float r33 = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
  r33 = clampf(r33, -1.0f, 1.0f);
  return acosf(r33) * kRadToDeg;
}

float heightFromSlantCm(int32_t distCm, float tiltDeg) {
  if (distCm < 0) return -1.0f;
  // At 90 deg the beam is horizontal and cos() says h = 0, "on the ground"; past
  // it the beam points up and h goes negative. Neither is a height. Seen
  // 2026-10-03 with no mounting reference stored: a replayed 20 m descent came
  // out as -1.27 m .. -0.01 m.
  // 中文：90° 时光束水平，cos 给出 h = 0，即「已经贴地」；超过 90° 光束朝上，
  //       h 变成负数。两者都不是高度。2026-10-03 没存安装基准时实测：
  //       回放一段 20 米下降，算出来的高度是 -1.27 m 到 -0.01 m。
  if (tiltDeg >= 90.0f) return -1.0f;
  return (float)distCm * cosf(tiltDeg / kRadToDeg);
}

}  // namespace flare
