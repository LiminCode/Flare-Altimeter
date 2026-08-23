#include "hw_imu.h"

#include <Arduino.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Wire.h>

#include "hw_console.h"
#include "pins.h"
#include "settings.h"

namespace hw {
namespace {

BNO08x imu;
uint8_t address = 0;
bool ready = false;
uint8_t accuracy = 0;

// Pulse the reset line. The BNO08x is much happier after a clean hardware reset
// than after a bare power-on, especially on breakout wiring.
//
// 中文说明：
//   BNO08x 内部跑着一颗 Cortex M0+，上电时序不干净时（杜邦线接触抖动、3V3 上升沿慢）
//   它可能停在半初始化状态：I2C 地址扫得到，但 begin() 就是不成功。
//   拉一下 RST 做一次明确的硬复位，比单纯依赖上电复位可靠得多。
//   低电平保持 10 ms 保证复位被识别，松开后等 200 ms 是留给那颗 M0+ 跑完启动固件 ——
//   这段时间内发 I2C 过去它不会理你。
void hardReset() {
  if (PIN_RST < 0) return;
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(10);
  digitalWrite(PIN_RST, HIGH);
  delay(200);
}

}  // namespace

void imuBegin() { ready = false; }

bool imuStart(uint8_t hintAddress) {
  hardReset();
  const uint8_t first = hintAddress ? hintAddress : (uint8_t)0x4A;
  const uint8_t second = (first == 0x4A) ? (uint8_t)0x4B : (uint8_t)0x4A;
  const uint8_t candidates[2] = {first, second};

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    const uint8_t addr = candidates[attempt];
    Out.printf("BNO08x: trying 0x%02X ... ", addr);
    if (imu.begin(addr, Wire)) {
      Out.println(F("OK"));
      address = addr;
      // Rotation vector = on-chip 9-DoF fusion, the output this project needs.
      if (!imu.enableRotationVector(IMU_REPORT_INTERVAL_MS)) {
        Out.println(F("  but enableRotationVector failed."));
        ready = false;
        return false;
      }
      ready = true;
      return true;
    }
    Out.println(F("no response"));
    hardReset();
  }
  ready = false;
  return false;
}

bool imuReady() { return ready; }
uint8_t imuAddress() { return address; }
uint8_t imuAccuracy() { return ready ? accuracy : 0; }

bool imuPoll(flare::Quat &q, uint8_t &acc) {
  if (!ready) return false;

  // The IMU resets itself on power glitches, and a reset clears the report
  // subscription. Without re-arming, the program keeps running happily and never
  // receives another sample -- the "no error but no data" failure mode.
  // 中文：BNO08x 遇到电源毛刺会自己复位，复位会清掉之前订阅的报告配置。
  //       不重新 enable 的话，表面上程序还在跑，实际再也收不到任何姿态数据 ——
  //       典型的「没报错但就是没数据」。所以每轮都检查一次。
  if (imu.wasReset()) {
    Out.println(F("\n[!] IMU reported a reset -- re-enabling rotation vector"));
    imu.enableRotationVector(IMU_REPORT_INTERVAL_MS);
  }

  if (!imu.getSensorEvent()) return false;
  if (imu.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) return false;

  const flare::Quat raw = {imu.getQuatReal(), imu.getQuatI(), imu.getQuatJ(),
                           imu.getQuatK()};
  accuracy = imu.getQuatAccuracy();   // 0=unreliable .. 3=high
  acc = accuracy;

  // Everything downstream -- console, CSV, the host viewer's 3-D axes -- sees the
  // referenced pose, so "level" on screen means the pose that was calibrated, not
  // the chip's factory frame. With no reference stored this is a multiply by
  // identity and the output is unchanged.
  q = flare::applyReference(raw, settings::s.mountingRef);
  return true;
}

bool imuCaptureReference(const char *&why) {
  // Refuse while the fusion is still unreliable. A reference taken from a
  // drifting estimate bakes that drift into every later reading, permanently and
  // invisibly -- far better to make the operator rotate the unit first.
  // 中文：融合精度还不可靠时直接拒绝。用一个还在漂的估计当基准，
  //       等于把这份漂移永久烤进之后每一次读数里，而且没有任何迹象能看出来。
  //       宁可让装机的人先转两下把精度转上去。
  if (!ready) {
    why = "IMU not running -- nothing to capture";
    return false;
  }
  if (imu.getQuatAccuracy() < 2) {
    why = "fusion accuracy is low. Rotate the unit on all 3 axes until it reads"
          " medium/high, then retry.";
    return false;
  }
  settings::s.mountingRef = {imu.getQuatReal(), imu.getQuatI(), imu.getQuatJ(),
                             imu.getQuatK()};
  settings::s.refStored = true;
  settings::save();
  why = "current pose stored as level (saved to NVS)";
  return true;
}

}  // namespace hw
