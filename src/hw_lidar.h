// TF02-Pro on UART1: byte pump, staleness, and the wiring self-diagnosis.
//
// Frame decoding itself lives in lib/tf02 so it can be tested off-target; this
// file is only the parts that need real hardware.
#pragma once

#include <stdint.h>

namespace hw {

void lidarBegin();

// The wiring probe leaves GPIO17/18 as plain inputs, which detaches them from
// UART1. Same trap as I2C and LEDC. end() then begin() puts them back.
void lidarReattach();

// Non-blocking: drains whatever bytes have arrived and returns. Must be called
// from every path through loop(), including the ones that bail out early --
// at 100 Hz the 256-byte RX FIFO fills in under three seconds of not looking.
// 中文：非阻塞。必须在 loop 里所有提前 return 之前调用 ——
//       100 Hz 的帧率下不到三秒就能把 256 字节的接收 FIFO 灌满，溢出就是整片丢帧。
void lidarPoll();

// Busy-wait on the stream for the wiring diagnostics only, never on the flight
// path. Returns the number of frames that arrived.
uint32_t lidarListen(uint32_t ms);

bool lidarOnline();
int32_t lidarDistCm();       // -1 when there is no trustworthy range
uint16_t lidarStrength();
float lidarTempC();
uint32_t lidarFrames();
uint32_t lidarBadChecksums();

// Report whether the sensor is talking, and if not, distinguish the two very
// different faults behind that silence.
void lidarDiagnose();

// --- replay injection ------------------------------------------------------
// Push a recorded or synthetic frame in as though the sensor had produced it.
// Everything downstream -- staleness, the validity gate, tilt correction, the
// alert engine, the buzzer, the CSV stream -- runs exactly as it does in flight,
// because none of it can tell the difference. That is the point: unit tests prove
// the logic, this proves the wiring between the logic and the hardware.
//
// Real frames are drained but discarded while replaying, so a laser sitting on
// the desk cannot fight the recording for control of the reading.
//
// 中文说明（为什么注入点选在这里）：
//   把一帧数据当成传感器自己产出的塞进来。下游的一切 —— 过期判断、有效性判据、
//   倾角修正、警报引擎、蜂鸣器、CSV 输出 —— 全都照常运行，因为它们分辨不出真假。
//   这正是目的：单元测试证明逻辑本身是对的，回放测试证明逻辑和硬件之间接对了。
//
//   回放期间真实帧照常从 UART 读出来（否则 FIFO 会溢出）但直接丢弃，
//   这样桌上那台还在工作的激光不会和录音抢控制权。
void lidarInject(int32_t distCm, uint16_t strength);
bool lidarReplayActive();

// A feeder that dies mid-run must not leave the device frozen on the last
// injected frame forever; after this long with no injection, real data resumes.
// 中文：喂数据的进程中途挂掉时，设备不能永远停在最后一帧注入值上 ——
//       超过这个时间没有新注入就自动切回真实数据。
constexpr uint32_t LIDAR_REPLAY_TIMEOUT_MS = 1000;

}  // namespace hw
