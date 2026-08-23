// Pin assignments and hardware timing constants, in one place.
//
// AS BUILT 2026-08-22. Anything not listed here is free; the ranges that are NOT
// free and NOT listed are flash/PSRAM (26-37) and native USB D-/D+ (19/20).
#pragma once

#include <stdint.h>

// --- I2C: BNO085 IMU (0x4A), OLED (0x3C) will share the bus ---
static constexpr int PIN_SDA = 8;
static constexpr int PIN_SCL = 9;
static constexpr int PIN_RST = 5;   // set to -1 if RST is not wired
static constexpr uint32_t I2C_HZ = 400000;

// --- TF02-Pro laser rangefinder on UART1 ---
//   Red -> 5V     Black -> GND (common ground, or the UART has no reference)
//   Green = sensor TXD -> our RX     White = sensor RXD -> our TX
// UART crossover: their TX goes to our RX. Swapping the pair damages nothing and
// is always the safe first thing to try, but it is not the usual fault -- a
// silent sensor is far more often an unconnected 5V wire.
// 中文：绿 = 传感器的 TX 接我们的 RX，白 = 传感器的 RX 接我们的 TX。
//       接反不会烧任何东西，永远可以放心对调试一次；但它不是最常见的故障 ——
//       「传感器不出声」绝大多数时候是 5V 那根线根本没接上。
static constexpr int PIN_LIDAR_RX = 18;    // <- green
static constexpr int PIN_LIDAR_TX = 17;    // -> white (optional: it free-runs)
static constexpr uint32_t LIDAR_BAUD = 115200;
static constexpr uint32_t LIDAR_STALE_MS = 500;   // no frame this long => offline

// --- Buzzer: 3-pin passive module on A, or a bare piezo disc across A and B ---
// With the 3-pin module, wire I/O to A and leave B unconnected. With a bare disc,
// wire both leads through 100R and B is driven inverted, which doubles the swing
// across the element to 6.6 Vpp -- about +6 dB for no extra parts.
// 中文：三脚无源模块只用 A 脚（I/O），B 脚空着。裸压电片则两脚都接（各串 100Ω），
//       B 脚反相输出，压电片两端摆幅翻倍到 6.6 Vpp，约 +6 dB，不花一分钱。
static constexpr int PIN_BUZZ_A = 15;
static constexpr int PIN_BUZZ_B = 16;      // -1 => single-ended
static constexpr uint8_t BUZZ_CH = 4;      // LEDC channel, low-speed group
static constexpr uint8_t BUZZ_RES_BITS = 10;

// --- Reporting cadence ---
static constexpr uint16_t IMU_REPORT_INTERVAL_MS = 20;   // 50 Hz from the chip
static constexpr uint32_t PRINT_HUMAN_MS = 100;          // 10 Hz to the terminal
static constexpr uint32_t PRINT_CSV_MS = 20;             // 50 Hz to the plotter

// Beyond this the beam points sideways: it is hitting ground metres away rather
// than below the foot, and no cosine can rescue that.
static constexpr float TILT_GATE_DEG = 40.0f;
