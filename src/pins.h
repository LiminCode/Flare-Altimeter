// Pin assignments and hardware timing constants, in one place.
//
// Two boards are supported, selected by a build flag in platformio.ini:
//   -DFLARE_BOARD_XIAO   Seeed XIAO ESP32S3   (the flight board, from 2026-10-01)
//   (no flag)            DevKitC-form ESP32-S3 N16R8 clone (the bench board)
//
// The pin NUMBERS differ between the boards, the SIGNAL LIST does not: every
// PIN_* name below exists for both, so src/hw_* compiles unchanged. A signal a
// board does not wire is -1, which the hardware layer already honours.
//
// 中文说明（为什么用一个宏切换两套引脚，而不是直接改数字）：
//   旧板子接着的外设和线都还在，调试时随时可能要回去对比。两套引脚并存，
//   切换只需换 platformio.ini 里的 env，不用改一行代码，也不会出现
//   「改了数字忘了改回来」这种事。
//   所有 PIN_* 名字两块板都有，没接的信号写 -1 —— hw_* 层本来就按 -1 处理。
#pragma once

#include <stdint.h>

#if defined(FLARE_BOARD_XIAO)
// =============================================================================
// Seeed XIAO ESP32S3 -- AS WIRED 2026-10-03
//
// Only 11 GPIOs reach the header; everything else is on the underside B2B
// connector and is not hand-solderable, so for this board it does not exist.
// All 11 are spoken for, which is why the piezo is single-ended here.
//
//   header   GPIO   use
//   D0        1     piezo A (single-ended)
//   D1        2     siren NMOS gate              (reserved, not in code yet)
//   D2        3     SD CS                        (reserved, not in code yet)
//   D3        4     BNO085 RST
//   D4        5     I2C SDA   (board default)
//   D5        6     I2C SCL   (board default)
//   D6       43     TF02-Pro TX  -> white (optional, see below)
//   D7       44     TF02-Pro RX  <- green
//   D8        7     SD SCK                       (reserved, not in code yet)
//   D9        8     SD MISO                      (reserved, not in code yet)
//   D10       9     SD MOSI                      (reserved, not in code yet)
//
// GPIO43/44 are the chip's UART0 pins. On this board the console is the native
// USB CDC, so UART0 is unused at runtime and UART1 can borrow its pins through
// the GPIO matrix. One boot-time caveat decides which way round they go: the ROM
// bootloader prints its log on GPIO43 for the first ~100 ms, so 43 must be the
// line we DRIVE (sensor's RXD, white) and never the one the sensor drives.
//
// 中文说明：
//   43/44 是芯片的 UART0 脚。这块板的控制台走原生 USB，UART0 运行时空着，
//   UART1 可以通过 GPIO 矩阵借用这两个脚。但 ROM 启动程序开机前约 100 ms
//   会在 GPIO43 上打印启动日志 —— 所以 43 只能做我们「往外发」的那根
//   （接传感器 RXD，白线），绝不能接传感器往外发的绿线，否则两边同时驱动一根线。
//   白线本来就是可选的（传感器自己不停发），bring-up 阶段可以干脆不接。
// =============================================================================
#define FLARE_BOARD_NAME "XIAO ESP32S3"

static constexpr int PIN_SDA = 5;
static constexpr int PIN_SCL = 6;
static constexpr int PIN_RST = 4;    // set to -1 if RST is not wired

static constexpr int PIN_LIDAR_RX = 44;    // <- green   (D7)
static constexpr int PIN_LIDAR_TX = 43;    // -> white   (D6, optional)

static constexpr int PIN_BUZZ_A = 1;       // D0
static constexpr int PIN_BUZZ_B = -1;      // single-ended: no free pin for B

// The buzzer on this board is a 3-pin "low-level trigger" module (GND / I/O /
// VCC): a PNP transistor switches the element on when I/O is LOW. So idle must be
// HIGH, or the thing sounds continuously whenever the firmware means "silent".
// A 50% square wave is the same tone either way up, so only the idle level cares.
// Power the module from 3V3, not 5V: with a 5 V VCC and a 3.3 V HIGH the PNP
// base still sits 1.7 V below the emitter and never turns off.
// 中文：这块板上接的是三脚「低电平触发」模块，PNP 管在 I/O 为低时导通发声。
//       所以空闲电平必须是高，否则固件想「静音」的时候它反而一直响。
//       50% 方波正反都一样，只有空闲电平需要在意。
//       模块 VCC 必须接 3V3 而不是 5V：VCC 5 V 时我们的高电平只有 3.3 V，
//       PNP 的基极仍比发射极低 1.7 V，管子永远关不掉。
static constexpr bool BUZZ_IDLE_HIGH = true;

// Every header GPIO. Nothing here is flash, PSRAM, USB or the console, so the
// whole header can be probed.
static constexpr uint8_t PROBE_GPIOS[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 43, 44};
#define PROBE_EXCLUDED_NOTE \
  "  (only the 11 header pins exist on this board; all of them are probed)"

#else
// =============================================================================
// DevKitC-form ESP32-S3 N16R8 clone + PinPulse shield -- AS BUILT 2026-08-22
//
// Anything not listed here is free; the ranges that are NOT free and NOT listed
// are flash/PSRAM (26-37) and native USB D-/D+ (19/20).
// =============================================================================
#define FLARE_BOARD_NAME "ESP32-S3 DevKitC (N16R8 clone)"

// --- I2C: BNO085 IMU (0x4A), OLED (0x3C) will share the bus ---
static constexpr int PIN_SDA = 8;
static constexpr int PIN_SCL = 9;
static constexpr int PIN_RST = 5;   // set to -1 if RST is not wired

// --- TF02-Pro laser rangefinder on UART1 ---
static constexpr int PIN_LIDAR_RX = 18;    // <- green
static constexpr int PIN_LIDAR_TX = 17;    // -> white (optional: it free-runs)

// --- Buzzer: 3-pin passive module on A, or a bare piezo disc across A and B ---
// With the 3-pin module, wire I/O to A and leave B unconnected. With a bare disc,
// wire both leads through 100R and B is driven inverted, which doubles the swing
// across the element to 6.6 Vpp -- about +6 dB for no extra parts.
// 中文：三脚无源模块只用 A 脚（I/O），B 脚空着。裸压电片则两脚都接（各串 100Ω），
//       B 脚反相输出，压电片两端摆幅翻倍到 6.6 Vpp，约 +6 dB，不花一分钱。
static constexpr int PIN_BUZZ_A = 15;
static constexpr int PIN_BUZZ_B = 16;      // -1 => single-ended
static constexpr bool BUZZ_IDLE_HIGH = false;   // bare disc: both leads low when silent

// Every GPIO that can be probed without breaking the running system.
//
// Excluded on purpose, and they are the blind spot if a wire seems to vanish:
//   26-37  octal PSRAM + quad flash -- touching these crashes the chip
//   43/44  UART0 to the CH340, i.e. this very console
//   19/20  native USB D-/D+ -- probing them electrically unplugs the board.
//          pinMode(20, INPUT_PULLDOWN) reaches gpio_ll_pullup_dis(), which
//          special-cases D+ and clears USB_SERIAL_JTAG_DP_PULLUP behind a
//          latching PAD_PULL_OVERRIDE; the port is gone until a chip reset.
// Strapping pins (0/3/45/46) ARE probed: they only matter at boot.
//
// 中文说明（这段是踩过坑换来的，改之前务必读完）：
//   GPIO19/20 曾经被放进这个「安全探测」列表，理由是「原生 USB 没用到，
//   运行时读一下总没事吧」。这个判断错了，而且错得很隐蔽：
//     1. 它们不是普通 IO，是 USB PHY 的引脚，GPIO20 = USB D+。
//     2. pinMode(pin, INPUT_PULLDOWN) 设下拉必先关上拉，进到 IDF 的
//        gpio_ll_pullup_dis()，那个函数对 D+ 有特判：置位 PAD_PULL_OVERRIDE
//        并清掉 DP_PULLUP。
//     3. D+ 上这个上拉电阻正是 USB 协议里「有设备接上来了」的电信号。
//        清掉它 = 在电气层面把板子从 USB 总线上拔下来。
//     4. PAD_PULL_OVERRIDE 是锁存的，一旦置位不会自己恢复，必须复位芯片。
//   表现出来是：插线 -> 正常枚举 -> 约 300 ms 后固件启动跑探测 -> 端口消失。
//   这曾被误判成「这块板的原生 USB 口是坏的」，其实是固件自己打死的。
static constexpr uint8_t PROBE_GPIOS[] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                                          11, 12, 13, 14, 15, 16, 17, 18, 21,
                                          38, 39, 40, 41, 42, 45, 46, 47, 48};
#define PROBE_EXCLUDED_NOTE                                                     \
  "  (GPIO26-37 = flash/PSRAM, GPIO43/44 = this console, and GPIO19/20 = USB " \
  "D-/D+ cannot be probed\n   -- a wire on any of those is invisible here)"

#endif

// =============================================================================
// Board-independent
// =============================================================================

static constexpr uint32_t I2C_HZ = 400000;

// TF02-Pro wiring, same on every board:
//   Red -> 5V     Black -> GND (common ground, or the UART has no reference)
//   Green = sensor TXD -> our RX     White = sensor RXD -> our TX
// UART crossover: their TX goes to our RX. Swapping the pair damages nothing and
// is always the safe first thing to try, but it is not the usual fault -- a
// silent sensor is far more often an unconnected 5V wire.
// 中文：绿 = 传感器的 TX 接我们的 RX，白 = 传感器的 RX 接我们的 TX。
//       接反不会烧任何东西，永远可以放心对调试一次；但它不是最常见的故障 ——
//       「传感器不出声」绝大多数时候是 5V 那根线根本没接上。
static constexpr uint32_t LIDAR_BAUD = 115200;
static constexpr uint32_t LIDAR_STALE_MS = 500;   // no frame this long => offline

static constexpr uint8_t BUZZ_CH = 4;      // LEDC channel, low-speed group
static constexpr uint8_t BUZZ_RES_BITS = 10;

// --- Reporting cadence ---
static constexpr uint16_t IMU_REPORT_INTERVAL_MS = 10;   // 100 Hz from the chip, same as the laser (was 20; raised 2026-10-03, measured rate shown in status)
static constexpr uint32_t PRINT_HUMAN_MS = 100;          // 10 Hz to the terminal
static constexpr uint32_t PRINT_CSV_MS = 20;             // 50 Hz to the plotter

// Beyond this the beam points sideways: it is hitting ground metres away rather
// than below the foot, and no cosine can rescue that.
static constexpr float TILT_GATE_DEG = 40.0f;
