#include "hw_diag.h"

#include <Arduino.h>
#include <Wire.h>

#include "hw_buzzer.h"
#include "hw_console.h"
#include "hw_lidar.h"
#include "pins.h"

namespace hw {
namespace {

// Every GPIO that can be probed without breaking the running system.
//
// Excluded on purpose, and they are the blind spot if a wire seems to vanish:
//   26-37  octal PSRAM + quad flash -- touching these crashes the chip
//   43/44  UART0 to the CH340, i.e. this very console
//   19/20  native USB D-/D+ -- see below
// Strapping pins (0/3/45/46) ARE probed: they only matter at boot, so reading
// them at runtime is harmless.
//
// GPIO19/20 are NOT harmless, which cost a long detour. They are the USB PHY
// pads, and pinMode(20, INPUT_PULLDOWN) reaches gpio_ll_pullup_dis(), which
// special-cases D+ and clears USB_SERIAL_JTAG_DP_PULLUP behind a latching
// PAD_PULL_OVERRIDE. That pull-up is the signal telling the host a device is
// attached, so probing the pin electrically unplugs the board mid-enumeration --
// Windows reports "Device Descriptor Request Failed" and the port never returns
// until a chip reset. Probing them is a one-way trip.
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
const uint8_t SAFE_GPIOS[] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                              11, 12, 13, 14, 15, 16, 17, 18, 21,
                              38, 39, 40, 41, 42, 45, 46, 47, 48};

// Pins with onboard circuitry that reads like an external connection. Naming them
// keeps a false positive from sending us down the wrong path.
const char *onboardNote(uint8_t pin) {
  switch (pin) {
    case 0:  return "  (BOOT button pull-up -- expected)";
    case 45: return "  (VDD_SPI strapping resistor -- expected)";
    case 46: return "  (boot-mode strapping resistor -- expected)";
    default: return "";
  }
}

enum LineState : uint8_t { LINE_FLOATING, LINE_PULLED_HIGH, LINE_STUCK_LOW };

// Classify a pin using both internal resistors. One reading cannot tell "nothing
// connected" from "shorted to ground" -- both read LOW under a pulldown -- so
// take two:
//   pull-up HIGH + pulldown HIGH -> an external pull-up wins   => wired
//   pull-up HIGH + pulldown LOW  -> the internals decide       => floating
//   pull-up LOW                  -> something drags it down    => shorted
//
// 中文说明：
//   为什么读两次：单次读数区分不了「什么都没接」和「短路到地」——
//   内部下拉时两种情况都读 LOW。所以用两种内部电阻做对照实验。
//   延时 500 us 是给引脚电容充放电留时间：内部上下拉在几十 kΩ 量级，
//   配上排线和面包板的寄生电容跳变没那么快，读太急会拿到上一次的残留电平。
LineState classifyLine(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(500);
  const bool highWithPullup = digitalRead(pin);
  pinMode(pin, INPUT_PULLDOWN);
  delayMicroseconds(500);
  const bool highWithPulldown = digitalRead(pin);
  pinMode(pin, INPUT);
  if (!highWithPullup) return LINE_STUCK_LOW;
  return highWithPulldown ? LINE_PULLED_HIGH : LINE_FLOATING;
}

const char *lineStateName(LineState s) {
  switch (s) {
    case LINE_PULLED_HIGH: return "pulled high";
    case LINE_STUCK_LOW:   return "stuck low  ";
    default:               return "floating   ";
  }
}

const char *imuPinMeaning(uint8_t pin, LineState s) {
  if (pin != PIN_SDA && pin != PIN_SCL && pin != PIN_RST) return "";
  switch (s) {
    case LINE_PULLED_HIGH: return " : module pull-up seen -- powered and wired";
    case LINE_STUCK_LOW:   return " : wired, but the chip has no power";
    default:               return " : wire not making contact";
  }
}

// A running TF02-Pro holds its TX at the UART idle level, which is high. So the
// green wire's DC level alone separates "sensor alive" from "sensor not powered",
// before a single byte has been parsed.
const char *lidarPinMeaning(uint8_t pin, LineState s) {
  if (pin != PIN_LIDAR_RX) return "";
  switch (s) {
    case LINE_PULLED_HIGH: return " : UART idle high -- sensor is powered";
    case LINE_STUCK_LOW:   return " : held low -- sensor has no power, or this is"
                                  " not its TX wire";
    default:               return " : wire not making contact";
  }
}

// Classic I2C unjam: a slave interrupted mid-byte keeps holding SDA low. Clocking
// SCL up to nine times lets it finish the byte and release.
//
// 中文说明：
//   I2C 卡死的典型成因：主机在从机正往外送某个字节的中途被复位了（烧录、按 RESET）。
//   从机不知道主机没了，还在等时钟把这个字节发完，于是一直把 SDA 拉低不放。
//   解法是手动敲最多 9 个时钟（8 数据位 + 1 ACK 位）让它吐完自然释放，
//   然后补一个 STOP 把总线状态复位。STOP 的定义是 SCL 高时 SDA 由低变高。
void recoverBus() {
  Out.println(F("  attempting bus recovery (9 clocks + STOP)..."));
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT);
  for (int i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; i++) {
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH);
  delayMicroseconds(5);
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  i2cReattach();
}

bool checkBusUsable() {
  LineState sda = classifyLine(PIN_SDA);
  LineState scl = classifyLine(PIN_SCL);

  if (sda == LINE_STUCK_LOW || scl == LINE_STUCK_LOW) {
    recoverBus();
    sda = classifyLine(PIN_SDA);
    scl = classifyLine(PIN_SCL);
  }
  i2cReattach();      // classifyLine() left the pins as plain inputs

  if (sda == LINE_STUCK_LOW || scl == LINE_STUCK_LOW) {
    Out.printf("  BUS JAMMED: SDA(GPIO%d)=%s, SCL(GPIO%d)=%s\n",
               PIN_SDA, sda == LINE_STUCK_LOW ? "LOW" : "ok",
               PIN_SCL, scl == LINE_STUCK_LOW ? "LOW" : "ok");
    Out.println(F("  a line is tied to GND. Skipping scan (it would hang)."));
    Out.println(F("  most likely: wire in the wrong hole, or VCC not at 3V3."));
    return false;
  }
  if (sda == LINE_FLOATING && scl == LINE_FLOATING) {
    Out.println(F("  both I2C lines floating: no pull-ups, nothing is wired here."));
    return false;
  }
  return true;
}

}  // namespace

void i2cBegin() {
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Wire.setTimeOut(500);   // the BNO08x stretches the clock; the default is tight
}

// A pin handed back as INPUT has its output driver disabled, which silently
// detaches it from the I2C peripheral: every transfer then times out at ~1 s, so
// a 112-address scan takes two minutes and looks exactly like a hang.
//
// Wire.end() first is not optional. Arduino-ESP32's Wire.begin() returns early
// with "Bus already started in Master Mode" once the driver is up and never
// re-binds the pins -- calling begin() alone does not fix it.
//
// 中文说明（这个坑今天栽了三次：I2C、UART1、LEDC）：
//   pinMode(pin, INPUT) 会关掉输出驱动。I2C 是开漏总线，主机全靠「把线拉低」发数据，
//   输出一关 Wire 外设就再也拉不动 SDA/SCL 了。症状不是报错，
//   而是每次传输都等满超时（实测约 1 秒），112 个地址扫下来近两分钟，看上去像死机。
//
//   必须先 end()：Arduino-ESP32 的 Wire.begin() 发现驱动已初始化就直接返回
//   （只打一句 "Bus already started"），根本不会重新配置引脚。
void i2cReattach() {
  Wire.end();
  i2cBegin();
}

void probeLines() {
  Out.println(F("--- line probe ---"));
  int interesting = 0;
  for (uint8_t i = 0; i < sizeof(SAFE_GPIOS); i++) {
    const uint8_t pin = SAFE_GPIOS[i];
    const LineState s = classifyLine(pin);
    const bool isImuPin = (pin == PIN_SDA || pin == PIN_SCL || pin == PIN_RST);
    const bool isLidarPin = (pin == PIN_LIDAR_RX || pin == PIN_LIDAR_TX);
    // Always report our own wires, so a missing one shows up as such rather than
    // as silence; report other pins only when something is there.
    if (s == LINE_FLOATING && !isImuPin && !isLidarPin) continue;
    const char *name = (pin == PIN_SDA) ? "  <- SDA"
                       : (pin == PIN_SCL) ? "  <- SCL"
                       : (pin == PIN_RST) ? "  <- RST"
                       : (pin == PIN_LIDAR_RX) ? "  <- LIDAR RX (green wire)"
                       : (pin == PIN_LIDAR_TX) ? "  <- LIDAR TX (white wire)"
                                               : onboardNote(pin);
    Out.printf("  GPIO%-2u %s%s%s%s\n", pin, lineStateName(s), name,
               imuPinMeaning(pin, s), lidarPinMeaning(pin, s));
    if ((!isImuPin && !isLidarPin) || s != LINE_FLOATING) interesting++;
  }
  if (interesting == 0) {
    Out.println(F("  every pin floating: nothing is connected and powered."));
    Out.println(F("  check VCC->3V3 and GND->GND first, nothing else matters yet."));
  }
  Out.printf("--- %d non-floating pin(s) ---\n", interesting);
  Out.println(F("  (GPIO26-37 = flash/PSRAM, GPIO43/44 = this console, and"
                " GPIO19/20 = USB D-/D+ cannot be probed"));
  Out.println(F("   -- a wire on any of those is invisible here)"));

  // The probe just stole every one of these from its peripheral.
  i2cReattach();
  lidarReattach();
  buzzReattach();
}

uint8_t scanI2C() {
  Out.println(F("--- I2C scan ---"));
  uint8_t found_imu = 0;
  if (!checkBusUsable()) {
    Out.println(F("--- scan skipped ---"));
    return 0;
  }

  Wire.setTimeOut(50);          // short leash while probing dead addresses
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char *tag = (addr == 0x4A || addr == 0x4B) ? "  <- BNO08x"
                        : (addr == 0x3C || addr == 0x3D) ? "  <- SSD1306 OLED"
                                                         : "";
      Out.printf("  0x%02X%s\n", addr, tag);
      if (addr == 0x4A || addr == 0x4B) found_imu = addr;
      found++;
    }
  }
  Wire.setTimeOut(500);         // the BNO08x stretches the clock; give it room
  if (found == 0) Out.println(F("  nothing responded."));
  Out.printf("--- %d device(s) ---\n", found);
  return found_imu;
}

}  // namespace hw
