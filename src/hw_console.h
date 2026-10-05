// Console. One object, `Out`, whatever the board's route to the PC is.
//
// Two layouts exist, decided by ARDUINO_USB_CDC_ON_BOOT, which the board profile
// sets:
//
//   =0  DevKitC clone. Two independent routes, and either may be the only cable
//       plugged in: UART0 through the CH340 bridge (`Serial`) and the S3's own
//       USB peripheral (`USBSerial`). The console writes to both so it follows
//       whichever cable is in use with no rebuild -- worth the indirection after
//       losing time to a silent port once already. The idle link costs nothing:
//       HWCDC::write() checks for a host and discards without blocking.
//
//   =1  XIAO ESP32S3. Only the native USB exists, and the core binds `Serial`
//       to it; there is no `USBSerial` object at all, so the tee collapses to a
//       single stream.
//
// 中文说明：
//   旧板子有两条互相独立、都能连到电脑的串口通道，而且你随时可能只插其中一根：
//     Serial    = UART0（GPIO43/44）-> 板载 CH340K -> 电脑上的 COM4
//     USBSerial = ESP32-S3 自带的 USB Serial/JTAG 外设，直接出到 USB 口
//   两条都写，于是「插哪根线就从哪根线看输出」，换线不用重新编译。
//   往没插的那条线写不会卡住：HWCDC::write() 先判断主机在不在，不在就丢掉。
//
//   XIAO 只有原生 USB 一条路。板卡配置把 ARDUINO_USB_CDC_ON_BOOT 设成 1，
//   这时 Arduino 核心把 Serial 直接绑到 USB 上，**USBSerial 这个对象根本不存在**，
//   引用它会编译失败 —— 所以这里按这个宏分成两种实现，对外都叫 Out。
#pragma once

#include <Arduino.h>

class TeeStream : public Print {
 public:
#if ARDUINO_USB_CDC_ON_BOOT
  size_t write(uint8_t c) override { return Serial.write(c); }
  size_t write(const uint8_t *buf, size_t n) override { return Serial.write(buf, n); }

  // USB CDC's baud rate is fiction -- whatever the host asks for -- but the HWCDC
  // begin() accepts and ignores one, so the call shape stays the same.
  void begin(unsigned long baud) { Serial.begin(baud); }

  int available() { return Serial.available(); }
  int read() { return Serial.read(); }
#else
  size_t write(uint8_t c) override {
    USBSerial.write(c);
    return Serial.write(c);
  }

  // printf/println all funnel through here, which is far cheaper than per-byte.
  size_t write(const uint8_t *buf, size_t n) override {
    USBSerial.write(buf, n);
    return Serial.write(buf, n);
  }

  // USBSerial.begin() takes no baud: only UART0 genuinely needs one.
  void begin(unsigned long baud) {
    Serial.begin(baud);
    USBSerial.begin();
  }

  // Commands are accepted from whichever link the user happens to type into.
  int available() { return Serial.available() + USBSerial.available(); }
  int read() { return Serial.available() ? Serial.read() : USBSerial.read(); }
#endif
};

extern TeeStream Out;
