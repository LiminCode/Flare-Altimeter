// Console that mirrors to both links to the PC.
//
// The board has two independent routes and either one may be the only cable
// plugged in: UART0 through the CH340 bridge (`Serial`), and the S3's own USB
// peripheral (`USBSerial`). Writing to both means the console follows whichever
// cable is in use with no rebuild -- worth the indirection after losing time to a
// silent port once already.
//
// The idle link costs nothing: HWCDC::write() checks for a host, and with none
// attached it discards the data and returns immediately rather than blocking.
//
// 中文说明：
//   板子上有两条互相独立、都能连到电脑的串口通道，而且你随时可能只插其中一根：
//     Serial    = UART0（GPIO43/44）-> 板载 CH340K -> 电脑上的 COM4
//     USBSerial = ESP32-S3 自带的 USB Serial/JTAG 外设，直接出到 USB 口
//   两条都写，于是「插哪根线就从哪根线看输出」，换线不用重新编译。
//   之所以值得多这一层间接，是因为之前已经为了一个不出声的串口排查了很久。
//
//   往没插的那条线写不会卡住：HWCDC::write() 先判断主机在不在，不在就把数据丢掉
//   直接返回，不阻塞也不等超时。
//
//   注意 platformio.ini 里必须保持 ARDUINO_USB_CDC_ON_BOOT=0 —— 这样 Serial 仍然
//   绑在 UART0 上，原生 USB 通过 USBSerial 这个独立对象额外挂上来。改成 =1 的话
//   Serial 会被重定向到 USB，CH340 那条路就彻底没输出了。
#pragma once

#include <Arduino.h>

class TeeStream : public Print {
 public:
  size_t write(uint8_t c) override {
    USBSerial.write(c);
    return Serial.write(c);
  }

  // printf/println all funnel through here, which is far cheaper than per-byte.
  size_t write(const uint8_t *buf, size_t n) override {
    USBSerial.write(buf, n);
    return Serial.write(buf, n);
  }

  // USBSerial.begin() takes no baud: USB CDC's baud rate is fiction, whatever the
  // host asks for. Only UART0 genuinely needs one.
  void begin(unsigned long baud) {
    Serial.begin(baud);
    USBSerial.begin();
  }

  // Commands are accepted from whichever link the user happens to type into.
  int available() { return Serial.available() + USBSerial.available(); }
  int read() { return Serial.available() ? Serial.read() : USBSerial.read(); }
};

extern TeeStream Out;
