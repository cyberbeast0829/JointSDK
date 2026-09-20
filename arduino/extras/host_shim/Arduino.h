/*
 * 极简 Arduino.h 替身 —— **只用于在 PC 上做语法/API 检查**，不是给板子用的。
 *
 * 为什么需要它：本仓库的 CI 没有 arduino-cli，而"写完没编过"的示例比没有示例更糟
 * （客户照抄会一头撞进拼写错误）。`tools/arduino_smoke.sh` 用 g++ 配这个替身把
 * sketch 的**逻辑**编一遍，能挡住大部分误用。
 *
 * ⚠ 它**挡不住** AVR 特有问题：int 是 16 bit、`snprintf` 体积、PROGMEM 语义、
 *   中断上下文里的调用约定…… 第一次上板仍要预留调试时间。
 */

#ifndef ARDUINO_HOST_SHIM_H
#define ARDUINO_HOST_SHIM_H

#include <cstdint>
#include <cstdio>
#include <cstring>

#define ARDUINO 10800
#define PROGMEM

/* 真实 Arduino.h 里 `F("x")` 的返回类型是 `const __FlashStringHelper*`（字面量放 Flash）。
   ⚠ 替身必须**一样地**这么定义：早期版本让 F() 返回 `const char*`，
     于是 `fatal(F("..."))` 这类调用在替身下报“cannot convert const char* to
     const __FlashStringHelper*”，而在真板上是合法的 —— 替身与真实 API 不一致时，
     它报的错比不报更浪费时间。 */
class __FlashStringHelper {};
#define F(x) (reinterpret_cast<const __FlashStringHelper *>(x))

/* print 的进制常量（
   Serial.print(v, HEX) 这类调用在真实库里就是这几个宏）。 */
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

/* AVR 的 int 是 16 bit；PC 上是 32 bit —— 故意不模拟这种差异，
   它能编过就说明没用到 16-bit 假设以外的怪东西。 */
typedef uint8_t byte;

/* --- millis()：宿主上每调用一次前进 1 ms（示例里够用） --- */
static inline uint32_t millis()
{
    static uint32_t t = 0u;
    return t++;
}
static inline void delay(uint32_t ms) { (void)ms; }

/* --- Serial：打到 stdout --- */
struct ArduinoHostSerial {
    void begin(unsigned long baud) { (void)baud; }

    void print(const char *s) { std::fputs(s, stdout); }
    void print(char c) { std::fputc(c, stdout); }
    void print(int v) { std::printf("%d", v); }
    void print(unsigned v) { std::printf("%u", v); }
    void print(long v) { std::printf("%ld", v); }
    void print(unsigned long v) { std::printf("%lu", v); }
    void print(double v, int digits = 2) { std::printf("%.*f", digits, v); }
    void print(const __FlashStringHelper *s) { std::fputs((const char *)s, stdout); }

    /* 带进制的重载：Serial.print(v, HEX) */
    void print(unsigned v, int base) {
        if (base == HEX) std::printf("%X", v); else std::printf("%u", v);
    }
    void print(int v, int base) { print((unsigned)v, base); }

    void println(const char *s = "") { std::printf("%s\n", s); }
    void println(int v) { std::printf("%d\n", v); }
    void println(unsigned v) { std::printf("%u\n", v); }
    void println(long v) { std::printf("%ld\n", v); }
    void println(unsigned long v) { std::printf("%lu\n", v); }
    void println(double v, int digits = 2) { std::printf("%.*f\n", digits, v); }
    void println(const __FlashStringHelper *s) { std::printf("%s\n", (const char *)s); }

    void println(unsigned v, int base) { print(v, base); std::fputc('\n', stdout); }
    void println(int v, int base) { print((unsigned)v, base); std::fputc('\n', stdout); }

    int  available() { return 0; }     /* 宿主上没有串口输入 */
    int  read() { return -1; }
};

static ArduinoHostSerial Serial;

/* 示例里用到 sin() —— 真实 Arduino 由 <math.h> 提供 */
#include <cmath>

#endif /* ARDUINO_HOST_SHIM_H */
