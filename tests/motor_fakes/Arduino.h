#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#define IRAM_ATTR
#define DRAM_ATTR
#define INPUT 0
#define OUTPUT 1
#define CHANGE 2
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
extern uint32_t fakeTime;
extern void (*fakeInterrupts[64])();
inline uint32_t micros() { return fakeTime; }
inline uint32_t millis() { return fakeTime/1000; }
inline void pinMode(int,int) {}
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int pin,void (*fn)(),int) { fakeInterrupts[pin]=fn; }
inline void ledcSetup(int,int,int) {}
inline void ledcAttachPin(int,int) {}
inline void ledcWrite(int,int) {}
template<class T,class U,class V> inline T constrain(T x,U lo,V hi) {
    return std::max(T(lo),std::min(x,T(hi)));
}

#ifndef ARDUINO_ARCH_ESP32
#include <cstdarg>
#include <cstdio>
#include <string>
using std::min;
struct FakeSerial {
    std::string output;
    void begin(int) {}
    int available() { return 0; }
    int read() { return -1; }
    void printf(const char* format,...) {
        char data[1024]; va_list args; va_start(args,format);
        vsnprintf(data,sizeof(data),format,args); va_end(args); output+=data;
    }
};
extern FakeSerial Serial;
#endif

#ifndef INPUT_PULLUP
#define INPUT_PULLUP 2
#endif
