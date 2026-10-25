// Use real command parsing/formatting without BLE/USB hardware.
#undef ARDUINO_ARCH_ESP32
#include "main.cpp"
#include <cassert>
#include <cstdio>
#include <driver/pcnt.h>
#include <soc/gpio_struct.h>
uint32_t fakeTime=0;
void (*fakeInterrupts[64])(){};
FakeGpio GPIO;
pcnt_config_t fakeChannels[2]{};
int16_t fakePcnt=0;
bool fakePcntFailure=false;
FakeSerial Serial;
int main() {
    execute("INFO");
    assert(Serial.output.find("TRACE_CRC,TRACE_REQUEST_ID,PCNT")!=std::string::npos);
    const char* invalid[]={"TRACE -1","TRACE 1","TRACE 0 -1","TRACE 0 4294967296", "TRACE 0 1 extra"};
    for (const char* command:invalid) {
        Serial.output.clear(); execute(command);
        assert(Serial.output.find("ERR TRACE ")==0);
    }
    Serial.output.clear(); execute("TRACE");
    assert(Serial.output.find("request=0")!=std::string::npos);
    Serial.output.clear(); execute("TRACE 0 4294967295");
    assert(Serial.output.find("request=4294967295")!=std::string::npos);
    std::fputs(Serial.output.c_str(),stdout);
}
