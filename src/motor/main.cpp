#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "motion_controller.h"
#include "host_output.h"

#ifdef ARDUINO_ARCH_ESP32
#include <atomic>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#endif
using namespace HammerConfig;
Drv8871Motor motor({MotorIn1, MotorIn2, 0, 1, 20000, 10});
HallEncoder encoder({EncoderA, EncoderB, 16, 27});
MotionController motion(motor, encoder);

constexpr unsigned LineCapacity = 160, OutputCapacity = 320;
enum class Link : uint8_t { Usb=1, Ble=2, Both=3 };
struct Command { char line[LineCapacity]; Link link=Link::Usb; };
struct Output { char line[OutputCapacity]; };
#ifdef ARDUINO_ARCH_ESP32
static QueueHandle_t commands = nullptr, usbOutputs = nullptr, bleOutputs = nullptr;
static Link replyLink = Link::Both;
static std::atomic<uint32_t> usbDropped{0}, bleDropped{0};
static TaskHandle_t controllerTask = nullptr;
static std::atomic<bool> stopRequested{false}, bleConnected{false}, restartAdvertising{false};
static BLECharacteristic* bleTx = nullptr;
static esp_timer_handle_t controlTimer;
#else
static bool stopRequested = false;
#endif

void hostPrintf(const char* format, ...) {
    Output out{};
    va_list args; va_start(args, format);
    vsnprintf(out.line, sizeof(out.line), format, args); va_end(args);
#ifdef ARDUINO_ARCH_ESP32
    // Replies go to the requesting link; asynchronous motion events reach both.
    const bool event = !strncmp(out.line,"EVT ",4) || !strncmp(out.line,"HELLO ",6);
    const unsigned mask = event ? 3 : unsigned(replyLink);
    if ((mask & 1) && Serial && usbOutputs && xQueueSend(usbOutputs,&out,0)!=pdTRUE) ++usbDropped;
    if ((mask & 2) && bleConnected && bleOutputs && xQueueSend(bleOutputs,&out,0)!=pdTRUE) ++bleDropped;
#else
    Serial.printf("%s", out.line);
#endif
}
static bool nextFloat(const char*& p, float& x) {
    while (*p == ' ') ++p;
    if (!*p) return false;
    char* end = nullptr; x = strtof(p, &end);
    if (end == p || !isfinite(x) || (*end && *end != ' ')) return false;
    p = end; return true;
}
static bool atEnd(const char* p) { while (*p == ' ') ++p; return !*p; }
static void sendTuning(const char* prefix) {
    const auto& p = motion.tuning();
    hostPrintf("%s kp=%.4f ki=%.4f ks=%.4f kv=%.4f kg=%.4f\n", prefix, p.kp,p.ki,p.ks,p.kv,p.kg);
}
static void execute(const char* c, Link link=Link::Usb) {
#ifdef ARDUINO_ARCH_ESP32
    replyLink=link;
#else
    (void)link;
#endif
    if (!*c) return;
    if (!strcmp(c,"STOP")) { motion.stop(); hostPrintf("OK STOP\n"); }
    else if (!strcmp(c,"PING")) hostPrintf("OK PING\n");
    else if (!strcmp(c,"INFO")) {
        hostPrintf("INFO firmware=HAMMER_COAST_V7 protocol=2 period_us=1000 features=USB,BLE,HOME,JOG,TRACE,TUNE\n");
    }
    else if (!strncmp(c,"JOG ",4)) {
        const char* p=c+4; float speed;
        if (!nextFloat(p,speed) || !atEnd(p)) hostPrintf("ERR JOG syntax_JOG_signed_speed\n");
        else if (!motion.jog(speed)) hostPrintf("ERR JOG busy_or_invalid\n");
    }
    else if (!strcmp(c,"HOME")) {
        if (!motion.home()) hostPrintf("ERR HOME busy_or_moving\n");
    }
    else if (!strcmp(c,"ZERO")) hostPrintf(motion.zeroHere() ? "OK ZERO angle=0\n" : "ERR ZERO busy_or_moving\n");
    else if (!strcmp(c,"RETRACT")) {
        if (!motion.retract()) hostPrintf("ERR RETRACT not_idle_or_not_zeroed_or_moving\n");
    } else if (!strcmp(c,"START")) {
        if (!motion.start()) hostPrintf("ERR START not_idle_or_not_zeroed_or_moving\n");
    } else if (!strcmp(c,"FIRE")) {
        if (!motion.fire()) hostPrintf("ERR FIRE not_ready_or_not_still\n");
    } else if (!strcmp(c,"STATUS")) {
        hostPrintf("STATUS state=%s zeroed=%u angle=%.2f speed=%.2f raw=%.2f pwm=%.0f shot=%u samples=%u homing=%u output=%s in1=%d in2=%d reason=%s pos_error=%.2f settled_ms=%lu phase_ms=%lu auto_fire=%u\n",
            MotionController::stateName(motion.state()), unsigned(motion.zeroed()), motion.angle(),
            motion.speed(), motion.rawSpeed(), motion.pwm(), motion.shotId(), motion.traceCount(),unsigned(motion.homing()),motor.outputMode(),motor.in1Duty(),motor.in2Duty(),motion.lastReason(),motion.positionError(),
            static_cast<unsigned long>(motion.settledMs()),static_cast<unsigned long>(motion.phaseMs()),unsigned(motion.autoFire()));
    } else if (!strcmp(c,"PARAMS")) {
        hostPrintf("PARAMS retract_angle=%.2f strike_speed=%.2f\n", motion.settings().retractAngleDeg, motion.settings().strikeSpeedDps);
    } else if (!strcmp(c,"TUNING")) sendTuning("TUNING");
    else if (!strncmp(c,"TUNE ",5)) {
        const char* p = c+5; SpeedTuning t;
        if (!nextFloat(p,t.kp) || !nextFloat(p,t.ki) || !nextFloat(p,t.ks)
            || !nextFloat(p,t.kv) || !nextFloat(p,t.kg) || !atEnd(p)) {
            hostPrintf("ERR TUNE syntax_TUNE_kp_ki_ks_kv_kg\n"); return;
        }
        if (motion.setTuning(t)) sendTuning("OK TUNE");
        else hostPrintf("ERR TUNE busy_or_invalid\n");
    } else if (!strncmp(c,"SET ",4)) {
        const char* p=c+4; MotionController::Settings s;
        if (!nextFloat(p,s.retractAngleDeg) || !nextFloat(p,s.strikeSpeedDps) || !atEnd(p)) {
            hostPrintf("ERR SET syntax_SET_angle_speed\n"); return;
        }
        if (motion.setSettings(s)) hostPrintf("OK SET retract_angle=%.2f strike_speed=%.2f\n",s.retractAngleDeg,s.strikeSpeedDps);
        else hostPrintf("ERR SET busy_or_non_positive\n");
    } else if (!strcmp(c,"TRACE") || !strncmp(c,"TRACE ",6)) {
        // Each request returns at most eight rows. The host retries missing chunks.
        if (motion.state() != MotionController::State::Idle) { hostPrintf("ERR TRACE busy\n"); return; }
        const char* p=c+5;
        while (*p==' ') ++p;
        char* end=nullptr;
        const unsigned long offset = *p ? strtoul(p,&end,10) : 0;
        if ((*p && (*p=='-' || end==p || !atEnd(end))) || offset > motion.traceCount()) {
            hostPrintf("ERR TRACE invalid_offset\n"); return;
        }
        const unsigned total=motion.traceCount();
        const auto& ts=motion.traceSettings();
        const auto& tp=motion.traceTuning();
        hostPrintf("TRACE_BEGIN shot=%u total=%u offset=%lu target=%.2f window_us=%lu "
            "reason=%s contact_candidate=%u retract_angle=%.2f kp=%.4f ki=%.4f ks=%.4f kv=%.4f kg=%.4f\n",
            motion.shotId(),total,offset,motion.traceTarget(),static_cast<unsigned long>(DropWindowUs),
            motion.traceReason(),unsigned(motion.traceContact()),ts.retractAngleDeg,tp.kp,tp.ki,tp.ks,tp.kv,tp.kg);
        const unsigned limit = min(total, unsigned(offset)+TraceChunk);
        for (unsigned i=unsigned(offset);i<limit;++i) {
            const auto& s=motion.trace(i);
            hostPrintf("TRACE shot=%u i=%u us=%lu angle=%.3f speed=%.3f raw=%.3f pwm=%.0f drop=%.3f baseline=%.3f\n",
                motion.shotId(),i,static_cast<unsigned long>(s.timeUs),s.angle,s.speed,s.rawSpeed,s.pwm,s.dropSpeed,s.baseline);
        }
        hostPrintf("TRACE_END shot=%u next=%u total=%u\n",motion.shotId(),limit,total);
    } else hostPrintf("ERR UNKNOWN_COMMAND\n");
}

static void submit(const char* line, Link link) {
    if (!strcmp(line,"STOP")) { stopRequested = true; return; }
#ifdef ARDUINO_ARCH_ESP32
    Command cmd{}; cmd.link=link; strncpy(cmd.line,line,sizeof(cmd.line)-1);
    if (xQueueSend(commands,&cmd,0)!=pdTRUE) hostPrintf("ERR COMMAND_QUEUE_FULL\n");
#else
    if (!stopRequested) execute(line,link);
#endif
}
struct LineParser {
    Link link;
    explicit LineParser(Link source=Link::Usb) : link(source) {}
    char line[LineCapacity]{};
    unsigned length=0; bool discard=false;
    void feed(char ch) {
        if (ch=='!') { stopRequested=true; length=0; discard=true; }
        else if (ch=='\n' || ch=='\r') {
            if (!discard && length) { line[length]=0; submit(line,link); }
            discard=false; length=0;
        } else if (!discard) {
            if (length+1<sizeof(line)) line[length++]=ch;
            else { length=0; discard=true; hostPrintf("ERR LINE_TOO_LONG\n"); }
        }
    }
};
static LineParser usbParser;
#ifdef ARDUINO_ARCH_ESP32
static LineParser bleParser(Link::Ble);
#endif
#ifdef ARDUINO_ARCH_ESP32
class BleServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override { bleParser=LineParser{Link::Ble}; bleConnected=true; }
    void onDisconnect(BLEServer*) override {
        bleConnected=false; restartAdvertising=true;
        stopRequested=true; // same STOP path; no extra state
    }
};
class BleRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) override {
        const std::string value=characteristic->getValue();
        for (char ch : value) bleParser.feed(ch);
    }
};
static void initBle() {
    BLEDevice::init(BleName);
    BLEDevice::setMTU(185);
    auto* server=BLEDevice::createServer(); server->setCallbacks(new BleServerCallbacks);
    auto* service=server->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    bleTx=service->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLECharacteristic::PROPERTY_NOTIFY);
    bleTx->addDescriptor(new BLE2902);
    auto* rx=service->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(new BleRxCallbacks);
    service->start();
    auto* adv=BLEDevice::getAdvertising();
    adv->addServiceUUID(service->getUUID()); adv->setScanResponse(true); adv->start();
}
static void timerTick(void*) { if (controllerTask) xTaskNotifyGive(controllerTask); }
static void controlLoop(void*) {
    while (true) {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        if (stopRequested.exchange(false)) {
            xQueueReset(commands); replyLink=Link::Both; motion.stop(); hostPrintf("OK STOP\n");
        } else {
            Command command{};
            if (xQueueReceive(commands,&command,0)==pdTRUE) execute(command.line,command.link);
        }
        motion.update();
    }
}
static void flushOutput() {
    // Independent drains: a slow BLE subscriber cannot stall USB replies.
    static Output usbPending{}, blePending{};
    static size_t usbOffset=0, usbLength=0, bleOffset=0, bleLength=0;
    static uint32_t lastNotifyUs=0;
    if (!Serial) { usbLength=0; xQueueReset(usbOutputs); }
    else {
        if (!usbLength && xQueueReceive(usbOutputs,&usbPending,0)==pdTRUE) {
            usbLength=strlen(usbPending.line); usbOffset=0;
        }
        const int available=Serial.availableForWrite();
        if (usbLength && available>0) {
            const size_t count=min(size_t(available),usbLength-usbOffset);
            usbOffset+=Serial.write(reinterpret_cast<const uint8_t*>(usbPending.line)+usbOffset,count);
            if (usbOffset==usbLength) usbLength=0;
        }
    }
    if (!bleConnected) { bleLength=0; xQueueReset(bleOutputs); return; }
    if (!bleLength && xQueueReceive(bleOutputs,&blePending,0)==pdTRUE) {
        bleLength=strlen(blePending.line); bleOffset=0;
    }
    if (bleLength && uint32_t(micros()-lastNotifyUs)>=2000) {
        const size_t count=min(size_t(20),bleLength-bleOffset);
        bleTx->setValue(reinterpret_cast<uint8_t*>(blePending.line)+bleOffset,count);
        bleTx->notify(); lastNotifyUs=micros(); bleOffset+=count;
        if (bleOffset==bleLength) bleLength=0;
    }
}
#endif
void setup() {
    Serial.begin(115200);
#ifdef ARDUINO_ARCH_ESP32
    commands=xQueueCreate(16,sizeof(Command));
    usbOutputs=xQueueCreate(64,sizeof(Output)); bleOutputs=xQueueCreate(64,sizeof(Output));
    if (!commands || !usbOutputs || !bleOutputs) { motor.begin(); motor.stop(); Serial.println("ERR INIT memory"); return; }
#endif
    motion.begin();
#ifdef ARDUINO_ARCH_ESP32
    initBle();
    if (xTaskCreatePinnedToCore(controlLoop,"hammer-control",6144,nullptr,5,&controllerTask,1)!=pdPASS) {
        motor.stop(); Serial.println("ERR INIT task"); return;
    }
    esp_timer_create_args_t timerArgs{};
    timerArgs.callback=timerTick; timerArgs.name="hammer-1ms";
    if (esp_timer_create(&timerArgs,&controlTimer)!=ESP_OK ||
        esp_timer_start_periodic(controlTimer,ControlPeriodUs)!=ESP_OK) {
        motor.stop(); Serial.println("ERR INIT timer"); return;
    }
#endif
    hostPrintf("HELLO HAMMER_COAST_V7 protocol=2 period_us=1000\n");
    hostPrintf("INFO Commands: INFO, JOG signed_speed, HOME, ZERO, SET 60 200, START, RETRACT, FIRE, STOP, STATUS, PARAMS, TUNING, TUNE, TRACE\n");
}
void loop() {
    for (unsigned n=0;n<64 && Serial.available();++n) usbParser.feed(char(Serial.read()));
#ifdef ARDUINO_ARCH_ESP32
    if (restartAdvertising.exchange(false)) BLEDevice::startAdvertising();
    flushOutput(); delay(1);
#else
    if (stopRequested) { stopRequested=false; motion.stop(); hostPrintf("OK STOP\n"); }
    motion.update();
#endif
}
