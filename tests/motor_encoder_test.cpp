// Runs the production encoder/controller against a hardware-counter model.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <driver/pcnt.h>
#include <soc/gpio_struct.h>
#include "hall_encoder.h"
#include "motion_controller.h"
#include "trace_integrity.h"
#include "pcnt_count.h"
uint32_t fakeTime=0;
void (*fakeInterrupts[64])(){};
FakeGpio GPIO;
pcnt_config_t fakeChannels[2]{};
int16_t fakePcnt=0;
bool fakePcntFailure=false;
void hostPrintf(const char*,...) {}

// Model both PCNT channels using the actual production channel configuration.
void transition(unsigned state,bool interrupt=true) {
    const uint32_t old=GPIO.in1.val;
    const uint32_t next=((state>>1)&1U)<<(39-32) | (state&1U)<<(40-32);
    GPIO.in1.val=next;
    for (const auto& c:fakeChannels) {
        const bool before=(old>>(c.pulse_gpio_num-32))&1;
        const bool after=(next>>(c.pulse_gpio_num-32))&1;
        if (before==after) continue;
        const bool control=(next>>(c.ctrl_gpio_num-32))&1;
        fakePcnt+=(after?c.pos_mode:c.neg_mode)*(control?c.hctrl_mode:c.lctrl_mode);
        if (fakePcnt==c.counter_h_lim || fakePcnt==c.counter_l_lim) fakePcnt=0;
    }
    fakeTime+=200;
    if (interrupt && old!=next) fakeInterrupts[39]();
}
unsigned phase=0;
void edges(HallEncoder& encoder,int count,bool interrupt=true,MotionController* motion=nullptr) {
    static const unsigned positive[]={0,1,3,2};
    const int direction=count>=0?1:-1;
    for (int i=0;i<std::abs(count);++i) {
        phase=(phase+direction+4)%4;
        transition(positive[phase],interrupt);
        // Worst model edge rate=5000/s, control polls at 1ms.
        if (i%5==4) { if (motion) motion->update(); else encoder.updateSpeed(.001f); }
    }
    encoder.updateSpeed(.001f);
}
int main() {
    assert(traceCrc32("123456789",9)==0xcbf43926U);
    assert(pcntDelta(1,29999)==2 && pcntDelta(-1,-29999)==-2);
    Drv8871Motor motor({41,42,0,1,20000,10});
    HallEncoder encoder({39,40,16,27});
    MotionController motion(motor,encoder); motion.begin();
    assert(encoder.ready() && motion.zeroHere());
    edges(encoder,240); assert(encoder.getCount()==240);
    assert(std::fabs(encoder.getAngleDeg()-50)<.001f);
    edges(encoder,-240); assert(encoder.getCount()==0);
    // Lose six forward ISR edges on every round trip; position must not drift.
    for (int shot=0;shot<500;++shot) {
        edges(encoder,-240);
        edges(encoder,6,false); edges(encoder,234);
        assert(encoder.getCount()==0);
    }
    assert(encoder.isrCount()!=encoder.getCount());
    edges(encoder,2,false); fakeInterrupts[39](); edges(encoder,-2);
    assert(encoder.invalidTransitions()>0);
    edges(encoder,65000); assert(encoder.getCount()==65000);
    edges(encoder,-130000); assert(encoder.getCount()==-65000);
    edges(encoder,65000); assert(encoder.getCount()==0);
    edges(encoder,100); const int16_t before=fakePcnt;
    encoder.reset(); assert(fakePcnt==before && encoder.getCount()==0);
    edges(encoder,4); assert(encoder.getCount()==4);
    encoder.reset();
    // Missed interrupts do not prevent the real state machine reaching READY.
    motion.update(); assert(motion.start()); edges(encoder,-288,false,&motion);
    for(int i=0;i<500 && motion.state()!=MotionController::State::Strike;++i) {
        fakeTime+=1000; motion.update();
    }
    assert(motion.state()==MotionController::State::Strike);
    // Missing contact must brake, invalidate ZERO, and forbid another strike.
    for (int i=0;i<1900 && motion.state()==MotionController::State::Strike;++i) {
        fakeTime+=1000; motion.update();
    }
    assert(motion.state()==MotionController::State::Idle && !motion.zeroed());
    assert(!std::strcmp(motion.lastReason(),"TIMEOUT_NO_DROP"));
    assert(!std::strcmp(motor.outputMode(),"BRAKE"));
    assert(!motion.start());
    assert(motion.zeroHere() && motion.start());
    edges(encoder,-288,false,&motion);
    for(int i=0;i<500 && motion.state()!=MotionController::State::Strike;++i) {
        fakeTime+=1000; motion.update();
    }
    assert(motion.state()==MotionController::State::Strike);
    // A missing target cannot cause a full rotation, regardless of timeout/drop.
    edges(encoder,335,false,&motion);
    assert(motion.state()==MotionController::State::Idle && !motion.zeroed());
    assert(!std::strcmp(motion.lastReason(),"STRIKE_TRAVEL_LIMIT"));
    assert(!std::strcmp(motor.outputMode(),"BRAKE"));
    for(int i=0;i<100;++i){fakeTime+=1000;motion.update();}
    motion.stop();
    assert(motion.state()==MotionController::State::Idle && !motion.zeroed());
    assert(!std::strcmp(motor.outputMode(),"BRAKE"));
    assert(motion.zeroHere() && motion.start());
    fakeTime+=60000; motion.update();
    assert(motion.state()==MotionController::State::Idle && !motion.zeroed());
    assert(!std::strcmp(motion.lastReason(),"CONTROL_OR_SENSOR"));
    assert(!std::strcmp(motor.outputMode(),"BRAKE"));
    // Static friction can arrest retract outside the old 20-degree I zone.
    // A P-only controller stays below the modeled 650 PWM breakaway threshold.
    assert(motion.zeroHere());
    assert(motion.setSettings({50,200}) && motion.retract());
    edges(encoder,-120,false,&motion); // halfway, -25 degrees
    bool brokeFree=false;
    const uint32_t forceStart=fakeTime;
    for(int i=0;i<650 && motion.state()!=MotionController::State::Ready;++i) {
        fakeTime+=1000; motion.update();
        if (!brokeFree && motion.pwm()<=-650) {
            assert(fakeTime-forceStart<750000);
            brokeFree=true; edges(encoder,-120,false,&motion);
        }
    }
    assert(brokeFree && motion.state()==MotionController::State::Ready);
    motion.stop(); assert(motion.zeroHere());
    // A rigid obstruction must still stop after 750ms despite the faster I term.
    assert(motion.retract());
    for(int i=0;i<800 && motion.state()!=MotionController::State::Idle;++i) {
        fakeTime+=1000; motion.update();
    }
    assert(motion.state()==MotionController::State::Idle && !motion.zeroed());
    assert(!std::strcmp(motion.lastReason(),"RETRACT_NO_PROGRESS"));
    assert(!std::strcmp(motor.outputMode(),"BRAKE"));
    assert(motion.home());
    edges(encoder,65,false,&motion);
    assert(motion.state()==MotionController::State::Idle && !motion.zeroed());
    assert(!std::strcmp(motion.lastReason(),"HOME_TRAVEL_LIMIT"));
    assert(!std::strcmp(motor.outputMode(),"BRAKE"));
    motion.stop("BLE_DISCONNECT");
    assert(!std::strcmp(motion.lastReason(),"BLE_DISCONNECT"));
    // A failed hardware counter must not allow a new reference/action.
    fakePcntFailure=true; encoder.updateSpeed(.001f);
    assert(!encoder.ready() && !motion.zeroHere() && !motion.jog(40) && !motion.home());
    std::puts("PCNT: direction, 500 lossy round trips, rollover, ZERO, READY, strike timeout, STOP, control gap, far-target stiction, obstruction and failure passed");
}
