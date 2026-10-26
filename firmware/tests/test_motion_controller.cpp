#include "motion_controller.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace rider::firmware;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::abort();}}while(0)
int main() {
    MotionController c;MotionRequest r;MotionInput i;
    i.mode=1;i.calibrated=true;i.now_ms=3001;i.zero_a=i.zero_b=512;
    auto first=c.step(i,r);CHECK(first.valid);CHECK(first.command.filtered_height>0);
    i.mode=0;CHECK(!c.step(i,r).valid);
    i.mode=1;auto fresh=c.step(i,r);CHECK(fresh.valid&&fresh.balance==first.balance);
    i.speed11=std::numeric_limits<float>::quiet_NaN();CHECK(!c.step(i,r).valid);i.speed11=0;
    r.command.max_forward=0;r.command.max_turn=0;r.command.automatic_lean=true;CHECK(c.step(i,r).valid);
    r={};r.roll_phase=std::numeric_limits<float>::quiet_NaN();CHECK(!c.step(i,r).valid);
    r={};r.height_phase=std::numeric_limits<float>::infinity();CHECK(!c.step(i,r).valid);
    r={};c.reset();i.encoder11=100;i.encoder21=200;i.leg12_position=400;
    auto original=c.step(i,r);CHECK(original.position!=original.physical_position);
    r.original_position_feedback=false;auto physical=c.step(i,r);CHECK(physical.position==physical.physical_position);
    r={};c.reset(0);i={};i.mode=1;i.calibrated=true;i.zero_a=i.zero_b=512;i.now_ms=3000;
    CHECK(c.step(i,r).hold.mode==1);i.now_ms+=501;CHECK(c.step(i,r).hold.mode==2);
    r.command.forward=255;CHECK(c.step(i,r).hold.mode==0);
    // An action's hold reset must not collapse the leg-height filter or erase
    // the control PID histories. Compare cloned controllers around that event.
    c.reset();r={};i.now_ms=1000;i.pitch_deg=.2f;i.speed11=.3f;i.speed21=.2f;
    for(unsigned n=0;n<200;++n){i.now_ms+=4;CHECK(c.step(i,r).valid);}
    i.now_ms=2500;
    MotionController retained=c,stopped=c;
    MotionRequest retainedRequest=r,stoppedRequest=r;
    c.resetHold(i.now_ms);stopped.resetStoppedPid();
    i.now_ms+=4;
    const auto afterHold=c.step(i,r),withoutReset=retained.step(i,retainedRequest),afterStop=stopped.step(i,stoppedRequest);
    CHECK(afterHold.valid&&withoutReset.valid&&afterStop.valid);
    CHECK(afterHold.command.filtered_height==withoutReset.command.filtered_height);
    CHECK(afterHold.balance==withoutReset.balance);
    CHECK(afterHold.hold.mode==1&&afterHold.hold.last_disabled_ms==i.now_ms-4);
    CHECK(afterStop.command.filtered_height==withoutReset.command.filtered_height);
    CHECK(afterStop.balance!=withoutReset.balance);
    c.reset();r={};r.roll_phase=r.height_phase=10;
    CHECK(c.step(i,r).valid);CHECK(r.roll_phase==20&&r.height_phase==20);
    r.leveling=true;r.heading_hold=true;
    for(unsigned n=0;n<20000;++n) {
        i.now_ms+=4;i.pitch_deg=std::sin(n*.003f)*12;i.roll_deg=std::sin(n*.007f)*10;
        i.yaw_deg=std::sin(n*.001f)*45;i.speed11=std::sin(n*.02f)*120;i.speed21=std::sin(n*.021f)*120;
        i.pitch_rate_rad_s=std::cos(n*.003f);r.command.forward=uint8_t(n%256);r.command.turn=uint8_t((n/3)%256);
        auto o=c.step(i,r);CHECK(o.valid);CHECK(std::abs(int(o.wheels.id11))<=1023&&std::abs(int(o.wheels.id21))<=1023);
        CHECK(o.legs.servo_a<=1023&&o.legs.servo_b<=1023);
    }
    std::printf("motion controller: %u checks passed\n",checks);
}
