#include "led_status.h"
#include <cstdio>
#include <cstdlib>
using namespace rider::board;
using rider::firmware::StopReason;
using rider::firmware::ledFaultFor;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::abort();}}while(0)

static bool solid(const LedPixels& p,unsigned r,unsigned g,unsigned b){
    for(const auto& px:p)if(px.r!=r||px.g!=g||px.b!=b)return false;
    return true;
}
static LedControl control(LedFault fault,unsigned mode){LedControl c;c.fault=fault;c.robot_mode=std::uint8_t(mode);return c;}

int main(){
    // Which indication each stop shows: balancing never shows one, every protective stop does, a plain power-on does not.
    CHECK(ledFaultFor(true,StopReason::Tilt)==LedFault::None&&ledFaultFor(true,StopReason::Fall)==LedFault::None);   // balancing again
    CHECK(ledFaultFor(false,StopReason::Tilt)==LedFault::Tilt);
    CHECK(ledFaultFor(false,StopReason::Fall)==LedFault::Fall);
    for(auto r:{StopReason::Imu,StopReason::Actuators,StopReason::Timing,StopReason::InvalidControl,StopReason::Policy,StopReason::InvalidCalibration})
        CHECK(ledFaultFor(false,r)==LedFault::Other);
    CHECK(ledFaultFor(false,StopReason::Manual)==LedFault::Manual);
    CHECK(ledFaultFor(false,StopReason::Startup)==LedFault::None&&ledFaultFor(false,StopReason::None)==LedFault::None);
    CHECK(ledFaultFor(false,StopReason::LowBattery)==LedFault::None&&ledFaultFor(false,StopReason::Calibration)==LedFault::None);

    LedPixels out;
    // No fault, balancing and low battery: nothing is drawn by this function (the original renderLeds decides).
    CHECK(!renderFaultLeds(control(LedFault::None,0),0,out));
    CHECK(!renderFaultLeds(control(LedFault::Tilt,1),0,out)&&!renderFaultLeds(control(LedFault::Fall,1),0,out));
    CHECK(!renderFaultLeds(control(LedFault::Other,250),0,out)&&!renderFaultLeds(control(LedFault::Tilt,250),0,out));
    // Fallen over: solid red in every stopped mode (including the modes the original leaves untouched: 251..254) and at any time.
    for(unsigned mode:{0u,251u,252u,253u,254u,255u})
        for(unsigned t:{0u,50u,123u,999u,12345u}){
            CHECK(renderFaultLeds(control(LedFault::Tilt,mode),t,out)&&solid(out,200,0,0));
        }
    // Free fall: red flicker at 5 Hz (100 ms on, 100 ms off).
    CHECK(renderFaultLeds(control(LedFault::Fall,0),0,out)&&solid(out,255,0,0));
    CHECK(renderFaultLeds(control(LedFault::Fall,0),99,out)&&solid(out,255,0,0));
    CHECK(renderFaultLeds(control(LedFault::Fall,0),100,out)&&solid(out,0,0,0));
    CHECK(renderFaultLeds(control(LedFault::Fall,0),199,out)&&solid(out,0,0,0));
    CHECK(renderFaultLeds(control(LedFault::Fall,253),200,out)&&solid(out,255,0,0));
    // Another protective stop: slow red blink at 1 Hz.
    CHECK(renderFaultLeds(control(LedFault::Other,252),0,out)&&solid(out,200,0,0));
    CHECK(renderFaultLeds(control(LedFault::Other,252),499,out)&&solid(out,200,0,0));
    CHECK(renderFaultLeds(control(LedFault::Other,252),500,out)&&solid(out,0,0,0));
    CHECK(renderFaultLeds(control(LedFault::Other,252),999,out)&&solid(out,0,0,0));
    CHECK(renderFaultLeds(control(LedFault::Other,252),1000,out)&&solid(out,200,0,0));
    // Stopped by a command: solid amber.
    for(unsigned t:{0u,333u,777u})CHECK(renderFaultLeds(control(LedFault::Manual,254),t,out)&&solid(out,200,70,0));
    // The three fault patterns are different from each other at the same instant mix: at t=150 the flicker is off while the tilt is on.
    {
        LedPixels a,b,c;
        CHECK(renderFaultLeds(control(LedFault::Tilt,0),150,a)&&renderFaultLeds(control(LedFault::Fall,0),150,b)&&renderFaultLeds(control(LedFault::Manual,0),150,c));
        CHECK(!solid(a,0,0,0)&&solid(b,0,0,0)&&c[0].g>0);
    }
    // The original rendering is unchanged by the new field: with no fault renderLeds behaves as before (mode 0 red, others untouched).
    {
        LedPixels user;LedPixels p;
        LedControl c;c.robot_mode=0;CHECK(renderLeds(c,0,user,p)&&solid(p,200,0,0));
        c.robot_mode=253;CHECK(!renderLeds(c,0,user,p));
        c.robot_mode=252;c.fault=LedFault::Other;CHECK(!renderLeds(c,0,user,p));     // the original function ignores `fault`; BoardSupport::updateLeds asks renderFaultLeds first
    }
    std::printf("led status: %u checks passed\n",checks);
    return 0;
}
