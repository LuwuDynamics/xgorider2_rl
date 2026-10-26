#include "fall_guard.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>
using namespace rider2::fall;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::abort();}}while(0)

static State run(State s,const Config& c,unsigned ticks,float g,float dt=5.0f){
    for(unsigned i=0;i<ticks;++i){Input in;in.accel_g=g;in.dt_ms=dt;s=update(s,c,in);}
    return s;
}
// Samples of a recording (g), 18.2 ms apart as in the 55 Hz USB trace.
static State replay(State s,const Config& c,const std::vector<float>& samples,float dt=18.2f){
    for(float g:samples){Input in;in.accel_g=g;in.dt_ms=dt;s=update(s,c,in);}
    return s;
}

int main(){
    const Config c;
    CHECK(c.freefall_g==0.5f&&c.freefall_ms==60.0f&&c.landed_g==0.8f&&c.landed_ms==40.0f);   // the firmware default hold is 60 ms
    // Standing (1 g) and normal balancing never trip.
    State s=run(State(),c,20000,1.0f);
    CHECK(!s.falling&&!s.low&&s.trips==0&&s.low_ms==0.0f);
    // Handling and pushes: 0.6 .. 1.6 g (the recordings: 0.7 .. 1.8, dips below 0.6 g last one sample), a rough pattern for a minute.
    {
        State h;
        for(unsigned i=0;i<12000;++i){
            const float g=1.1f+0.5f*std::sin(0.01f*float(i))*std::sin(0.037f*float(i));
            Input in;in.accel_g=g;in.dt_ms=5.0f;h=update(h,c,in);
        }
        CHECK(h.trips==0&&!h.falling);
    }
    // Real recordings of 2026-10-22: the deepest dips of 560 s of handling, each a single 55 Hz sample (a push; the wheels
    // touching down).  They must never count as a fall.
    CHECK(replay(State(),c,{0.95f,0.85f,1.03f,1.0f,0.93f,0.6f,0.23f,0.71f,1.04f,1.3f,1.66f,1.83f,1.35f}).trips==0);
    CHECK(replay(State(),c,{1.0f,1.0f,1.0f,1.01f,1.01f,1.25f,0.29f,0.94f,0.96f,1.24f,1.44f,1.17f,1.01f}).trips==0);
    // Even two such samples in a row (36 ms) are not a fall.
    CHECK(replay(State(),c,{1.0f,0.25f,0.3f,1.0f}).trips==0);

    // The persistence: 55 ms below the limit is not enough, 60 ms trips on the 12th 5 ms tick.
    s=run(State(),c,11,0.05f);
    CHECK(s.low&&!s.falling&&s.trips==0&&std::fabs(s.low_ms-55.0f)<1e-3f);
    s=run(s,c,1,0.05f);
    CHECK(s.falling&&s.trips==1);
    // The limit itself: 0.45 g (a tumbling fall reads a little above zero) trips, 0.55 g does not.
    CHECK(run(State(),c,40,0.45f).falling);
    CHECK(!run(State(),c,400,0.55f).falling&&!run(State(),c,400,0.55f).low);
    // It has to be continuous: two 55 ms dips with one normal sample between them are not a fall.
    s=run(State(),c,11,0.1f);s=run(s,c,1,1.0f);s=run(s,c,11,0.1f);
    CHECK(!s.falling&&s.trips==0);

    // A table drop: 1 s standing, 400 ms of free fall, the impact, then lying still.  Exactly one trip, within one tick of the
    // persistence, and the fall ends only once the acceleration has been back above 0.8 g for 40 ms.
    s=run(State(),c,200,1.0f);
    unsigned tripped_at=0;
    for(unsigned t=0;t<80;++t){            // 80 ticks = 400 ms
        Input in;in.accel_g=0.04f;in.dt_ms=5.0f;s=update(s,c,in);
        if(s.falling&&!tripped_at)tripped_at=t+1;
    }
    CHECK(tripped_at==12&&s.trips==1&&s.falling);
    s=run(s,c,1,6.0f);                      // the impact spike alone does not end it (needs 40 ms)
    CHECK(s.falling);
    s=run(s,c,6,1.0f);                      // 5 + 30 ms above the limit
    CHECK(s.falling);
    s=run(s,c,1,1.0f);                      // 40 ms
    CHECK(!s.falling&&s.trips==1&&s.high_ms==0.0f);
    // A second drop is a second trip.
    s=run(s,c,100,0.05f);
    CHECK(s.falling&&s.trips==2);
    // One normal-looking sample in the middle of a fall (a spin glitch) does not end it.
    s=run(s,c,1,1.2f);s=run(s,c,20,0.05f);
    CHECK(s.falling&&s.trips==2);

    // Irregular control periods (5..8 ms) count by elapsed time, not by ticks.
    s=State();
    float elapsed=0;
    for(unsigned t=0;t<40&&!s.falling;++t){const float dt=5.0f+float(t%4);Input in;in.accel_g=0.1f;in.dt_ms=dt;s=update(s,c,in);elapsed+=dt;}
    CHECK(s.falling&&elapsed>=60.0f&&elapsed<70.0f);

    // Samples that are not evidence: a stall, a missing read, NaN or a zero interval restart the count (and never trip).
    s=run(State(),c,10,0.05f);
    {Input in;in.accel_g=0.05f;in.dt_ms=200.0f;s=update(s,c,in);}       // 200 ms stall
    CHECK(!s.falling&&s.low_ms==0.0f&&!s.low);
    s=run(s,c,10,0.05f);
    {Input in;in.accel_g=0.05f;in.dt_ms=5.0f;in.fresh=false;s=update(s,c,in);}
    CHECK(s.low_ms==0.0f);
    s=run(s,c,10,0.05f);
    {Input in;in.accel_g=std::numeric_limits<float>::quiet_NaN();in.dt_ms=5.0f;s=update(s,c,in);}
    CHECK(s.low_ms==0.0f&&!s.falling);
    {Input in;in.accel_g=0.05f;in.dt_ms=0.0f;s=update(s,c,in);}
    CHECK(s.low_ms==0.0f);
    s=run(s,c,11,0.05f);CHECK(!s.falling);s=run(s,c,1,0.05f);CHECK(s.falling);

    // Non-finite input never latches a fall by itself, and a fall already latched survives unusable samples.
    s=State();s=run(s,c,100,0.05f);CHECK(s.falling);
    {Input in;in.fresh=false;s=update(s,c,in);}
    CHECK(s.falling&&s.trips==1);

    // A stricter configuration (what the console could set) behaves accordingly.
    Config strict;strict.freefall_g=0.3f;strict.freefall_ms=150.0f;
    CHECK(!run(State(),strict,20,0.2f).falling&&run(State(),strict,30,0.2f).falling&&!run(State(),strict,400,0.4f).falling);


    // The hold time can be changed at run time (`!fallguard ms N`): the range check and the console line parser.
    CHECK(kMinHoldMs==20.0f&&kMaxHoldMs==200.0f);
    CHECK(validHoldMs(20.0f)&&validHoldMs(40.0f)&&validHoldMs(60.0f)&&validHoldMs(200.0f));
    CHECK(!validHoldMs(19.9f)&&!validHoldMs(200.1f)&&!validHoldMs(0.0f)&&!validHoldMs(-60.0f));
    CHECK(!validHoldMs(std::numeric_limits<float>::quiet_NaN())&&!validHoldMs(std::numeric_limits<float>::infinity()));
    {
        float ms=123.0f;
        CHECK(parseHoldCommand("!fallguard ms",ms)==1&&ms==123.0f);                  // query: nothing is stored
        CHECK(parseHoldCommand("!fallguard ms   ",ms)==1&&ms==123.0f);
        CHECK(parseHoldCommand("!fallguard ms 40",ms)==2&&ms==40.0f);
        CHECK(parseHoldCommand("!fallguard ms   45.5  ",ms)==2&&ms==45.5f);
        CHECK(parseHoldCommand("!fallguard ms 20",ms)==2&&ms==20.0f);
        CHECK(parseHoldCommand("!fallguard ms 200",ms)==2&&ms==200.0f);
        CHECK(parseHoldCommand("!fallguard ms 1e2",ms)==2&&ms==100.0f);
        ms=77.0f;                                                                     // rejected lines leave the value alone
        CHECK(parseHoldCommand("!fallguard ms 10",ms)==-1&&ms==77.0f);
        CHECK(parseHoldCommand("!fallguard ms 250",ms)==-1&&ms==77.0f);
        CHECK(parseHoldCommand("!fallguard ms -5",ms)==-1&&ms==77.0f);
        CHECK(parseHoldCommand("!fallguard ms abc",ms)==-1&&ms==77.0f);
        CHECK(parseHoldCommand("!fallguard ms 40x",ms)==-1&&ms==77.0f);
        CHECK(parseHoldCommand("!fallguard ms 40 50",ms)==-1&&ms==77.0f);
        CHECK(parseHoldCommand("!fallguard ms nan",ms)==-1&&ms==77.0f);
        CHECK(parseHoldCommand("!fallguard ms inf",ms)==-1&&ms==77.0f);
        CHECK(parseHoldCommand("!fallguard msx",ms)==0&&parseHoldCommand("!fallguard on",ms)==0&&parseHoldCommand("!fallguard",ms)==0);
        CHECK(parseHoldCommand("!liftguard ms 40",ms)==0&&parseHoldCommand("",ms)==0&&parseHoldCommand(nullptr,ms)==0&&ms==77.0f);
    }
    // A shorter hold catches shorter dips, and only those.
    {
        Config fast;fast.freefall_ms=40.0f;
        CHECK(!run(State(),fast,7,0.05f).falling&&run(State(),fast,8,0.05f).falling);            // 35 ms no, 40 ms yes
        CHECK(!run(State(),c,11,0.05f).falling);                                                 // the default (60 ms) ignores 55 ms
        Config minimum;minimum.freefall_ms=kMinHoldMs;
        CHECK(!replay(State(),minimum,{1.0f,0.23f,1.0f}).falling);                              // one 55 Hz sample (18 ms) is never a fall, even at the shortest hold
        CHECK(replay(State(),minimum,{1.0f,0.25f,0.3f,1.0f}).trips==1);                         // two in a row (36 ms) are, at 20 ms
        CHECK(replay(State(),fast,{1.0f,0.25f,0.3f,1.0f}).trips==0);                            // ... but not at 40 ms
        // Hand-moved drop of the third robot (2026-10-23, trace drop-test-1, t=17.5 s, 15 ms apart): dips to 0.24 g for ~75 ms.  The
        // replay holds each sample 18.2 ms, so it is the pessimistic estimate; the real detector (5 ms ticks) did not trip at 80 ms.
        const std::vector<float> hand_drop={1.16f,0.95f,0.78f,0.61f,0.40f,0.24f,0.35f,0.32f,0.40f,0.60f,0.46f,1.0f,1.2f,1.77f,2.09f};
        CHECK(replay(State(),fast,hand_drop).trips==1);
        // Lowering and holding by hand never goes below 0.5 g at any hold time.
        const std::vector<float> slow_lowering={1.0f,0.95f,0.85f,0.79f,0.8f,0.9f,1.0f,1.3f,1.0f,0.59f,0.75f,1.0f};
        CHECK(replay(State(),minimum,slow_lowering).trips==0);
    }


    // Diagnostics: the lowest sample and the longest continuous low stretch are remembered and survive the end of a fall.
    {
        State d;
        d=run(d,c,50,1.0f);CHECK(d.min_g==1.0f&&d.longest_low_ms==0.0f);
        d=run(d,c,7,0.3f);d=run(d,c,3,1.0f);d=run(d,c,4,0.12f);d=run(d,c,50,1.0f);
        CHECK(std::fabs(d.min_g-0.12f)<1e-6f&&std::fabs(d.longest_low_ms-35.0f)<1e-3f&&!d.falling);   // 7 ticks of 5 ms
        d=run(d,c,20,0.04f);CHECK(d.falling&&d.trips==1&&d.min_g<0.05f&&d.longest_low_ms>=100.0f-1e-3f);
        d=run(d,c,20,1.0f);CHECK(!d.falling&&d.min_g<0.05f&&d.longest_low_ms>=100.0f-1e-3f);          // landed: the statistics stay
        State u;{Input in;in.accel_g=0.0f;in.fresh=false;u=update(u,c,in);}                            // unusable samples are not evidence
        CHECK(u.min_g==1000.0f&&u.longest_low_ms==0.0f);
    }
    // The deep detector (below 0.061 g = 1000 counts for 20 ms): where the IMU validity gate declares a sample unusable.
    {
        const Config deep=deepConfig();
        CHECK(std::fabs(deep.freefall_g-1000.0f/16384.0f)<1e-9f&&deep.freefall_ms==6.0f&&deep.landed_g==0.8f&&deep.landed_ms==40.0f);
        CHECK(!run(State(),deep,1,0.03f).falling&&run(State(),deep,2,0.03f).falling);                  // one read is a glitch, two reads in a row (IMU reads are ~3 ms apart) are a fall
        {Input in;in.accel_g=0.03f;in.dt_ms=3.0f;State d=update(State(),deep,in);CHECK(!d.falling&&d.low);d=update(d,deep,in);CHECK(d.falling&&d.trips==1);}   // the real read interval
        CHECK(!run(run(State(),deep,1,0.03f),deep,1,1.0f).falling);                                   // the glitch followed by a normal read is not a fall
        CHECK(!run(State(),deep,400,0.07f).falling);                                                  // 0.07 g is not "deep"
        CHECK(replay(State(),deep,{1.0f,0.02f,1.0f}).trips==1);                                       // a held 55 Hz trace sample is already several reads
        CHECK(replay(State(),deep,{1.0f,0.02f,0.03f,1.0f}).trips==1);
        // The real release of 2026-10-23 (trace release-test-1, 15 ms between samples): 1.0 g falling to 0.18 g in 50 ms, then the IMU gate
        // (below 0.061 g) stopped the robot.  The first 8 samples are the recorded ones, the last two what the gate saw.
        const std::vector<float> to_018={1.0f,0.99f,0.96f,0.92f,0.86f,0.69f,0.43f,0.18f};
        std::vector<float> to_003=to_018;to_003.push_back(0.05f);to_003.push_back(0.03f);
        CHECK(replay(State(),c,to_018).trips==0&&replay(State(),deep,to_018).trips==0);               // nothing yet at 0.18 g
        CHECK(replay(State(),deep,to_003).trips==1);                                                  // the deep path trips 2 samples into the real free fall
    }

    // The IMU fusion restarts after an invalid read or after > 30 ms without a valid one; a read that only showed ~0 g does not restart it by itself.
    CHECK(!imuFusionRestart(true,false,3)&&!imuFusionRestart(true,false,30));            // normal reads
    CHECK(imuFusionRestart(true,false,31)&&imuFusionRestart(false,false,3));             // stale, or the last read was invalid for another reason
    CHECK(!imuFusionRestart(false,true,3)&&!imuFusionRestart(false,true,30));            // one or a few reads of ~0 g: keep the attitude
    CHECK(imuFusionRestart(false,true,31)&&imuFusionRestart(false,true,200));            // a real fall (> 30 ms without a valid read): restart afterwards

    std::printf("fall guard: %u checks passed\n",checks);
    return 0;
}
