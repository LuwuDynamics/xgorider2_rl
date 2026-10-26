#include "lift_guard.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace rider2::lift;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::abort();}}while(0)

// Both wheels at `speed`; pitch/roll and their rates as given (calm by default).
static State run(State s,const Config& c,unsigned ticks,float speed,float pitch=0,float roll=0,float pitch_rate=0,float roll_rate=0){
    for(unsigned i=0;i<ticks;++i){
        Input in;in.speed11=speed;in.speed21=speed;in.pitch_deg=pitch;in.roll_deg=roll;in.pitch_rate_deg_s=pitch_rate;in.roll_rate_deg_s=roll_rate;
        s=update(s,c,in);
    }
    return s;
}
static State runWheels(State s,const Config& c,unsigned ticks,float s11,float s21){
    for(unsigned i=0;i<ticks;++i){Input in;in.speed11=s11;in.speed21=s21;s=update(s,c,in);}
    return s;
}

int main(){
    const Config c;
    // Standing on the ground never gets near the limit (mean wheel speed peaked at 222 in hard pushes): no trip, ever.
    State s=run(State(),c,20000,200);
    CHECK(!s.active&&!s.fast&&s.trips==0);
    s=run(State(),c,20000,-200);
    CHECK(!s.active&&s.trips==0);
    // A hard sideways push spins the wheels against each other: one wheel fast, the mean and the half difference are not.
    // (Measured: -70 / -269.  The old per-wheel rule sat 10% below its limit there.)
    s=runWheels(State(),c,2000,-70,-269);
    CHECK(!s.active&&s.trips==0);
    s=runWheels(State(),c,2000,-269,-70);
    CHECK(!s.active&&s.trips==0);

    // A short burst above the limit is ignored; one tick longer than the persistence trips.
    s=run(State(),c,c.fast_ticks-1,400);
    CHECK(s.fast&&!s.active);
    s=run(s,c,1,50);
    CHECK(!s.fast&&!s.active&&s.fast_count==0);
    s=run(s,c,c.fast_ticks-1,400);
    CHECK(!s.active);
    s=run(s,c,1,400);
    CHECK(s.active&&s.trips==1&&s.calm_needed==c.calm_ticks);

    // The runaway measured in the air: 0 -> 540 within ~35 ticks; it must trip once, shortly after crossing the limit.
    s=State();
    unsigned tripped_at=0;
    for(unsigned t=0;t<60;++t){
        Input in;in.speed11=in.speed21=std::fmin(540.0f,t*18.0f);in.pitch_deg=-3;in.roll_deg=1;
        s=update(s,c,in);
        if(s.active&&!tripped_at)tripped_at=t;
    }
    CHECK(tripped_at>0&&tripped_at<=17+c.fast_ticks+1);    // crosses 300 at tick 17, then the persistence
    CHECK(s.trips==1);

    // Wheels spinning against each other in the air (large half difference, zero mean) count as well.
    s=runWheels(State(),c,c.fast_ticks,350,-350);
    CHECK(s.active);
    s=runWheels(State(),c,c.fast_ticks,-350,350);
    CHECK(s.active);

    // While active the guard stays on as long as the wheels spin, the body is tilted, or it is being moved by hand ...
    s=run(State(),c,c.fast_ticks,540);
    CHECK(s.active);
    s=run(s,c,1000,540);
    CHECK(s.active);                                // still racing
    s=run(s,c,1000,5,20,0);
    CHECK(s.active);                                // calm wheels but tilted 20 deg (carried around)
    s=run(s,c,1000,5,0,12);
    CHECK(s.active);                                // rolled 12 deg
    s=run(s,c,1000,45,0,0);
    CHECK(s.active);                                // wheels still turning at 45 mm/s
    s=run(s,c,1000,5,2,1,20,0);
    CHECK(s.active);                                // upright but pitching at 20 deg/s: a hand is moving it
    s=run(s,c,1000,5,2,1,0,20);
    CHECK(s.active);                                // rolling at 20 deg/s
    s=runWheels(s,c,1000,40,-40);
    CHECK(s.active);                                // half difference 40 (turning in the air)
    // ... and releases after calm_ticks of "upright, quiet and still", counted without interruption.
    s=run(s,c,c.calm_ticks-1,5,1,1,3,3);
    CHECK(s.active);
    s=run(s,c,1,5,1,1,9,3);                         // one disturbed tick starts the count again
    s=run(s,c,c.calm_ticks-1,5,1,1,3,3);
    CHECK(s.active);
    s=run(s,c,1,5,1,1,3,3);
    CHECK(!s.active&&s.trips==1&&s.since_release==0);

    // Tripping again right after a release (still in the air) makes the next release wait longer, but only that once;
    // after a quiet spell the short wait applies again.
    s=State();
    s=run(s,c,c.fast_ticks,540);
    CHECK(s.active&&s.calm_needed==c.calm_ticks);
    s=run(s,c,c.calm_ticks,0);
    CHECK(!s.active);
    s=run(s,c,c.retrip_window_ticks/2,0);           // released for half a second, then it runs away again
    s=run(s,c,c.fast_ticks,540);
    CHECK(s.active&&s.trips==2&&s.calm_needed==c.retrip_calm_ticks);
    s=run(s,c,c.retrip_calm_ticks-1,0);
    CHECK(s.active);
    s=run(s,c,1,0);
    CHECK(!s.active);
    s=run(s,c,20,0);
    s=run(s,c,c.fast_ticks,540);
    CHECK(s.active&&s.trips==3&&s.calm_needed==c.retrip_calm_ticks);               // no further escalation
    s=run(s,c,c.retrip_calm_ticks,0);
    CHECK(!s.active);
    s=run(s,c,c.retrip_window_ticks+10,0);          // a quiet spell longer than the window
    s=run(s,c,c.fast_ticks,540);
    CHECK(s.active&&s.trips==4&&s.calm_needed==c.calm_ticks);

    // Non-finite inputs neither trip nor release it.
    const float nan=std::numeric_limits<float>::quiet_NaN();
    s=run(State(),c,100,nan);
    CHECK(!s.active&&s.trips==0);
    s=run(State(),c,c.fast_ticks,540);
    s=run(s,c,1000,5,nan,0);
    CHECK(s.active);
    s=run(s,c,1000,5,0,0,nan,0);
    CHECK(s.active);

    // The counters saturate instead of wrapping.
    s=State();s.since_release=65535;s=update(s,c,Input());
    CHECK(s.since_release==65535);
    s=run(State(),c,70000,350);
    CHECK(s.active&&s.fast_count==c.fast_ticks);    // stops counting once it has tripped

    std::printf("lift guard: %u checks passed\n",checks);
}
