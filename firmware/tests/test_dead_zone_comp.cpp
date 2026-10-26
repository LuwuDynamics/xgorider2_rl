#include "policy/dead_zone_comp.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace rider::policy;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::abort();}}while(0)

constexpr float kDt=0.005f;   // the 5 ms control period

static DeadZoneConfig config(float max_units=40.0f){DeadZoneConfig c;c.max_units=max_units;return c;}
static float run(DeadZoneState& s,const DeadZoneConfig& c,unsigned ticks,float u,bool quiet){
    float out=0;
    for(unsigned i=0;i<ticks;++i)out=applyDeadZonePreload(s,c,u,quiet,kDt);
    return out;
}

int main(){
    // Off (the default): nothing is added and the state stays clean, whatever the input.
    {
        DeadZoneState s;const DeadZoneConfig off;
        CHECK(off.max_units==0.0f);
        CHECK(run(s,off,1000,500.0f,true)==0.0f);
        CHECK(s.preload==0.0f&&s.sign==0&&s.flip_rate==0.0f&&s.trips==0);
    }
    // Bad input is ignored rather than propagated into the wheel command.
    {
        DeadZoneState s;const DeadZoneConfig c=config();
        CHECK(applyDeadZonePreload(s,c,std::numeric_limits<float>::quiet_NaN(),true,kDt)==0.0f);
        CHECK(applyDeadZonePreload(s,c,50.0f,true,0.0f)==0.0f);
        CHECK(applyDeadZonePreload(s,c,50.0f,true,-kDt)==0.0f);
        CHECK(s.preload==0.0f&&s.sign==0);
    }
    // The sign follows the command only beyond the hysteresis and is held inside it (a Schmitt trigger, not a sign()).
    {
        DeadZoneState s;const DeadZoneConfig c=config();
        run(s,c,1,5.0f,false);CHECK(s.sign==0);                       // inside the band before any push: undecided
        run(s,c,1,c.hysteresis+0.5f,false);CHECK(s.sign==1);
        run(s,c,1,-c.hysteresis+0.5f,false);CHECK(s.sign==1);          // still inside: held
        run(s,c,1,0.0f,false);CHECK(s.sign==1);
        run(s,c,1,-c.hysteresis-0.5f,false);CHECK(s.sign==-1);
        run(s,c,1,c.hysteresis-0.5f,false);CHECK(s.sign==-1);
    }
    // The preload grows only while the robot is quiet AND the command is beyond the hysteresis, at the configured rate,
    // and never above the cap.
    {
        DeadZoneState s;const DeadZoneConfig c=config(40.0f);
        run(s,c,400,30.0f,false);CHECK(s.preload==0.0f);               // being driven or pushed: no learning
        run(s,c,400,3.0f,true);CHECK(s.preload==0.0f);                 // standing still, nothing blocked: no learning
        const float out=run(s,c,200,30.0f,true);                       // 1 s quiet and pushing
        CHECK(std::fabs(s.preload-c.ramp_units_per_s*1.0f)<0.05f);
        CHECK(std::fabs(out-s.preload)<1e-6f);                         // positive push, positive offset
        run(s,c,20000,30.0f,true);CHECK(s.preload==40.0f&&s.trips==0);// capped
        const float neg=run(s,c,1,-30.0f,true);CHECK(neg==-40.0f);     // the offset carries the sign of the push
        run(s,c,1,0.0f,true);CHECK(applyDeadZonePreload(s,c,0.0f,true,kDt)==-40.0f);   // held inside the band
    }
    // Chatter: the sign flipping much faster than a standing robot does means the preload is larger than the dead zone.
    // Cut it, freeze it, and cut again if it keeps chattering.
    {
        DeadZoneState s;const DeadZoneConfig c=config(40.0f);
        run(s,c,20000,30.0f,true);CHECK(s.preload==40.0f);
        // a slow, healthy micro-oscillation (a flip every 200 ms = 5 flips/s) stays below the limit
        for(unsigned i=0;i<800;++i)applyDeadZonePreload(s,c,(i/40)%2?-20.0f:20.0f,true,kDt);
        CHECK(s.trips==0&&s.preload==40.0f);
        // a 50 Hz-ish flip (every 10 ms) trips within a fraction of a second
        unsigned first=0;
        for(unsigned i=0;i<400;++i){
            applyDeadZonePreload(s,c,(i/2)%2?-20.0f:20.0f,true,kDt);
            if(s.trips&&!first)first=i;
        }
        CHECK(first>0&&first<100);                                      // < 0.5 s
        CHECK(s.trips>=2&&s.locked);                                   // it keeps cutting while the chatter lasts
        CHECK(s.preload<40.0f*c.chatter_backoff+1e-3f);
        const float cut=s.preload;
        // The same fast flipping with a WIDE command swing is a push or a hand on the robot, not chatter: no trip, and the flips
        // that happened meanwhile do not count once the swing has died down.
        DeadZoneState w;run(w,c,20000,30.0f,true);CHECK(w.preload==40.0f);
        for(unsigned i=0;i<400;++i)applyDeadZonePreload(w,c,(i/2)%2?-60.0f:60.0f,true,kDt);
        CHECK(w.trips==0&&w.preload==40.0f&&w.flip_rate==0.0f);
        // ... and chatter after the push has passed is caught again (the peak memory decays in about 0.4 s)
        unsigned caught=0;
        for(unsigned i=0;i<800&&!caught;++i){applyDeadZonePreload(w,c,(i/2)%2?-20.0f:20.0f,true,kDt);if(w.trips)caught=i;}
        CHECK(caught>20&&caught<400&&w.preload<40.0f);
        // calm again: a locked preload does not grow back
        run(s,c,2000,30.0f,true);CHECK(s.preload==cut&&s.locked);
        // chatter with nothing learned (preload ~0) cannot trip: the relay output is zero anyway
        DeadZoneState z;
        for(unsigned i=0;i<400;++i)applyDeadZonePreload(z,c,(i/2)%2?-20.0f:20.0f,false,kDt);
        CHECK(z.trips==0&&z.preload==0.0f);
    }
    // A starting value: nothing has to be learned, the preload is there from the first tick; it is capped by max_units.
    {
        DeadZoneConfig c=config(40.0f);c.initial_units=12.0f;
        DeadZoneState s;startDeadZone(s,c);
        CHECK(s.preload==12.0f&&s.trips==0&&!s.locked);
        CHECK(applyDeadZonePreload(s,c,3.0f,false,kDt)==0.0f);               // undecided direction: nothing added yet
        CHECK(applyDeadZonePreload(s,c,20.0f,false,kDt)==12.0f);            // first push: the preload is already in
        c.initial_units=60.0f;startDeadZone(s,c);CHECK(s.preload==40.0f);    // capped
        c.max_units=0.0f;startDeadZone(s,c);CHECK(s.preload==0.0f);          // off
    }
    // Growth waits for evidence that the preload is too small: the controller keeps pushing (mean |command| over 1 s) without the
    // wheel giving way.  The micro-oscillation of a robot that stands still swings +-9 around zero and must not make it grow.
    {
        DeadZoneConfig c=config(55.0f);c.initial_units=12.0f;c.grow_mean_u=12.0f;
        DeadZoneState s;startDeadZone(s,c);
        for(unsigned i=0;i<4000;++i)applyDeadZonePreload(s,c,(i/50)%2?-9.0f:9.0f,true,kDt);        // +-9 at 2 Hz for 20 s
        CHECK(s.preload==12.0f&&s.trips==0);
        for(unsigned i=0;i<200;++i)applyDeadZonePreload(s,c,25.0f,true,kDt);                        // blocked: it keeps pushing
        CHECK(s.preload>12.0f);
        const float grown=s.preload;
        for(unsigned i=0;i<800;++i)applyDeadZonePreload(s,c,3.0f,true,kDt);                          // gives way: growth stops
        CHECK(s.preload<grown+3.2f);                                                                 // at most the tail while the mean decays
        CHECK(s.preload<20.0f);
    }
    // Resets: the relay and the flip average are forgotten, what was learned stays; the full reset clears everything.
    {
        DeadZoneState s;const DeadZoneConfig c=config(40.0f);
        run(s,c,2000,30.0f,true);
        for(unsigned i=0;i<400;++i)applyDeadZonePreload(s,c,(i/2)%2?-20.0f:20.0f,true,kDt);
        CHECK(s.trips>0);
        const float learned=s.preload;const auto trips=s.trips;
        resetDeadZoneRelay(s);
        CHECK(s.sign==0&&s.flip_rate==0.0f&&s.u_peak==0.0f&&s.u_abs_mean==0.0f&&s.preload==learned&&s.trips==trips&&s.locked);
        CHECK(run(s,c,1,3.0f,true)==0.0f);                              // undecided again: nothing is added
        resetDeadZoneLearning(s);
        CHECK(s.preload==0.0f&&s.trips==0&&!s.locked&&s.sign==0);
    }
    // Lowering the cap at run time takes effect immediately.
    {
        DeadZoneState s;DeadZoneConfig c=config(40.0f);
        run(s,c,20000,30.0f,true);CHECK(s.preload==40.0f);
        c.max_units=25.0f;CHECK(run(s,c,1,30.0f,true)==25.0f);
        c.max_units=0.0f;CHECK(run(s,c,1,30.0f,true)==0.0f&&s.sign==0);
    }
    std::printf("dead zone preload: %u checks passed\n",checks);
    return 0;
}
