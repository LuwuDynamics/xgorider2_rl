// Replays a recording through the SAME free-fall detector the firmware uses (src/fall_guard.h).
// Input on stdin: one "ms g" pair per line (firmware milliseconds, total acceleration in g).  Each sample is held for ~18 ms and
// fed at a 5 ms control tick (a 55 Hz trace sample covers ~18 ms): the worst case for dips, which then last at least one sample.
// Output: trips, longest run below the limit (ms), minimum g.   Optional arguments: limit in g, persistence in ms.
#include "fall_guard.h"
#include <cstdio>
#include <cstdlib>
using namespace rider2::fall;
int main(int argc,char** argv){
    Config c;
    if(argc>2){c.freefall_g=float(std::atof(argv[1]));c.freefall_ms=float(std::atof(argv[2]));}
    State s;double ms=0,g=1,min_g=1e9,longest=0,run=0;
    while(std::scanf("%lf %lf",&ms,&g)==2){
        if(g<min_g)min_g=g;
        Input in;in.accel_g=float(g);in.dt_ms=5.0f;
        for(double t=0;t<18.2;t+=5.0){
            s=update(s,c,in);
            if(s.low){run+=5.0;if(run>longest)longest=run;}else run=0;
        }
    }
    std::printf("%u %.0f %.3f\n",s.trips,longest,min_g);
    return 0;
}
