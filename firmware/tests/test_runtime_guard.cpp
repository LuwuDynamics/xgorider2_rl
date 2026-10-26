#include "runtime_guard.h"
#include <string>
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace rider::firmware;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::abort();}}while(0)
GuardInput good(){GuardInput i;i.now_ms=i.imu_ms=i.wheel11_ms=i.wheel21_ms=i.leg12_ms=i.leg22_ms=5000;
i.imu_valid=i.imu_calibrated=i.calibration_valid=i.wheel11_valid=i.wheel21_valid=i.leg12_valid=i.leg22_valid=true;return i;}
int main(){auto i=good();CHECK(outputGuard(i)==StopReason::None);CHECK(uprightForStart(i));
i.manual_stop=true;CHECK(outputGuard(i)==StopReason::Manual);
i=good();i.calibrating=true;CHECK(outputGuard(i)==StopReason::Calibration);
i=good();i.calibration_valid=false;CHECK(outputGuard(i)==StopReason::InvalidCalibration);
i=good();i.low_battery=true;CHECK(outputGuard(i)==StopReason::LowBattery);
i=good();i.imu_ms-=31;CHECK(outputGuard(i)==StopReason::Imu);
i=good();i.wheel21_ms-=101;CHECK(outputGuard(i)==StopReason::Actuators);
i=good();i.leg12_ms-=251;CHECK(outputGuard(i)==StopReason::Actuators);
i=good();i.control_overrun=true;CHECK(outputGuard(i)==StopReason::Timing);
i=good();i.pitch=46;CHECK(outputGuard(i)==StopReason::Tilt);
i=good();i.roll=61;CHECK(outputGuard(i)==StopReason::Tilt);
i=good();i.pitch=9;CHECK(outputGuard(i)==StopReason::None&&!uprightForStart(i));
i=good();i.pitch=std::numeric_limits<float>::quiet_NaN();CHECK(outputGuard(i)==StopReason::Imu);
i=good();i.imu_calibrated=false;CHECK(outputGuard(i)==StopReason::Startup);
i=good();i.falling=true;CHECK(outputGuard(i)==StopReason::Fall&&!uprightForStart(i));          // no restart in mid-air
i=good();i.falling=true;i.imu_ms-=31;CHECK(outputGuard(i)==StopReason::Imu);                      // sensor faults keep their priority
i=good();i.falling=true;i.manual_stop=true;CHECK(outputGuard(i)==StopReason::Manual);
i=good();i.falling=true;i.pitch=46;CHECK(outputGuard(i)==StopReason::Fall);                       // a fall is reported before the landing tilt
i=good();i.falling=false;CHECK(outputGuard(i)==StopReason::None&&uprightForStart(i));
CHECK(std::string(stopReasonName(StopReason::Fall))=="free_fall");
CHECK(rearmableStop(StopReason::Fall));
CHECK(!rearmableStop(StopReason::None));
CHECK(rearmableStop(StopReason::Startup));
CHECK(!rearmableStop(StopReason::Manual));
CHECK(!rearmableStop(StopReason::Calibration));
CHECK(!rearmableStop(StopReason::InvalidCalibration));
CHECK(rearmableStop(StopReason::Imu));
CHECK(rearmableStop(StopReason::Actuators));        // a dropped servo re-arms once it answers again (outputGuard gates that)
CHECK(rearmableStop(StopReason::Tilt));
CHECK(!rearmableStop(StopReason::LowBattery));
CHECK(rearmableStop(StopReason::Timing));
CHECK(rearmableStop(StopReason::InvalidControl));
CHECK(rearmableStop(StopReason::Policy));
// Freshness arithmetic remains valid after millis wraps; startup delay reoccurs
// for 3 seconds at the 49-day wrap, a conservative non-driving boundary.
i=good();i.now_ms=5;i.imu_ms=i.wheel11_ms=i.wheel21_ms=i.leg12_ms=i.leg22_ms=0xfffffff0;
CHECK(outputGuard(i)==StopReason::Startup);
// While driving, a "startup" can only be the IMU fusion having restarted: it is reported as the IMU stop it is (not silent); everything else is unchanged.
CHECK(stopReasonWhileDriving(StopReason::Startup,true)==StopReason::Imu);
CHECK(stopReasonWhileDriving(StopReason::Startup,false)==StopReason::Startup);              // a power-on wait stays a startup
for(auto r:{StopReason::None,StopReason::Manual,StopReason::Calibration,StopReason::Imu,StopReason::Actuators,StopReason::Tilt,StopReason::LowBattery,
            StopReason::Timing,StopReason::InvalidControl,StopReason::Policy,StopReason::Fall,StopReason::InvalidCalibration}){
    CHECK(stopReasonWhileDriving(r,true)==r&&stopReasonWhileDriving(r,false)==r);
}
std::printf("runtime guard: %u checks passed\n",checks);}
