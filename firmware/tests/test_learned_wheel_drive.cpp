#include "learned_wheel_drive.h"
#include "policy/trained_weights.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
using namespace rider::policy;
using rider::firmware::LearnedWheelDrive;
static uint32_t ticks=0,cost=100;
static uint32_t clockNow(){const auto value=ticks;ticks+=cost;return value;}
static void stopped(const Decision& d){assert(d.stop&&!d.applied&&d.wheel11==0&&d.wheel21==0);}
int main() {
    const auto& model=weights::compiledModel();LearnedWheelDrive drive(model);
    State s;s.fresh=true;s.pitch_deg=3;s.height_units=18;s.now_us=4000;
    stopped(drive.step(s,clockNow));assert(drive.status().mode==Mode::Off);
    assert(drive.setMode(Mode::Shadow));stopped(drive.step(s,clockNow));assert(drive.status().calls==1);
    // Arm always discards shadow/previous command history and selects Active.
    assert(drive.arm()&&drive.status().mode==Mode::Active);
    auto d=drive.step(s,clockNow);assert(d.applied&&!d.stop&&d.wheel11!=d.wheel21);
    assert(std::abs(d.wheel11)<=80&&std::abs(d.wheel21)<=80);
    assert(std::isfinite(d.leg12_m)&&std::isfinite(d.leg22_m));
    assert(std::abs(d.leg12_m)<=.02f&&std::abs(d.leg22_m)<=.02f);
    for(unsigned n=0;n<30;++n){s.now_us+=4000;d=drive.step(s,clockNow);assert(d.applied&&!d.stop);}
    assert(drive.setMode(Mode::Off));stopped(drive.step(s,clockNow));
    assert(drive.arm());s.fresh=false;stopped(drive.step(s,clockNow));
    assert(drive.status().error==Error::State&&drive.status().mode==Mode::Off);
    s.fresh=true;s.now_us+=4000;stopped(drive.step(s,clockNow));
    assert(drive.arm());assert(drive.step(s,clockNow).applied);
    s.now_us+=30001;stopped(drive.step(s,clockNow));assert(drive.status().error==Error::Gap);
    assert(drive.arm());cost=2001;stopped(drive.step(s,clockNow));assert(drive.status().error==Error::Deadline);cost=100;
    assert(drive.arm());s.pitch_deg=std::numeric_limits<float>::quiet_NaN();stopped(drive.step(s,clockNow));
    assert(drive.status().error==Error::Numeric);s.pitch_deg=0;
    for(unsigned n=0;n<4;++n) {
        assert(drive.arm());s.forward_mm_s=n==0?166:0;s.turn_deg_s=n==1?37:0;
        s.height_units=n==2?30:18;s.roll_command_deg=n==3?5:0;
        stopped(drive.step(s,clockNow));assert(drive.status().error==Error::Reference);
    }
    s.forward_mm_s=s.turn_deg_s=s.roll_command_deg=0;s.height_units=18;
    Model residual=model;residual.action_space=ActionSpace::Residual;residual.action_scale=80;residual.yaw_integral_gain=0;
    LearnedWheelDrive residualDrive(residual);assert(residualDrive.status().model_valid);
    assert(!residualDrive.arm()&&!residualDrive.setMode(Mode::Shadow));stopped(residualDrive.step(s,clockNow));
    Model wheelsOnly=model;wheelsOnly.action_space=ActionSpace::Wheels;wheelsOnly.action_scale=1023;
    LearnedWheelDrive wheelsDrive(wheelsOnly);assert(wheelsDrive.status().model_valid);
    assert(!wheelsDrive.arm()&&!wheelsDrive.setMode(Mode::Shadow));stopped(wheelsDrive.step(s,clockNow));
    Model untrained=model;untrained.trained=false;LearnedWheelDrive untrainedDrive(untrained);
    assert(!untrainedDrive.arm());stopped(untrainedDrive.step(s,clockNow));
    Model invalid=model;invalid.contract=0;LearnedWheelDrive invalidDrive(invalid);
    assert(!invalidDrive.arm());stopped(invalidDrive.step(s,clockNow));
    assert(drive.arm());assert(drive.step(s,clockNow).applied);
    std::puts("RL-only gate: trained direct actor, off/shadow, stop/rearm, stale/numeric/deadline/gap/reference faults and residual rejection passed");
}
