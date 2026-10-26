#include "policy/policy.h"
#include "policy/policy_weights.h"
#include "command_protocol.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace rider::policy;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::abort();}}while(0)
static uint32_t ticks=0,cost=100;
static uint32_t clockNow(){const auto v=ticks;ticks+=cost;return v;}
struct Fixture {
    std::array<float,18> mean{},inv{};
    std::array<float,576> w1{};
    std::array<float,32> b1{},b2{};
    std::array<float,1024> w2{};
    std::array<float,128> w3{};
    std::array<float,4> b3{};
    Model model;
    Fixture():model{"test_actor",7,6,true,ActionSpace::Residual,80,.005f,5,
        mean.data(),inv.data(),w1.data(),b1.data(),w2.data(),b2.data(),w3.data(),b3.data()} {
        inv.fill(1);b3[0]=1;b3[1]=-1;b3[2]=.5f;b3[3]=-.5f;
    }
};
int main() {
    CHECK(validateModel(weights::compiledModel()));
    Controller diagnostic(weights::compiledModel());CHECK(diagnostic.status().mode==Mode::Off);
    CHECK(!diagnostic.setMode(Mode::Active));CHECK(diagnostic.status().error==Error::Untrained);
    CHECK(diagnostic.setMode(Mode::Shadow));
    State s;s.fresh=true;s.now_us=4000;s.pitch_deg=10;s.leg12=600;s.zero12=500;
    s.roll_rate_deg_s=180;s.yaw_rate_deg_s=-90;s.speed11_mm_s=200;
    s.forward_mm_s=100;s.turn_deg_s=90;s.height_units=18;s.position_m=10;
    auto d=diagnostic.step(s,17,-29,clockNow);CHECK(!d.stop&&!d.applied&&d.wheel11==17&&d.wheel21==-29);
    CHECK(diagnostic.status().calls==1&&diagnostic.status().last_us==100);
    const auto& obs=diagnostic.status().observation;
    CHECK(std::fabs(obs[0]-.174532925f)<1e-6f);CHECK(std::fabs(obs[3]-3.14159265f)<1e-6f);
    CHECK(std::fabs(obs[4]+1.5707963f)<1e-6f);CHECK(std::fabs(obs[5]-.2f)<1e-6f);
    CHECK(obs[7]==100.0f/512);CHECK(obs[11]==.5f&&obs[12]==0);
    CHECK(obs[13]==0&&obs[14]==0&&std::fabs(obs[15]-.003f)<1e-7f);
    CHECK(std::fabs(obs[16]-.004f)<1e-7f&&obs[17]==0);
    // Contract 6 runs the actor on every control tick (>= kPeriodUs apart); shorter gaps hold the action.
    s.now_us+=4000;diagnostic.step(s,0,0,clockNow);CHECK(diagnostic.status().calls==2);
    CHECK(diagnostic.status().last_interval_us==4000);
    CHECK(std::fabs(diagnostic.status().observation[12]+.0004f)<1e-6f);
    s.now_us+=2000;diagnostic.step(s,0,0,clockNow);CHECK(diagnostic.status().calls==2);
    s.now_us+=2000;diagnostic.step(s,0,0,clockNow);CHECK(diagnostic.status().calls==3);
    CHECK(diagnostic.status().last_interval_us==4000);
    CHECK(std::fabs(diagnostic.status().observation[12]+.0008f)<1e-6f);
    s.now_us+=12000;s.position_m+=.05f;diagnostic.step(s,0,0,clockNow);
    CHECK(diagnostic.status().last_interval_us==12000);
    CHECK(std::fabs(diagnostic.status().observation[12]-(.05f-.0020f))<1e-6f);
    diagnostic.resetHistory();s.now_us+=12000;diagnostic.step(s,0,0,clockNow);
    CHECK(diagnostic.status().observation[12]==0);
    s.turn_deg_s=0;
    Fixture f;Controller residual(f.model);CHECK(residual.setMode(Mode::Active));
    s.now_us=100000;d=residual.step(s,1000,-1000,clockNow);
    CHECK(d.applied&&!d.stop&&d.wheel11==1023&&d.wheel21==-1023);
    CHECK(residual.status().command[0]==80&&residual.status().command[1]==-80);
    s.now_us+=4000;d=residual.step(s,-10,20,clockNow);CHECK(d.wheel11==70&&d.wheel21==-60);
    const auto calls=residual.status().calls;CHECK(residual.setMode(Mode::Off));
    s.fresh=false;d=residual.step(s,37,-41,nullptr);CHECK(!d.applied&&!d.stop&&d.wheel11==37&&d.wheel21==-41);
    CHECK(residual.status().calls==calls);s.fresh=true;
    Fixture direct;direct.model.action_space=ActionSpace::Wheels;direct.model.action_scale=1023;
    Controller wheels(direct.model);CHECK(wheels.setMode(Mode::Active));s.now_us=0;
    d=wheels.step(s,777,-888,clockNow);CHECK(d.wheel11==80&&d.wheel21==-80);
    for(unsigned i=0;i<15;++i){s.now_us+=4000;d=wheels.step(s,777,-888,clockNow);}
    CHECK(d.wheel11==1023&&d.wheel21==-1023);CHECK(!d.stop);
    wheels.resetHistory();s.now_us+=4000;d=wheels.step(s,0,0,clockNow);CHECK(d.wheel11==80);
    s.now_us+=30001;d=wheels.step(s,777,-888,clockNow);
    CHECK(d.stop&&!d.applied&&wheels.status().mode==Mode::Off&&wheels.status().error==Error::Gap);
    CHECK(wheels.status().action[0]==0&&wheels.status().command[0]==0);
    CHECK(wheels.setMode(Mode::Active));cost=2001;s.now_us+=4000;d=wheels.step(s,0,0,clockNow);
    CHECK(d.stop&&wheels.status().error==Error::Deadline&&wheels.status().last_us==2001);cost=100;
    CHECK(wheels.setMode(Mode::Active));s.fresh=false;d=wheels.step(s,0,0,clockNow);
    CHECK(d.stop&&wheels.status().error==Error::State);s.fresh=true;
    CHECK(wheels.setMode(Mode::Active));s.pitch_deg=std::numeric_limits<float>::quiet_NaN();
    d=wheels.step(s,0,0,clockNow);CHECK(d.stop&&wheels.status().error==Error::Numeric);s.pitch_deg=0;
    CHECK(wheels.setMode(Mode::Shadow));cost=2001;d=wheels.step(s,71,-92,clockNow);
    CHECK(!d.stop&&!d.applied&&d.wheel11==71&&d.wheel21==-92&&wheels.status().mode==Mode::Off);cost=100;
    CHECK(wheels.setMode(Mode::Active));s.now_us=UINT32_MAX-2000;wheels.step(s,0,0,clockNow);
    s.now_us+=12000;d=wheels.step(s,0,0,clockNow);CHECK(!d.stop&&wheels.status().last_interval_us==12000);
    s.now_us+=4000;s.yaw_rate_deg_s=std::numeric_limits<float>::quiet_NaN();
    d=wheels.step(s,0,0,clockNow);CHECK(d.stop&&wheels.status().error==Error::Numeric);s.yaw_rate_deg_s=0;
    for(unsigned limit=0;limit<4;++limit) {
        CHECK(wheels.setMode(Mode::Active));s.now_us+=4000;
        s.forward_mm_s=limit==0?166:0;s.turn_deg_s=limit==1?37:0;
        s.height_units=limit==2?30:18;s.roll_command_deg=limit==3?5:0;
        d=wheels.step(s,0,0,clockNow);CHECK(d.stop&&wheels.status().error==Error::Reference);
    }
    s.forward_mm_s=s.turn_deg_s=s.roll_command_deg=0;s.height_units=18;
    Fixture pi;pi.model.action_space=ActionSpace::Wheels;pi.model.action_scale=1023;
    pi.model.yaw_integral_gain=.05f;pi.model.yaw_integral_limit=.01f;pi.b3.fill(0);
    Controller yaw(pi.model);CHECK(yaw.setMode(Mode::Active));State ys;ys.fresh=true;ys.turn_deg_s=30;
    for(unsigned i=0;i<100;++i){ys.now_us=i*6000;d=yaw.step(ys,0,0,clockNow);}
    CHECK(!d.stop&&std::fabs(yaw.status().yaw_integral-.01f)<1e-7f);
    CHECK(d.wheel11==-10&&d.wheel21==-10&&yaw.status().action[0]==0);
    ys.yaw_rate_deg_s=30;ys.now_us+=6000;yaw.step(ys,0,0,clockNow);
    CHECK(std::fabs(yaw.status().yaw_integral-.01f)<1e-7f);
    CHECK(std::fabs(yaw.status().observation[13]-float(-10)/1023)<1e-7f);
    CHECK(std::fabs(yaw.status().observation[14]-float(-10)/1023)<1e-7f);
    CHECK(std::fabs(yaw.status().observation[16]-.006f)<1e-7f);
    CHECK(yaw.status().observation[17]==0);   // input 17 is the position integral (zero position error here)
    yaw.resetHistory();CHECK(yaw.status().yaw_integral==0);
    pi.model.yaw_integral_gain=.21f;CHECK(!validateModel(pi.model));pi.model.yaw_integral_gain=.05f;
    pi.model.action_space=ActionSpace::Residual;CHECK(!validateModel(pi.model));
    f.inv[0]=0;CHECK(!validateModel(f.model));f.inv[0]=1;
    f.model.action_scale=129;CHECK(!validateModel(f.model));f.model.action_scale=80;
    f.w1[0]=std::numeric_limits<float>::infinity();CHECK(!validateModel(f.model));f.w1[0]=0;
    f.model.contract=5;CHECK(!validateModel(f.model));f.model.contract=6;
    f.model.name="invalid\"json";CHECK(!validateModel(f.model));f.model.name="test_actor";
    // Model normalization overflow must fail rather than silently clip Infinity.
    f.mean[0]=-std::numeric_limits<float>::max();f.inv[0]=100;Controller overflow(f.model);
    CHECK(overflow.setMode(Mode::Active));s.pitch_deg=std::numeric_limits<float>::max();
    // Use a representable angle whose normalization overflows through inverse std.
    s.pitch_deg=1;d=overflow.step(s,0,0,clockNow);CHECK(d.stop);
    // Inputs 5/6: first-order low-pass (tau kSpeedFilterTauS) initialised to the first sample after a reset.
    Fixture lp;lp.model.action_space=ActionSpace::Wheels;lp.model.action_scale=1023;lp.b3.fill(0);
    Controller filt(lp.model);CHECK(filt.setMode(Mode::Active));State fs;fs.fresh=true;fs.speed11_mm_s=100;fs.speed21_mm_s=-50;
    fs.now_us=1000000;filt.step(fs,0,0,clockNow);
    CHECK(std::fabs(filt.status().observation[5]-.1f)<1e-7f&&std::fabs(filt.status().observation[6]+.05f)<1e-7f);
    fs.speed11_mm_s=0;fs.speed21_mm_s=0;
    for(unsigned i=0;i<20;++i){fs.now_us+=6000;filt.step(fs,0,0,clockNow);}
    {const float a=.006f/(kSpeedFilterTauS+.006f);float expected=.1f;for(unsigned i=0;i<20;++i)expected*=(1-a);
     CHECK(std::fabs(filt.status().observation[5]-expected)<2e-6f);}
    filt.resetHistory();fs.speed11_mm_s=200;fs.now_us+=6000;filt.step(fs,0,0,clockNow);
    CHECK(std::fabs(filt.status().observation[5]-.2f)<1e-7f);
    // Input 17: position-error integral, sampled before the tick's update, clipped, cleared on reset.
    Fixture integ;integ.model.action_space=ActionSpace::Wheels;integ.model.action_scale=1023;integ.b3.fill(0);
    Controller pos(integ.model);CHECK(pos.setMode(Mode::Active));State ps;ps.fresh=true;ps.position_m=0;
    ps.now_us=1000000;pos.step(ps,0,0,clockNow);CHECK(pos.status().observation[17]==0);
    ps.position_m=.1f;
    for(unsigned i=0;i<100;++i){ps.now_us+=6000;pos.step(ps,0,0,clockNow);}
    // 100 ticks of 6 ms at a 0.1 m error: 0.06 m*s (observed one tick late, so 99 ticks).
    CHECK(std::fabs(pos.status().observation[17]-.1f*.006f*99)<1e-5f);
    CHECK(std::fabs(pos.status().pos_integral-.1f*.006f*100)<1e-5f);
    ps.position_m=1000.f;for(unsigned i=0;i<10;++i){ps.now_us+=6000;pos.step(ps,0,0,clockNow);}
    CHECK(std::fabs(pos.status().pos_integral-kPosIntegralLimit)<1e-6f);
    pos.resetHistory();CHECK(pos.status().pos_integral==0);
    Fixture wheelLegs;wheelLegs.model.action_space=ActionSpace::WheelsLegs;
    wheelLegs.model.action_scale=1023;wheelLegs.model.leg_action_scale=.005f;
    Controller legActor(wheelLegs.model);CHECK(legActor.setMode(Mode::Active));
    s.now_us=500000;s.forward_mm_s=s.turn_deg_s=s.roll_command_deg=0;s.height_units=18;
    d=legActor.step(s,0,0,clockNow);
    CHECK(d.applied&&!d.stop&&std::fabs(d.leg12_m-.0002f)<1e-7f&&std::fabs(d.leg22_m+.0002f)<1e-7f);
    CHECK(std::fabs(legActor.status().command[2]-.04f)<1e-7f&&std::fabs(legActor.status().command[3]+.04f)<1e-7f);
    CHECK(std::fabs(legActor.status().command[0]-80.0f)<1e-7f);
    for(unsigned i=0;i<20;++i){s.now_us+=4000;d=legActor.step(s,0,0,clockNow);}
    CHECK(std::fabs(d.leg12_m-.0025f)<1e-6f&&std::fabs(d.leg22_m+.0025f)<1e-6f);
    // The leg action profile must stay in its declared metric range.
    wheelLegs.model.leg_action_scale=.03f;CHECK(!validateModel(wheelLegs.model));
    wheelLegs.model.leg_action_scale=.005f;
    // Dead-zone preload on the common-mode wheel command (dead_zone_comp.h): off by default, validated, learns only while
    // the robot stands quietly, goes in with opposite signs on the mirrored wheels, follows the push direction.
    {
        Fixture dzf;dzf.model.action_space=ActionSpace::Wheels;dzf.model.action_scale=100;      // network: wheel 11 +100, wheel 21 -100
        Controller dz(dzf.model);CHECK(dz.setMode(Mode::Active));
        State q;q.fresh=true;q.now_us=1000000;
        for(unsigned i=0;i<60;++i){q.now_us+=5000;d=dz.step(q,0,0,clockNow);}
        CHECK(d.wheel11==100&&d.wheel21==-100&&dz.status().dz_preload==0&&dz.deadZoneConfig().max_units==0);
        DeadZoneConfig bad;bad.max_units=-1;CHECK(!dz.setDeadZone(bad));
        bad=DeadZoneConfig();bad.max_units=kMaxDeadZoneUnits+1;CHECK(!dz.setDeadZone(bad));
        bad=DeadZoneConfig();bad.max_units=10;bad.hysteresis=0;CHECK(!dz.setDeadZone(bad));
        bad=DeadZoneConfig();bad.max_units=10;bad.chatter_backoff=1.0f;CHECK(!dz.setDeadZone(bad));
        bad=DeadZoneConfig();bad.max_units=10;bad.ramp_units_per_s=-1;CHECK(!dz.setDeadZone(bad));
        bad=DeadZoneConfig();bad.max_units=10;bad.chatter_max_u=5;CHECK(!dz.setDeadZone(bad));
        bad=DeadZoneConfig();bad.max_units=10;bad.chatter_max_u=500;CHECK(!dz.setDeadZone(bad));
        bad=DeadZoneConfig();bad.max_units=std::numeric_limits<float>::quiet_NaN();CHECK(!dz.setDeadZone(bad));
        CHECK(dz.deadZoneConfig().max_units==0);
        DeadZoneConfig on;on.max_units=40;on.ramp_units_per_s=20;CHECK(dz.setDeadZone(on));
        for(unsigned i=0;i<600;++i){q.now_us+=5000;d=dz.step(q,0,0,clockNow);}      // 3 s at 20 units/s: capped at 40
        CHECK(dz.status().dz_preload==40&&dz.status().dz_sign==1&&d.wheel11==140&&d.wheel21==-140);
        // what was learned survives a history reset (stop, lift guard); the relay starts undecided and decides again at once
        dz.resetHistory();CHECK(dz.status().dz_sign==0);
        for(unsigned i=0;i<3;++i){q.now_us+=5000;d=dz.step(q,0,0,clockNow);}           // the slew limiter refills the command first
        CHECK(dz.status().dz_preload==40&&d.wheel11==140&&d.wheel21==-140);
        dz.resetDeadZoneLearning();CHECK(dz.status().dz_preload==0&&dz.status().dz_trips==0);
        q.now_us+=5000;d=dz.step(q,0,0,clockNow);CHECK(d.wheel11==100&&d.wheel21==-100);      // learning restarts from zero
        // a starting value is in from the first tick (the wheels are mirrored), a reset goes back to it, an out-of-range one is rejected
        {
            Controller start(dzf.model);CHECK(start.setMode(Mode::Active));
            DeadZoneConfig begin;begin.max_units=40;begin.initial_units=12;begin.ramp_units_per_s=0;CHECK(start.setDeadZone(begin));
            CHECK(start.status().dz_preload==12);
            State sq;sq.fresh=true;sq.now_us=1000000;
            for(unsigned i=0;i<20;++i){sq.now_us+=5000;d=start.step(sq,0,0,clockNow);}
            CHECK(d.wheel11==112&&d.wheel21==-112&&start.status().dz_preload==12);
            start.resetDeadZoneLearning();CHECK(start.status().dz_preload==12&&start.status().dz_trips==0);
            DeadZoneConfig off=begin;off.max_units=0;CHECK(start.setDeadZone(off));                  // off: initial above max is clamped, not rejected
            DeadZoneConfig tooBig=begin;tooBig.initial_units=kMaxDeadZoneUnits+1;CHECK(!start.setDeadZone(tooBig));
            DeadZoneConfig badGrow=begin;badGrow.grow_mean_u=-1;CHECK(!start.setDeadZone(badGrow));
            badGrow.grow_mean_u=61;CHECK(!start.setDeadZone(badGrow));
        }
        // a robot that is being driven does not learn
        Controller driven(dzf.model);CHECK(driven.setMode(Mode::Active));CHECK(driven.setDeadZone(on));
        State dq;dq.fresh=true;dq.now_us=1000000;dq.forward_mm_s=50;
        for(unsigned i=0;i<400;++i){dq.now_us+=5000;d=driven.step(dq,0,0,clockNow);}
        CHECK(driven.status().dz_preload==0&&d.wheel11==100);
        // nor one that is moving (a wheel that rolls is not blocked by the dead zone)
        dq.forward_mm_s=0;dq.speed11_mm_s=dq.speed21_mm_s=200;
        for(unsigned i=0;i<400;++i){dq.now_us+=5000;d=driven.step(dq,0,0,clockNow);}
        CHECK(driven.status().dz_preload<0.5f);                                     // (the speed filter needs ~2 ticks to rise past the gate)
        // a push in the other direction gives the opposite offset
        Fixture neg;neg.model.action_space=ActionSpace::Wheels;neg.model.action_scale=100;neg.b3[0]=-1;neg.b3[1]=1;
        Controller back(neg.model);CHECK(back.setMode(Mode::Active));CHECK(back.setDeadZone(on));
        State bq;bq.fresh=true;bq.now_us=1000000;
        for(unsigned i=0;i<600;++i){bq.now_us+=5000;d=back.step(bq,0,0,clockNow);}
        CHECK(back.status().dz_sign==-1&&d.wheel11==-140&&d.wheel21==140);
        // shadow mode computes but never applies; the preload is neither learned nor added there
        Controller shadow(dzf.model);CHECK(shadow.setMode(Mode::Active));CHECK(shadow.setDeadZone(on));CHECK(shadow.setMode(Mode::Shadow));
        for(unsigned i=0;i<400;++i){q.now_us+=5000;d=shadow.step(q,7,-9,clockNow);}
        CHECK(!d.applied&&d.wheel11==7&&d.wheel21==-9&&shadow.status().dz_preload==0);
    }
    // New binary API is bounded and does not change the legacy stream packet.
    using namespace rider2::protocol;
    CommandEvent e;e.reg=0xb0;e.length=1;e.data[0]=2;CHECK(validCommand(e));
    CHECK(commandKind(e)==CommandKind::PolicyMode);e.data[0]=3;CHECK(!validCommand(e));
    e.data[0]=0;e.type=2;Telemetry t;t.policy_mode=1;t.policy_model_id=0x12345678;
    t.policy_valid=true;t.policy_trained=true;t.policy_action_space=2;uint8_t bytes[64];
    CHECK(buildReadResponse(e,t,bytes,sizeof bytes)==9&&bytes[5]==1);
    e.reg=0xb1;CHECK(buildReadResponse(e,t,bytes,sizeof bytes)==16);
    CHECK(bytes[5]==0x78&&bytes[8]==0x12&&bytes[9]==2&&bytes[10]==1&&bytes[11]==1&&bytes[12]==6);
    CHECK(buildStreamTelemetry(t,bytes,sizeof bytes)==45);
    std::printf("native policy: %u checks passed\n",checks);
}
