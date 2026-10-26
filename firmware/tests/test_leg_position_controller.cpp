#include "leg_position_controller.h"
#include "policy/policy.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
using namespace rider::firmware;
int main() {
    LegPositionController legs;MotionController previous;
    MotionInput i;i.mode=1;i.calibrated=true;i.zero_a=601;i.zero_b=409;
    MotionRequest a;a.command.max_forward=100;a.command.max_turn=20;MotionRequest b=a;
    for(unsigned n=0;n<20000;++n) {
        i.now_ms=4000+n*4;i.pitch_deg=std::sin(n*.007f)*5;i.roll_deg=std::sin(n*.009f)*3;
        i.yaw_deg=std::sin(n*.001f)*40;i.speed11=std::sin(n*.01f)*90;i.speed21=-i.speed11;
        i.encoder11=int32_t(n);i.encoder21=-int32_t(n/2);
        a.command.forward=uint8_t(n%256);a.command.turn=uint8_t(n/3);
        a.command.height=uint8_t(n%200);a.command.roll=uint8_t(110+n%35);
        a.leveling=(n/1000)%2;b=a;
        const auto actual=legs.step(i,a),reference=previous.step(i,b);
        assert(actual.valid&&reference.valid);
        assert(actual.wheels.id11==0&&actual.wheels.id21==0);
        assert(actual.command.filtered_forward==reference.command.filtered_forward);
        assert(actual.command.filtered_turn==reference.command.filtered_turn);
        assert(actual.command.filtered_height==reference.command.filtered_height);
        assert(actual.legs.roll_target_deg==reference.legs.roll_target_deg);
        assert(actual.legs.extension_a==reference.legs.extension_a&&actual.legs.extension_b==reference.legs.extension_b);
        assert(actual.legs.servo_a==reference.legs.servo_a&&actual.legs.servo_b==reference.legs.servo_b);
    }
    // Full-range phone commands and OEM waves must remain within actor limits.
    legs.reset();a={};a.roll_phase=a.height_phase=10;
    for(unsigned n=0;n<20000;++n) {
        i.now_ms+=4;a.command.forward=uint8_t(n);a.command.turn=uint8_t(n/7);
        const auto out=legs.step(i,a);assert(out.valid);
        assert(out.wheels.id11==0&&out.wheels.id21==0);
        assert(std::fabs(out.command.filtered_forward)<=rider::policy::kMaxForwardMmS);
        assert(std::fabs(out.command.filtered_turn)<=rider::policy::kMaxTurnDegS);
        assert(out.command.height<=29&&out.command.filtered_height<=29);
        assert(std::fabs(out.command.roll)<=4&&out.legs.servo_a<=1023&&out.legs.servo_b<=1023);
        assert(a.roll_phase>0&&a.height_phase>0);
    }
    a={};a.command.height=255;a.command.roll=255;a.command.forward=a.command.turn=255;
    for(unsigned n=0;n<10000;++n){i.now_ms+=4;assert(legs.step(i,a).valid);}
    auto out=legs.step(i,a);assert(out.command.height==29&&out.command.roll==4);
    assert(out.command.filtered_height<=29);
    // Wheel feedback and pitch cannot create a wheel command in this module.
    i.pitch_deg=30;i.pitch_rate_rad_s=20;i.speed11=10000;i.speed21=-10000;
    out=legs.step(i,a);assert(out.valid&&out.wheels.id11==0&&out.wheels.id21==0);
    a.command.max_forward=a.command.max_turn=0;a.command.automatic_lean=true;assert(legs.step(i,a).valid);
    i.roll_deg=std::numeric_limits<float>::quiet_NaN();assert(!legs.step(i,a).valid);i.roll_deg=0;
    a.roll_phase=std::numeric_limits<float>::infinity();assert(!legs.step(i,a).valid);a.roll_phase=0;
    i.zero_a=1024;assert(!legs.step(i,a).valid);i.zero_a=601;
    i.calibrated=false;assert(!legs.step(i,a).valid);i.calibrated=true;
    // Actor leg offsets are metric extension residuals and stay in range.
    auto base=legs.step(i,a);
    auto same=base;rider::control::offsetLegExtension(same.legs,0,0,601,409);
    assert(same.legs.servo_a==base.legs.servo_a&&same.legs.servo_b==base.legs.servo_b);
    auto up=base;rider::control::offsetLegExtension(up.legs,.003f,.003f,601,409);
    assert(up.legs.extension_a>base.legs.extension_a&&up.legs.extension_b>base.legs.extension_b);
    assert(up.legs.servo_a<base.legs.servo_a&&up.legs.servo_b>base.legs.servo_b);
    auto down=base;rider::control::offsetLegExtension(down.legs,-.003f,-.003f,601,409);
    assert(down.legs.servo_a>base.legs.servo_a&&down.legs.servo_b<base.legs.servo_b);
    auto clamped=base;rider::control::offsetLegExtension(clamped.legs,1.0f,-1.0f,601,409);
    assert(clamped.legs.extension_a==81&&clamped.legs.extension_b==45);
    assert(clamped.legs.servo_a<=1023&&clamped.legs.servo_b<=1023);
    std::puts("leg position: 20000-step parity, 30000 bounded phone/wave steps, zero wheel output, actor leg-offset bounds and invalid-state rejection passed");
}
