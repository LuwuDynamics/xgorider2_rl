#include "rider_motion.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace rider { namespace control {
namespace {
constexpr double pi=3.14159265358979323846;
std::int16_t wrap16(std::int32_t x) {
    const auto u=static_cast<std::uint16_t>(x);
    return static_cast<std::int16_t>(u<0x8000u?static_cast<std::int32_t>(u):static_cast<std::int32_t>(u)-65536);
}
// Finite, representable inputs follow trunc.s followed by sext a?,a?,15.
// Exceptional values use an explicit host fallback; those paths have not been
// verified on ESP32 hardware and are outside the differential-test claim.
std::int16_t trunc16(float x) {
    if(std::isnan(x)||x<=-2147483648.0f)return 0;
    if(x>=2147483648.0f)return -1;
    return wrap16(static_cast<std::int32_t>(x));
}
float decode(std::uint8_t x,float range) {
    return applyDeadband(mapByte(x,0,255,-range,range),
                         static_cast<float>(static_cast<double>(range)*.05),0);
}
}

void updateCommandTargets(CommandState &s,const MotionCommand &c) {
    s.forward=decode(c.forward,c.max_forward);
    s.turn=decode(c.turn,c.max_turn);
    // 0x400d4248: each wave period is 2000..4000 phase units.
    const float roll_period=std::fma(static_cast<float>(c.roll_period)/255.0f,2000.0f,2000.0f);
    const float height_period=std::fma(static_cast<float>(c.height_period)/255.0f,2000.0f,2000.0f);
    if(c.automatic_lean) {
        if(static_cast<double>(std::fabs(s.filtered_forward))>static_cast<double>(c.max_forward)*.1) {
            const float ratio=static_cast<float>(static_cast<double>(std::fabs(s.filtered_forward))*.6/c.max_forward+.4);
            const float turn_ratio=-s.filtered_turn/c.max_turn;
            s.roll=static_cast<float>(static_cast<double>(ratio)*20.0*turn_ratio);
        }
    } else if(s.roll_phase!=0) {
        const float phase=s.roll_phase/roll_period;
        s.roll=static_cast<float>(std::sin(static_cast<double>(phase+phase)*pi)*15.0);
        s.roll_phase+=10;
        if(s.roll_phase>roll_period)s.roll_phase=10;
    } else s.roll=mapByte(c.roll,0,255,-20,20);
    if(s.height_phase!=0) {
        const float phase=s.height_phase/height_period;
        s.height=static_cast<float>((std::sin(static_cast<double>(phase+phase)*pi)*26.0)*.5+18.0);
        s.height_phase+=10;
        if(s.height_phase>height_period)s.height_phase=10;
    } else s.height=mapByte(c.height,0,255,0,36);
    const float forward=std::fma(s.filtered_forward,.949999988079071f,s.forward*.05000001192092896f);
    const float delta=forward-s.filtered_forward;
    s.filtered_forward=std::fabs(delta)<5?forward:s.filtered_forward+(forward<s.filtered_forward?-5.0f:5.0f);
    s.filtered_turn=std::fma(s.filtered_turn,.949999988079071f,s.turn*.05000001192092896f);
    s.filtered_height=static_cast<float>(static_cast<double>(s.filtered_height)*.97+static_cast<double>(s.height)*.03);
}

void updatePositionHold(HoldState &s,std::uint32_t now,std::uint8_t mode,
                        float forward,std::uint8_t turn,float position,
                        float pitch,bool protection) {
    if(std::uint32_t(now-s.last_disabled_ms)<750)s.correction=0;
    if(mode!=1){s.correction=0;return;} // firmware leaves hold mode untouched here
    if(std::fabs(forward)<1 && now>2000 && turn>=113 && turn<=143) {
        if(s.mode==0){s.mode=1;s.began_ms=now;}
    } else s.mode=0;
    if(std::uint32_t(now-s.began_ms)>500 && s.mode==1)s.mode=2;
    if(s.mode==2)s.correction=(s.position-position)*1350.0f;
    else {s.position=position;s.correction=0;}
    if(protection && (static_cast<double>(std::fabs(s.position-position))>.2 || std::fabs(pitch)>20) && s.mode==2)
        s.mode=0; // correction is intentionally retained until next call
}

float wheelDistance(std::int32_t first,std::int32_t second) {
    const std::uint32_t sum=static_cast<std::uint32_t>(first)+static_cast<std::uint32_t>(second);
    const double signed_sum=sum<0x80000000u?static_cast<double>(sum):static_cast<double>(sum)-4294967296.0;
    return static_cast<float>((((signed_sum*.5)*.0009765625)*3.1415)*.055);
}

float updateBalance(BalanceState &s,float target,float speed11,float speed21,
                    float pitch,float bias,float pitch_rate,float height,
                    bool holding,Wheels &w) {
    s.speed.kp=holding?.140000001f:.109999999f;
    s.speed.ki=holding?.000736842107f:.000733333349f;
    s.pitch.kp=37.5f;s.pitch.kd=0;
    const float height_ratio=height/36.0f;
    const float rate_gain=static_cast<float>((static_cast<double>(height_ratio)+.45)*100.0);
    s.speed.target=target;
    s.speed.measurement=(speed11+speed21)*.5f;
    updatePid(s.speed);
    s.pitch.target=s.speed.output;
    s.pitch.measurement=pitch+bias;
    updatePid(s.pitch);
    const float output=std::fma(pitch_rate,rate_gain,s.pitch.output);
    w.id21=trunc16(output);w.id11=wrap16(-static_cast<std::int32_t>(w.id21));
    return output;
}

void addWheelSpeedCorrection(Wheels &w,float forward,float turn,float speed11,float speed21) {
    if(std::fabs(forward)<10 && std::fabs(turn)<5)return;
    const double turn_speed=(((static_cast<double>(turn)/180.0)*pi)*.04)/.003583;
    const float error21=-(static_cast<float>(static_cast<double>(forward)+turn_speed)-speed21);
    const float error11=static_cast<float>(static_cast<double>(forward)-turn_speed)-speed11;
    const int delta11=std::max(-120,std::min(120,static_cast<int>(trunc16(error11+error11))));
    const int delta21=std::max(-120,std::min(120,static_cast<int>(trunc16(error21+error21))));
    if(std::fabs(forward)>=10){w.id11=wrap16(w.id11+delta11);w.id21=wrap16(w.id21+delta21);}
}

void addHeadingCorrection(HeadingState &s,Wheels &w,std::uint32_t now,
                          std::uint8_t mode,bool hold_heading,float forward,
                          float turn,float filtered_turn,float yaw,
                          bool leveling,float roll) {
    if(std::fabs(forward)>=10 || std::fabs(turn)>=1)s.last_motion_ms=now;
    else {
        const auto elapsed=std::uint32_t(now-s.last_motion_ms);
        if((elapsed<600 && elapsed>200)||mode!=1)s.target=yaw;
    }
    if(!(std::fabs(forward)>10 || std::fabs(turn)>1 || (hold_heading && mode==1)))return;
    s.target=static_cast<float>(static_cast<double>(filtered_turn)*.005+static_cast<double>(s.target));
    s.pid.target=s.target;s.pid.measurement=yaw;updatePid(s.pid);
    const float demand=static_cast<float>(static_cast<double>(-s.pid.output)-static_cast<double>(turn)*.5);
    const int offset=demand<0?1023:-1023;
    const float room=static_cast<float>(std::min(std::abs(static_cast<int>(w.id11)+offset),std::abs(static_cast<int>(w.id21)+offset)));
    const float magnitude=std::min(room,std::fabs(demand));
    const float correction=demand<0?-magnitude:magnitude;
    const auto delta=trunc16(correction);
    w.id11=wrap16(w.id11+delta);w.id21=wrap16(w.id21+delta);
    if(leveling) {
        const int extra=static_cast<int>(std::fabs(static_cast<double>(roll)/20.0)*correction);
        if(roll>0)w.id11=wrap16(w.id11+extra);else w.id21=wrap16(w.id21+extra);
    }
}

void addWheelFriction(Wheels &w) {
    for(auto *value:{&w.id11,&w.id21}){
        if(*value>10)*value=wrap16(*value+15);
        if(*value<-10)*value=wrap16(*value-15);
    }
}
}} // namespace rider::control
