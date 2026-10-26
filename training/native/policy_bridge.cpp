#include "policy/policy.h"
#include POLICY_HEADER
#include <algorithm>
namespace {
uint32_t policyClock() {
    // Do not turn macOS process descheduling into a simulated MCU deadline miss.
    // Target inference deadlines are checked by firmware tests and on the MCU.
    return 0;
}
}
extern "C" {
unsigned policy_inputs(){return rider::policy::kInputs;}
void* policy_new(){return new rider::policy::Controller(rider::policy::weights::compiledModel());}
void policy_free(void* p){delete static_cast<rider::policy::Controller*>(p);}
int policy_reset(void* p){return static_cast<rider::policy::Controller*>(p)->setMode(rider::policy::Mode::Active);}
int policy_set_dead_zone(void* p,float max_units,float hysteresis,float ramp,float chatter_rate,float backoff,float chatter_max_u,float initial_units,float grow_mean_u) {
    rider::policy::DeadZoneConfig c;c.max_units=max_units;c.hysteresis=hysteresis;c.ramp_units_per_s=ramp;c.chatter_rate=chatter_rate;c.chatter_backoff=backoff;c.chatter_max_u=chatter_max_u;c.initial_units=initial_units;c.grow_mean_u=grow_mean_u;
    return static_cast<rider::policy::Controller*>(p)->setDeadZone(c)?1:0;
}
void policy_reset_dead_zone(void* p){static_cast<rider::policy::Controller*>(p)->resetDeadZoneLearning();}
void policy_tick(void* p,uint32_t us,const float* x,float* out) {
    auto& controller=*static_cast<rider::policy::Controller*>(p);rider::policy::State s;
    s.now_us=us;s.fresh=true;s.pitch_deg=x[0];s.roll_deg=x[1];s.pitch_rate_rad_s=x[2];
    s.roll_rate_deg_s=x[3];s.yaw_rate_deg_s=x[4];s.speed11_mm_s=x[5];s.speed21_mm_s=x[6];
    s.leg12=int16_t(x[7]);s.leg22=int16_t(x[8]);s.zero12=uint16_t(x[9]);s.zero22=uint16_t(x[10]);
    s.forward_mm_s=x[11];s.turn_deg_s=x[12];s.height_units=x[13];s.position_m=x[14];
    s.roll_command_deg=x[15];
    auto decision=controller.step(s,0,0,policyClock);const auto& state=controller.status();
    constexpr auto n=rider::policy::kInputs;constexpr auto m=rider::policy::kOutputs;
    std::copy(state.observation,state.observation+n,out);
    std::copy(state.action,state.action+m,out+n);
    std::copy(state.command,state.command+m,out+n+m);
    out[n+2*m]=float(decision.wheel11);out[n+2*m+1]=float(decision.wheel21);
    out[n+2*m+2]=decision.leg12_m;out[n+2*m+3]=decision.leg22_m;
    out[n+2*m+4]=float(state.calls);
    out[n+2*m+5]=float(unsigned(state.error));out[n+2*m+6]=float(unsigned(state.mode));
    out[n+2*m+7]=float(state.last_interval_us);out[n+2*m+8]=state.yaw_integral;
    out[n+2*m+9]=state.dz_preload;out[n+2*m+10]=float(state.dz_sign);out[n+2*m+11]=float(state.dz_trips);out[n+2*m+12]=state.dz_flip_rate;
}
int policy_infer(const float* raw,float* action){return rider::policy::infer(rider::policy::weights::compiledModel(),raw,action);}
}
