#include "policy.h"
#include <algorithm>
#include <cmath>

namespace rider { namespace policy {
namespace {
constexpr float kDegToRad=0.017453292519943295f;
bool finiteArray(const float* p,unsigned n) {
    if(!p)return false;
    for(unsigned i=0;i<n;++i)if(!std::isfinite(p[i]))return false;
    return true;
}
float clip(float value,float bound) {return std::max(-bound,std::min(bound,value));}
bool layer(const float* in,unsigned ni,const float* w,const float* b,
           float* out,unsigned no,bool activate) {
    for(unsigned row=0;row<no;++row) {
        float sum=b[row];
        for(unsigned col=0;col<ni;++col)sum+=w[row*ni+col]*in[col];
        if(!std::isfinite(sum))return false;
        out[row]=activate?std::tanh(sum):clip(sum,1);
    }
    return true;
}
bool validName(const char* name) {
    if(!name||!*name)return false;
    for(unsigned i=0;i<=48;++i) {
        const char c=name[i];
        if(!c)return i<=48;
        if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'||c=='.'))return false;
    }
    return false;
}
}
bool validateModel(const Model& m) {
    if(!validName(m.name)||m.contract!=kContractVersion||
       (m.action_space!=ActionSpace::Residual&&m.action_space!=ActionSpace::Wheels&&
        m.action_space!=ActionSpace::WheelsLegs)||
       !std::isfinite(m.action_scale)||m.action_scale<=0||
       m.action_scale>(m.action_space==ActionSpace::Residual?kMaxResidual:kMaxCommand)||
       !std::isfinite(m.leg_action_scale)||m.leg_action_scale<=0||m.leg_action_scale>.02f||
       !std::isfinite(m.observation_clip)||m.observation_clip<=0||m.observation_clip>100)return false;
    if(!std::isfinite(m.yaw_integral_gain)||m.yaw_integral_gain<0||m.yaw_integral_gain>.2f||
       !std::isfinite(m.yaw_integral_limit)||m.yaw_integral_limit<=0||m.yaw_integral_limit>.1f||
       (m.yaw_integral_gain>0&&m.action_space==ActionSpace::Residual))return false;
    if(!finiteArray(m.mean,kInputs)||!finiteArray(m.inv_std,kInputs)||
       !finiteArray(m.w1,kInputs*kHidden)||!finiteArray(m.b1,kHidden)||
       !finiteArray(m.w2,kHidden*kHidden)||!finiteArray(m.b2,kHidden)||
       !finiteArray(m.w3,kHidden*kOutputs)||!finiteArray(m.b3,kOutputs))return false;
    for(unsigned i=0;i<kInputs;++i)if(m.inv_std[i]<=0)return false;
    return true;
}
// Keep the exact kernel visible in target ELF for profiling and provenance checks.
#if defined(__GNUC__)
__attribute__((noinline))
#endif
bool infer(const Model& m,const float raw[kInputs],float action[kOutputs]) {
    if(!raw||!action)return false;
    float normalized[kInputs],a[kHidden],b[kHidden],result[kOutputs];
    for(unsigned i=0;i<kInputs;++i) {
        const float value=(raw[i]-m.mean[i])*m.inv_std[i];
        if(!std::isfinite(raw[i])||!std::isfinite(value))return false;
        normalized[i]=clip(value,m.observation_clip);
    }
    if(!layer(normalized,kInputs,m.w1,m.b1,a,kHidden,true)||
       !layer(a,kHidden,m.w2,m.b2,b,kHidden,true)||
       !layer(b,kHidden,m.w3,m.b3,result,kOutputs,false))return false;
    std::copy(result,result+kOutputs,action);
    return true;
}
Controller::Controller(const Model& model):model_(model) {
    status_.model_valid=validateModel(model);status_.trained=model.trained;
    if(!status_.model_valid)status_.error=Error::Model;
}
bool Controller::setMode(Mode mode) {
    if(mode!=Mode::Off&&mode!=Mode::Shadow&&mode!=Mode::Active)return false;
    if(mode!=Mode::Off&&!status_.model_valid){status_.error=Error::Model;return false;}
    if(mode==Mode::Active&&!model_.trained){status_.error=Error::Untrained;return false;}
    status_.mode=mode;status_.error=Error::None;resetHistory();return true;
}
void Controller::resetHistory() {
    ran_=ticked_=false;last_run_us_=last_tick_us_=0;
    position_origin_=position_reference_=0;
    speed_filtered_[0]=speed_filtered_[1]=0;speed_filter_ready_=false;
    status_.yaw_integral=0;status_.pos_integral=0;
    resetDeadZoneRelay(dz_state_);status_.dz_sign=0;status_.dz_flip_rate=0;
    std::fill(status_.action,status_.action+kOutputs,0);
    std::fill(status_.command,status_.command+kOutputs,0);
    std::fill(status_.observation,status_.observation+kInputs,0);
}
bool Controller::setDeadZone(const DeadZoneConfig& config) {
    if(!std::isfinite(config.max_units)||config.max_units<0||config.max_units>kMaxDeadZoneUnits||
       !std::isfinite(config.hysteresis)||config.hysteresis<2||config.hysteresis>40||
       !std::isfinite(config.ramp_units_per_s)||config.ramp_units_per_s<0||config.ramp_units_per_s>20||
       !std::isfinite(config.chatter_rate)||config.chatter_rate<2||config.chatter_rate>40||
       !std::isfinite(config.chatter_backoff)||config.chatter_backoff<.1f||config.chatter_backoff>.95f||
       !std::isfinite(config.chatter_max_u)||config.chatter_max_u<10||config.chatter_max_u>200||
       !std::isfinite(config.initial_units)||config.initial_units<0||config.initial_units>kMaxDeadZoneUnits||
       !std::isfinite(config.grow_mean_u)||config.grow_mean_u<0||config.grow_mean_u>60)return false;
    dz_config_=config;
    // nothing learned yet: the preload is at its starting value from the first tick
    if(dz_state_.preload==0&&dz_state_.trips==0&&!dz_state_.locked)startDeadZone(dz_state_,dz_config_);
    if(dz_state_.preload>dz_config_.max_units)dz_state_.preload=dz_config_.max_units;
    status_.dz_preload=dz_state_.preload;
    return true;
}
void Controller::resetDeadZoneLearning() {
    startDeadZone(dz_state_,dz_config_);
    status_.dz_preload=dz_state_.preload;status_.dz_sign=0;status_.dz_flip_rate=0;status_.dz_locked=false;status_.dz_trips=0;
}
Decision Controller::fail(Error error,int b11,int b21) {
    const bool active=status_.mode==Mode::Active;
    ++status_.failures;status_.error=error;status_.mode=Mode::Off;resetHistory();
    Decision d;d.wheel11=b11;d.wheel21=b21;d.stop=active;return d;
}
Decision Controller::step(const State& s,int b11,int b21,Clock clock) {
    Decision d;d.wheel11=b11;d.wheel21=b21;
    if(status_.mode==Mode::Off)return d;
    if(!status_.model_valid)return fail(Error::Model,b11,b21);
    if(!s.fresh)return fail(Error::State,b11,b21);
    const uint32_t gap=ran_?uint32_t(s.now_us-last_run_us_):kPeriodUs;
    if(ran_&&gap>kMaxGapUs)return fail(Error::Gap,b11,b21);
    const uint32_t tick_us=ticked_?uint32_t(s.now_us-last_tick_us_):4000;
    if(!ticked_)position_origin_=s.position_m;
    else position_reference_+=s.forward_mm_s*.001f*float(tick_us)*.000001f;
    // First-order low-pass on the wheel speeds, initialised to the first sample after any reset.
    if(!speed_filter_ready_) {
        speed_filtered_[0]=s.speed11_mm_s*.001f;speed_filtered_[1]=s.speed21_mm_s*.001f;speed_filter_ready_=true;
    } else {
        const float dt_s=float(std::min(tick_us,kMaxGapUs))*.000001f,alpha=dt_s/(kSpeedFilterTauS+dt_s);
        speed_filtered_[0]+=alpha*(s.speed11_mm_s*.001f-speed_filtered_[0]);
        speed_filtered_[1]+=alpha*(s.speed21_mm_s*.001f-speed_filtered_[1]);
    }
    float raw[kInputs]={s.pitch_deg*kDegToRad,s.roll_deg*kDegToRad,s.pitch_rate_rad_s,
        s.roll_rate_deg_s*kDegToRad,s.yaw_rate_deg_s*kDegToRad,
        speed_filtered_[0],speed_filtered_[1],
        float(int(s.leg12)-int(s.zero12))/512.0f,float(int(s.leg22)-int(s.zero22))/512.0f,
        s.forward_mm_s*.001f,s.turn_deg_s*kDegToRad,s.height_units/36.0f,
        s.position_m-position_origin_-position_reference_,float(std::lround(status_.command[0]))/kMaxCommand,
        float(std::lround(status_.command[1]))/kMaxCommand,float(gap)*.000001f,
        float(tick_us)*.000001f,status_.pos_integral};
    // Check even on a held-action tick; never wait for the next network update.
    if(!finiteArray(raw,kInputs))return fail(Error::Numeric,b11,b21);
    if(!std::isfinite(s.roll_command_deg))return fail(Error::Numeric,b11,b21);
    if(status_.mode==Mode::Active&&model_.action_space!=ActionSpace::Residual&&
       (std::fabs(s.forward_mm_s)>kMaxForwardMmS||std::fabs(s.turn_deg_s)>kMaxTurnDegS||
        s.height_units<0||s.height_units>kMaxHeightUnits||std::fabs(s.roll_command_deg)>kMaxRollCommandDeg))
        return fail(Error::Reference,b11,b21);
    if(!ran_||gap>=kPeriodUs) {
        if(!clock)return fail(Error::Deadline,b11,b21);
        const uint32_t began=clock();
        float action[kOutputs];const bool ok=infer(model_,raw,action);
        const uint32_t elapsed=uint32_t(clock()-began);
        ++status_.calls;status_.last_us=elapsed;status_.max_us=std::max(status_.max_us,elapsed);
        status_.last_interval_us=gap;
        if(!ok)return fail(Error::Numeric,b11,b21);
        if(elapsed>kDeadlineUs)return fail(Error::Deadline,b11,b21);
        std::copy(raw,raw+kInputs,status_.observation);
        std::copy(action,action+kOutputs,status_.action);
        last_run_us_=s.now_us;ran_=true;
    }
    // Apply rate limiting at every control tick, including held policy actions.
    // Outputs 0/1 are wheel command units; outputs 2/3 are normalized leg
    // extension offsets whose physical scale is model_.leg_action_scale.
    const float dt=float(std::min(tick_us,kMaxGapUs))*.000001f;
    // Input 17 was sampled above, before this tick's update.
    status_.pos_integral=clip(status_.pos_integral+raw[12]*dt,kPosIntegralLimit);
    const float delta=slew_per_second_*dt;
    const float leg_delta=kLegSlewPerSecond*dt;
    status_.yaw_integral=clip(status_.yaw_integral+model_.yaw_integral_gain*
        (s.turn_deg_s-s.yaw_rate_deg_s)*kDegToRad*dt,
        model_.yaw_integral_limit);
    for(unsigned i=0;i<2;++i) {
        const float target=clip((status_.action[i]-status_.yaw_integral)*model_.action_scale,kMaxCommand);
        status_.command[i]+=clip(target-status_.command[i],delta);
    }
    for(unsigned i=2;i<kOutputs;++i) {
        const float target=clip(status_.action[i],1.0f);
        status_.command[i]+=clip(target-status_.command[i],leg_delta);
    }
    last_tick_us_=s.now_us;ticked_=true;
    if(status_.mode==Mode::Active) {
        float c11=status_.command[0]+(model_.action_space==ActionSpace::Residual?b11:0);
        float c21=status_.command[1]+(model_.action_space==ActionSpace::Residual?b21:0);
        if(dz_config_.max_units>0) {
            const float v_common=.5f*(speed_filtered_[0]+speed_filtered_[1]);
            const bool quiet=std::fabs(s.forward_mm_s)<1e-3f&&std::fabs(s.turn_deg_s)<1e-3f&&
                std::fabs(s.pitch_rate_rad_s)<kDeadZoneQuietPitchRate&&std::fabs(v_common)<kDeadZoneQuietSpeed;
            // The wheels are mirrored: the common mode is (c11-c21)/2 and the preload goes in with opposite signs.
            const float preload=applyDeadZonePreload(dz_state_,dz_config_,.5f*(c11-c21),quiet,dt);
            c11+=preload;c21-=preload;
            status_.dz_preload=dz_state_.preload;status_.dz_sign=dz_state_.sign;status_.dz_flip_rate=dz_state_.flip_rate;
            status_.dz_locked=dz_state_.locked;status_.dz_trips=dz_state_.trips;
        }
        d.wheel11=int(std::lround(clip(c11,kMaxCommand)));
        d.wheel21=int(std::lround(clip(c21,kMaxCommand)));
        if(model_.action_space==ActionSpace::WheelsLegs) {
            d.leg12_m=status_.command[2]*model_.leg_action_scale;
            d.leg22_m=status_.command[3]*model_.leg_action_scale;
        }
        d.applied=true;
    }
    return d;
}
const char* modeName(Mode mode) {switch(mode){case Mode::Off:return "off";case Mode::Shadow:return "shadow";case Mode::Active:return "active";}return "invalid";}
const char* errorName(Error e) {switch(e){case Error::None:return "none";case Error::Model:return "invalid_model";case Error::Untrained:return "untrained_model";case Error::State:return "stale_state";case Error::Numeric:return "nonfinite";case Error::Deadline:return "inference_deadline";case Error::Gap:return "policy_gap";case Error::Reference:return "reference_out_of_range";}return "invalid";}
const char* actionSpaceName(ActionSpace s) {return s==ActionSpace::Wheels?"wheels":s==ActionSpace::WheelsLegs?"wheels_legs":"residual";}
}} // namespace rider::policy
