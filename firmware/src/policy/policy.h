#pragma once
#include <cstddef>
#include <cstdint>
#include "dead_zone_comp.h"

// Float32 actor only. No allocator, interpreter, critic, or training on the MCU.
namespace rider { namespace policy {
constexpr unsigned kContractVersion=6, kInputs=18, kHidden=32, kOutputs=4;
constexpr float kWheelScale=61.6f/55.0f; // Supplied Rider2 STEP vs legacy odometry.
// Contract 6 runs the actor on every control tick (the control loop gates at >=4 ms; kPeriodUs only
// has to be below the shortest tick). A 12 ms hold added ~6 ms of average latency at the loop's
// 7-14 Hz sensitive band.
constexpr uint32_t kPeriodUs=3000, kDeadlineUs=2000, kMaxGapUs=30000;
// Input 17 is the position-error integral (m*s). A memoryless network cannot otherwise remove the
// steady lean/drift produced by an unknown centre-of-mass offset.
constexpr float kPosIntegralLimit=1.0f;
// Inputs 5/6 are the bus wheel speeds low-passed with this time constant. The wheel-speed channel has the
// least predictable gain/phase at 5-14 Hz (measured on the robot), so it is kept out of the fast loop.
constexpr float kSpeedFilterTauS=0.12f;
constexpr float kMaxResidual=128.0f, kMaxCommand=1023.0f, kSlewPerSecond=20000.0f;
// Normalized leg-offset channels move full scale in 200 ms, slower than the
// wheels so servo targets cannot dither at the policy rate.
constexpr float kLegSlewPerSecond=10.0f, kMaxLegActionScale=.02f;
// Dead-zone preload (dead_zone_comp.h): the preload only learns while the robot stands quietly -- no remote command, body
// pitch rate and filtered wheel speed (sensor m/s, ~3.6x slower than the true ground speed) below these. A wheel that is
// already rolling faster than that is not being blocked by the dead zone.
constexpr float kDeadZoneQuietPitchRate=0.5f, kDeadZoneQuietSpeed=0.010f, kMaxDeadZoneUnits=80.0f;
// Defaults the firmware applies at boot (Controller itself starts with the feature off so its unit tests stay exact).
constexpr float kDeadZoneDefaultMaxUnits=55.0f, kDeadZoneDefaultRamp=3.0f, kDeadZoneDefaultInitial=12.0f, kDeadZoneDefaultGrowMean=6.0f;
// This contract's learned direct-wheel profile is qualified for low-speed motion.
constexpr float kMaxForwardMmS=165, kMaxTurnDegS=36, kMaxHeightUnits=29, kMaxRollCommandDeg=4;
enum class ActionSpace:uint8_t { Residual=0, Wheels=1, WheelsLegs=2 };
enum class Mode:uint8_t { Off=0, Shadow=1, Active=2 };
enum class Error:uint8_t { None, Model, Untrained, State, Numeric, Deadline, Gap, Reference };

struct Model {
    const char* name=nullptr;
    uint32_t id=0;
    unsigned contract=kContractVersion;
    bool trained=false;
    ActionSpace action_space=ActionSpace::Residual;
    float action_scale=80, leg_action_scale=.005f, observation_clip=5;
    const float *mean=nullptr, *inv_std=nullptr;
    // Output-major weights, matching torch.nn.Linear.weight[out][in].
    const float *w1=nullptr, *b1=nullptr, *w2=nullptr, *b2=nullptr, *w3=nullptr, *b3=nullptr;
    // Optional, explicitly exported yaw PI profile. The actor still balances.
    float yaw_integral_gain=0,yaw_integral_limit=.05f;
};
bool validateModel(const Model& model);
// Caller validates the immutable model once. Every input/intermediate is checked.
bool infer(const Model& model,const float raw[kInputs],float action[kOutputs]);

struct State {
    uint32_t now_us=0;
    bool fresh=false;
    float pitch_deg=0,roll_deg=0,pitch_rate_rad_s=0,roll_rate_deg_s=0,yaw_rate_deg_s=0;
    float speed11_mm_s=0,speed21_mm_s=0;
    int16_t leg12=0,leg22=0;
    uint16_t zero12=0,zero22=0;
    float forward_mm_s=0,turn_deg_s=0,height_units=0,position_m=0,roll_command_deg=0;
};
struct Status {
    Mode mode=Mode::Off;
    Error error=Error::None;
    bool model_valid=false,trained=false;
    uint32_t calls=0,failures=0,last_us=0,max_us=0,last_interval_us=0;
    float observation[kInputs]{},action[kOutputs]{},command[kOutputs]{};
    float yaw_integral=0,pos_integral=0;
    // Dead-zone preload (command units, signed by the last push direction as -1/0/+1) and how often it chattered.
    float dz_preload=0,dz_flip_rate=0;
    int8_t dz_sign=0;
    bool dz_locked=false;
    uint32_t dz_trips=0;
};
struct Decision {
    int wheel11=0,wheel21=0;
    // Metres added to each leg extension target. Zero unless WheelsLegs is active.
    float leg12_m=0,leg22_m=0;
    bool applied=false,stop=false;
};
using Clock=uint32_t (*)();
class Controller {
public:
    explicit Controller(const Model& model);
    // Firmware requires a latched manual stop before changing modes.
    bool setMode(Mode mode);
    // Arm/stop/calibration discard action history; never reset timing counters.
    void resetHistory();
    Decision step(const State& state,int baseline11,int baseline21,Clock clock);
    const Status& status() const {return status_;}
    const Model& model() const {return model_;}
    // Runtime wheel-command slew limit (units/second). Used by the diagnostic
    // firmware to test damping of high-frequency command chatter.
    void setSlewRate(float rate){if(rate>=1000.0f&&rate<=40000.0f)slew_per_second_=rate;}
    // Dead-zone preload on the common-mode wheel command (dead_zone_comp.h). max_units 0 = off. Changing the
    // configuration keeps what was learned; resetDeadZoneLearning() forgets it.
    bool setDeadZone(const DeadZoneConfig& config);
    const DeadZoneConfig& deadZoneConfig() const {return dz_config_;}
    void resetDeadZoneLearning();
private:
    Decision fail(Error error,int baseline11,int baseline21);
    const Model& model_;
    Status status_{};
    bool ran_=false,ticked_=false;
    uint32_t last_run_us_=0,last_tick_us_=0;
    float position_origin_=0,position_reference_=0;
    float speed_filtered_[2]={0,0};bool speed_filter_ready_=false;
    float slew_per_second_=kSlewPerSecond;
    DeadZoneConfig dz_config_{};
    DeadZoneState dz_state_{};
};
const char* modeName(Mode mode);
const char* errorName(Error error);
const char* actionSpaceName(ActionSpace space);
}} // namespace rider::policy
