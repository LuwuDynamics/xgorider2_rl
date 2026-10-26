#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include "board_imu.h"
#include "calibration.h"
#include "device_settings.h"
#include "actuator_bus.h"
#include "board_support.h"
#include "action_engine.h"
#include "motion_controller.h"
#if defined(RIDER2_RL_ONLY) && RIDER2_RL_ONLY == 1
#include "leg_position_controller.h"
#include "learned_wheel_drive.h"
#if !defined(RIDER2_TRAINED_POLICY) || RIDER2_TRAINED_POLICY != 1
#error RL-only requires the trained direct-wheel actor.
#endif
extern "C" {extern const volatile uint32_t rider_rl_only_build=0x524c4f31;}
#endif
#include "runtime_guard.h"
#include "odometry.h"
#include "command_protocol.h"
#include "ble_transport.h"
#include "trace.h"
#include "lift_guard.h"
#include "fall_guard.h"
#include "led_status.h"
#include "policy/policy.h"
#if defined(RIDER2_TRAINED_POLICY) && RIDER2_TRAINED_POLICY == 1
#include "policy/trained_weights.h"
#else
#include "policy/policy_weights.h"
#endif

#if !defined(RIDER2_MOTOR_OUTPUT) || RIDER2_MOTOR_OUTPUT != 1
#error This project is the motor-enabled Rider2 application.
#endif
using namespace rider::firmware;
using namespace rider::bringup;
using namespace rider2::protocol;
namespace rl = rider::policy;
static_assert(rl::weights::kExportedContract==rl::kContractVersion,"Regenerate policy weights for this observation contract");
namespace {
constexpr uint32_t kImuPeriodUs=2000,kControlPeriodUs=4000;
// Consecutive 4 ms control ticks the robot must remain plausibly upright and
// still before a protective stop may re-select the learned controller.
constexpr uint16_t kAutoRearmTicks=250;
constexpr uint8_t kIds[]={11,21,12,22};
// Diagnostic trace: 28-byte samples, about 55 Hz for 20 seconds. The control
// task fills the ring (sample + USB-only companion record); the application
// task streams it over BLE on request (0xb3) or exports it as text over USB
// (!trace dump, !trace usb on) -- the USB route leaves Bluetooth free for the phone.
using TraceSample=rider2::trace::Sample;
constexpr uint32_t kTraceCapacity=1000;   // the static DRAM region is nearly full: 1200 entries with the 40-byte companion record overflowed it by 1.2 KB
TraceSample trace_samples[kTraceCapacity];
rider2::trace::Extra trace_extra[kTraceCapacity];
volatile uint32_t trace_head=0;
volatile bool trace_stream_ble=false;
rider2::trace::UsbCursor trace_usb;   // only the application task (console + loop) touches it
volatile float wheel_gain=1.0f;
// Scale applied to the wheel speeds the *policy* sees. The training simulator assumes the speed register
// reads about 1/pi of the true rim speed (identified from the robot's own ringing); if a bench check
// (tools/calibrate_wheel_speed.py) shows a different ratio, set this instead of retraining.
volatile float speed_cal=1.0f;
// Lift guard (src/lift_guard.h): hold the wheel command at zero while the wheels have run away in the air.  The detector
// always runs and is recorded in the trace/status; this switch (!liftguard on/off) decides whether it also acts.
volatile bool lift_guard_enabled=true;
// Free-fall guard (src/fall_guard.h): the stop (wheels to zero, leg servos released) when the accelerometer reads below 0.5 g for
// `fall_guard_ms` (default 60, `!fallguard ms N`, RAM only; the console writes it, the control task reads it once per sensor tick).
// The detector always runs and is recorded; this switch (!fallguard on/off) decides whether it also stops the robot.
volatile bool fall_guard_enabled=true;
volatile float fall_guard_ms=rider2::fall::Config().freefall_ms;
volatile bool fall_clear_request=false;   // !fallguard clear: the control task zeroes the statistics and the trip counters
// Dead-zone preload (src/policy/dead_zone_comp.h): the console writes these, the control task (which owns the controller)
// applies them.  dz_max_units 0 = off; what was learned is kept while it is off.
volatile float dz_max_units=rl::kDeadZoneDefaultMaxUnits;
volatile float dz_hysteresis=8.0f;
volatile float dz_ramp=rl::kDeadZoneDefaultRamp;
volatile float dz_initial=rl::kDeadZoneDefaultInitial;   // preload at boot: what this robot needs, nothing to learn
volatile float dz_grow_mean=rl::kDeadZoneDefaultGrowMean; // learn upwards only while the controller is blocked
volatile uint32_t dz_reset_request=0;
// Shown in !status so a robot can be told apart from its firmware after a few flashes.
constexpr const char* kBuildId="rider2-rl-only | rider_balance_v6 | lift guard v4 | dead-zone preload v3 | free-fall guard 60 ms + low-g = fall (v3) + LED fault codes | servo-dropout auto re-arm | 2026-10-24";
volatile uint32_t wheel_smooth_ms=0;
volatile float yaw_slew_per_second=20000.0f;
float yaw_command_filtered=0;
bool yaw_filter_init=false;
uint32_t last_filter_us=0;
bool wheel_filter_init=false;
float wheel_filtered[2]={0,0};
uint32_t trace_div=0;
enum class EventKind:uint8_t { Protocol,Arm,Neutral,ResetOdometry };
struct Event {EventKind kind=EventKind::Protocol;CommandEvent command{};};
struct Timing {
    uint32_t count=0,last_us=0,min_us=UINT32_MAX,max_us=0,max_work_us=0;
    uint64_t sum_us=0;
    void start(uint32_t us){if(count){uint32_t d=us-last_us;min_us=std::min(min_us,d);max_us=std::max(max_us,d);sum_us+=d;}last_us=us;++count;}
    void end(uint32_t us){max_work_us=std::max(max_work_us,uint32_t(us-last_us));}
};
struct Snapshot {
    uint32_t ms=0,imu_ms=0,rejected_commands=0,imu_rejected=0;
    bool driving=false,manual_stop=false,leg_calibrating=false,stream=false,imu_valid=false;
    uint8_t mode=253,action=0,pitch_calibrating=0,setting06=0;
    Source stream_source=Source::Usb;
    StopReason reason=StopReason::Startup;
    StopReason last_stop_reason=StopReason::Startup;
    uint32_t rl_output_ticks=0;
    rider::control::ImuState imu{};
    rider2::ImuDiagnostics imu_diag{};
    rider2::Calibration calibration{};
    rider::board::BoardState battery{};
    ActuatorSample actuators[4]{};
    ActuatorBusStats bus{};
    MotionResult motion{};
    Telemetry telemetry{};
    Timing imu_timing{},control_timing{};
    rl::Status policy{};
    bool lift_active=false,lift_acting=false;
    uint32_t lift_trips=0;
    bool fall_low=false,fall_falling=false;
    uint32_t fall_trips=0,fall_deep_trips=0;
    float fall_min_g=0,fall_longest_low_ms=0;
    char last_result[64]="booting";
};
Snapshot shared;
portMUX_TYPE stateLock=portMUX_INITIALIZER_UNLOCKED;
bool emergencyRequested=false;
QueueHandle_t commands=nullptr;
HardwareSerial commandSerial(1);
StreamParser parser;
bool bleReady=false;
bool robotTaskStarted=false;
uint32_t policyClock(){return micros();}

Snapshot snapshot(){Snapshot s;portENTER_CRITICAL(&stateLock);s=shared;portEXIT_CRITICAL(&stateLock);return s;}
bool enqueue(const Event& e){return commands&&xQueueSend(commands,&e,0)==pdTRUE;}
void emergencyStop(){portENTER_CRITICAL(&stateLock);emergencyRequested=true;portEXIT_CRITICAL(&stateLock);}

class Robot {
public:
    rider2::BoardImu imu_board;
    ActuatorBus bus;
    rider::board::BoardSupport board;
    rider2::ActionEngine actions;
#if RIDER2_RL_ONLY
    LegPositionController controller;
    LearnedWheelDrive policy{rl::weights::compiledModel()};
#else
    MotionController controller;
    rl::Controller policy{rl::weights::compiledModel()};
#endif
    MotionRequest request;
    OriginalOdometry odometry;
    Snapshot s;
    bool imu_valid=false,imu_low_g_only=false,driving=false,manual_stop=false,leg_calibrating=false;   // imu_low_g_only: the last read worked but showed ~0 g
    bool save_legs_pending=false,stop_sent=false,tilt_fault_stop=false;
    uint16_t save_a=0,save_b=0,trim_stable=0,runaway_count=0;
    uint16_t auto_rearm_ticks=0;
    uint32_t dz_reset_seen=0;
    uint32_t save_started=0,last_imu_us=0,last_control_us=0,last_publish_ms=0;
    uint8_t sensor_loops=0,trim_stage=0;
    rider2::lift::State lift_state;rider2::lift::Config lift_config;bool lift_acting=false;uint32_t control_dt_us=0;
    rider2::fall::State fall_state,fall_deep_state;rider2::fall::Config fall_config,fall_deep_config=rider2::fall::deepConfig();uint32_t fall_last_us=0;
    double trim_to_save=0;
    bool save_trim_pending=false;
    float heading_limit=300;
    void result(const char* text){std::snprintf(s.last_result,sizeof s.last_result,"%s",text);}
    void resetFusion(){s.imu={};s.imu.last_us=micros();imu_valid=false;imu_low_g_only=false;controller.reset(millis());policy.resetHistory();}
    bool prepareMotorBus() {
        // setup owns the bus until the control task is created. Stop retained
        // actuator commands before BLE initialization can block the CPU.
        bus.begin();bus.pauseFeedback(true);bus.stop(true);stop_sent=true;
        const uint32_t errors_before=bus.stats().tx_errors,began=millis();
        do {
            bus.poll(millis());
            if(bus.transportIdle())return bus.stats().tx_errors==errors_before;
            delay(1);
        }while(uint32_t(millis()-began)<100);
        return false;
    }
    void begin() {
        s.calibration=rider2::readCalibration();
        // Extended trim record is loaded below when present; legacy machines
        // retain the original low32/zero-high32 decoded value.
        loadExtendedTrim();
        board.begin();imu_board.begin();bus.pauseFeedback(false);
        s.imu.last_us=micros();last_imu_us=last_control_us=micros();
#if RIDER2_RL_ONLY
        actions.state().auto_recover=false;
        {
            auto& st=actions.state();
            st.command.forward=st.command.turn=st.command.roll=128;
            st.command.height=128;  // trained leg-height centre, not the low pose
            st.leveling=true;
        }
        if(!policy.arm()){manual_stop=true;actions.requestMode(252,true);result("RL model rejected; stop latched");return;}
#endif
        result("ready; waiting for fresh sensors and upright calibration");
    }
    void loadExtendedTrim();
    void cancelTrim() {
        trim_stage=0;trim_stable=0;save_trim_pending=false;
        loadExtendedTrim();
    }
    void stop(StopReason reason,bool latch) {
        if(!manual_stop||latch)s.last_stop_reason=reason;
        if(driving){policy.resetHistory();if(policy.status().mode==rl::Mode::Active)policy.setMode(rl::Mode::Off);}
        tilt_fault_stop=false;lift_state=rider2::lift::State();lift_acting=false;
        if(driving||!stop_sent){bus.stop(true);stop_sent=true;}
        driving=false;s.driving=false;s.reason=reason;
        // Published commands must reflect the requested stop, not the last
        // controller result. The bus stop has zeroed wheels and released legs.
        s.motion.wheels={};s.motion.legs={};
        wheel_filter_init=false;yaw_filter_init=false;last_filter_us=millis()*1000;
        controller.reset(millis(),s.imu.yaw_deg);
        if(latch)manual_stop=true;
    }
    bool stopForFlash() {
        const uint32_t errors_before=bus.stats().tx_errors;
        bus.pauseFeedback(true);bus.stop(true);driving=false;stop_sent=true;
        const uint32_t began=millis();
        do {
            bus.poll(millis());
            if(bus.stats().tx_errors!=errors_before) {
                manual_stop=true;actions.requestMode(251,true);bus.pauseFeedback(false);
                result("flash refused: motor stop transmission failed; stop latched");return false;
            }
            if(bus.transportIdle())return true;
            delay(1);
        }while(uint32_t(millis()-began)<100);
        manual_stop=true;actions.requestMode(251,true);bus.pauseFeedback(false);
        result("flash refused: motor stop transmission did not drain");return false;
    }
    void afterFlash(){policy.setMode(rl::Mode::Off);bus.pauseFeedback(false);resetFusion();actions.requestMode(253,true);stop_sent=false;}
    void enterCalibration() {
        policy.setMode(rl::Mode::Off);
        cancelTrim();
        actions.cancel(millis());actions.requestMode(255,true);leg_calibrating=true;
        save_legs_pending=false;
        stop(StopReason::Calibration,false);result("leg calibration entered; torque released");
    }
    void saveCalibration() {
        const uint32_t now=millis();const auto& a=bus.sample(12);const auto& b=bus.sample(22);
        if(!leg_calibrating||save_legs_pending||!a.fresh(now,250)||!b.fresh(now,250)){
            ++s.rejected_commands;result("leg save refused: enter calibration and obtain fresh legs");return;
        }
        save_a=uint16_t(a.position_ticks);save_b=uint16_t(b.position_ticks);
        save_started=now;save_legs_pending=true;result("leg zero positions captured; saving after 1500ms");
    }
    void arm() {
        if(leg_calibrating||save_legs_pending){result("arm refused during leg calibration write");++s.rejected_commands;return;}
#if RIDER2_RL_ONLY
        if(!policy.arm()){stop(StopReason::Policy,true);actions.requestMode(252,true);result("arm refused: RL model unavailable");++s.rejected_commands;return;}
        {auto& st=actions.state();if(st.command.height==0)st.command.height=128;}
#endif
        cancelTrim();
        manual_stop=false;actions.cancel(millis());actions.requestMode(253,true);
        controller.reset(millis(),s.imu.yaw_deg);policy.resetHistory();stop_sent=false;
        auto_rearm_ticks=0;
        result("arm requested; waiting for valid upright sensor state");
    }
    void dispatch(const CommandEvent& e) {
        auto& a=actions.state();const uint8_t value=e.data[0];const uint32_t now=millis();
        const auto kind=commandKind(e);
        if(board.state().low_stage && kind!=CommandKind::TelemetryStream){++s.rejected_commands;return;}
        switch(kind) {
        case CommandKind::ActionLoop:actions.setLoop(value,now);break;
        case CommandKind::LegCalibration:if(value==1)enterCalibration();else if(value==0)saveCalibration();break;
        case CommandKind::ReservedNoop:break;
        case CommandKind::UnknownSetting06:s.setting06=value;break;
        case CommandKind::TelemetryStream:s.stream=value!=0;s.stream_source=e.source;break;
        case CommandKind::PitchCalibration:
#if RIDER2_RL_ONLY
            ++s.rejected_commands;result("wheel PID pitch trim unavailable in RL-only firmware");break;
#else
            if(policy.status().mode==rl::Mode::Active){++s.rejected_commands;result("stop and disable policy before pitch trim");break;}
            if(value==1&&!leg_calibrating&&!manual_stop){actions.enable(now);trim_stage=1;trim_stable=0;result("pitch calibration requested");}break;
#endif
        case CommandKind::PolicyMode:
            if(!manual_stop||driving||leg_calibrating||save_legs_pending||save_trim_pending){++s.rejected_commands;result("policy mode requires a latched manual stop");break;}
            if(!policy.setMode(static_cast<rl::Mode>(value))){++s.rejected_commands;result(policy.status().model_valid?"policy rejected: trained model required":"policy rejected: invalid model");}
            else result(value==2?"policy active selected; arm explicitly":"policy mode selected");
            break;
        case CommandKind::TraceStream:trace_stream_ble=value!=0;result(value?"trace streaming enabled":"trace streaming disabled");break;
        case CommandKind::WheelGain:wheel_gain=float(value)/100.0f;result("wheel gain updated");break;
        case CommandKind::SpeedCal:
            if(driving||value<13){++s.rejected_commands;result("policy speed scale rejected (needs standstill, value 13..255 = 0.2..3.98)");break;}
            speed_cal=float(value)/64.0f;result("policy speed scale updated");break;
        case CommandKind::WheelSmoothMs:wheel_smooth_ms=unsigned(value);result("wheel smoothing updated");break;
        case CommandKind::WheelSlew:policy.setSlewRate(1000.0f+float(value)*18000.0f/255.0f);result("wheel slew updated");break;
        case CommandKind::YawSlew:yaw_slew_per_second=1000.0f+float(value)*18000.0f/255.0f;result("yaw slew updated");break;
        case CommandKind::HeadingHold:a.heading_hold=value!=0;break;
        case CommandKind::BleSuffix:
            cancelTrim();
            stop(StopReason::Manual,true);
            if(stopForFlash()){
                const bool ok=rider2::saveBleSuffix(e.data,e.length);afterFlash();
                result(ok?"BLE name saved; takes effect after restart; use arm to resume":"BLE name write failed");
            }break;
        case CommandKind::Forward:a.command.forward=value;break;
        case CommandKind::Turn:a.command.turn=value;if(std::abs(int(value)-128)>16)a.leveling=false;break;
        case CommandKind::Height:a.command.height=value;break;
        case CommandKind::Roll:a.command.roll=value;break;
        case CommandKind::RollWave:a.command.roll_period=value;a.roll_phase=value?10:0;break;
        case CommandKind::HeightWave:a.command.height_period=value;a.height_phase=value?10:0;break;
        case CommandKind::Action:if(value==255)arm();else if(!leg_calibrating)actions.request(value,now);break;
        case CommandKind::PositionHoldProtection:a.hold_protection=value!=0;break;
        case CommandKind::Leveling:a.leveling=value!=0;break;
        case CommandKind::MaxForward:a.command.max_forward=float(double(value)/255.0*210.0);break;
        case CommandKind::MaxTurn:
            a.command.max_turn=float(double(value)/255.0*300.0);
            heading_limit=float((double(value)*.75/255.0+.25)*300.0);break;
        case CommandKind::Rgb0:case CommandKind::Rgb1:case CommandKind::Rgb2:case CommandKind::Rgb3:
            if(e.length==12){for(uint8_t n=0;n<4;++n)board.setUserLed(n,e.data[n*3],e.data[n*3+1],e.data[n*3+2]);}
            else board.setUserLed(uint8_t(e.reg-0x69),e.data[0],e.data[1],e.data[2]);
            break;
        default:++s.rejected_commands;break;
        }
    }
    void events() {
        bool emergency;
        portENTER_CRITICAL(&stateLock);emergency=emergencyRequested;emergencyRequested=false;portEXIT_CRITICAL(&stateLock);
        if(emergency) {
            policy.setMode(rl::Mode::Off);
            xQueueReset(commands);manual_stop=true;leg_calibrating=false;
            save_legs_pending=false;cancelTrim();
            actions.cancel(millis());actions.requestMode(254,true);stop(StopReason::Manual,true);result("manual stop latched");
        }
        Event event;
        // Bounded work per iteration; packet floods cannot starve the controller.
        for(unsigned n=0;n<4&&xQueueReceive(commands,&event,0)==pdTRUE;++n) {
            switch(event.kind){
            case EventKind::Arm:arm();break;
            case EventKind::Neutral:actions.cancel(millis());actions.state().command.forward=128;actions.state().command.turn=128;break;
            case EventKind::ResetOdometry:odometry={};odometry.last_ms=millis();break;
            case EventKind::Protocol:dispatch(event.command);break;
            }
        }
    }
    GuardInput guard(uint32_t now,bool overrun=false) {
        GuardInput g;g.now_ms=now;g.imu_ms=s.imu_ms;g.imu_valid=imu_valid;
        g.imu_calibrated=s.imu.calibrated;g.pitch=s.imu.pitch_deg;g.roll=s.imu.roll_deg;
        g.calibration_valid=s.calibration.flash_read_ok&&s.calibration.marker_valid&&s.calibration.zeros_valid&&s.calibration.pitch_trim_valid;
        g.manual_stop=manual_stop;g.calibrating=leg_calibrating||save_trim_pending||board.state().calibration_pressed;
        g.low_battery=board.state().stop_required;g.control_overrun=overrun;
        // A real free fall reads below 0.061 g, which the IMU validity gate in sensor() calls "unusable": without this the robot stopped ~30 ms later
        // as an IMU fault (no red LED, not re-armable).  While the deep detector sees such samples the IMU counts as valid, and the stop is a fall.
        const bool deep_low=fall_guard_enabled&&fall_deep_state.low;
        if(deep_low&&!imu_valid){g.imu_valid=true;g.imu_ms=now;}
        g.falling=fall_guard_enabled&&(fall_state.falling||fall_deep_state.falling);
        g.wheel11_valid=bus.sample(11).valid;g.wheel21_valid=bus.sample(21).valid;
        g.leg12_valid=bus.sample(12).valid;g.leg22_valid=bus.sample(22).valid;
        g.wheel11_ms=bus.sample(11).last_update_ms;g.wheel21_ms=bus.sample(21).last_update_ms;
        g.leg12_ms=bus.sample(12).last_update_ms;g.leg22_ms=bus.sample(22).last_update_ms;
        return g;
    }
    void sensor(uint32_t us) {
        s.imu_timing.start(us);uint8_t raw[12];
        fall_config.freefall_ms=fall_guard_ms;   // the console may have changed the hold time
        if(fall_clear_request){fall_clear_request=false;fall_state=rider2::fall::State();fall_deep_state=rider2::fall::State();}
        if(imu_board.read(raw)) {
            uint32_t sample_us=micros(),now=millis();
            if(rider2::fall::imuFusionRestart(imu_valid,imu_low_g_only,uint32_t(now-s.imu_ms))){s.imu={};s.imu.last_us=sample_us;}
            const int16_t ax=int16_t(uint16_t(raw[0])<<8|raw[1]),ay=int16_t(uint16_t(raw[2])<<8|raw[3]),az=int16_t(uint16_t(raw[4])<<8|raw[5]);
            const double norm=double(ax)*ax+double(ay)*ay+double(az)*az;
            {   // free-fall detector: from the raw sample, before the IMU validity gate (1 g = 16384 counts, +-2 g full scale)
                rider2::fall::Input fin;fin.accel_g=float(std::sqrt(norm))/16384.0f;
                fin.dt_ms=float(uint32_t(sample_us-fall_last_us))*0.001f;fall_last_us=sample_us;
                fall_state=rider2::fall::update(fall_state,fall_config,fin);
                fall_deep_state=rider2::fall::update(fall_deep_state,fall_deep_config,fin);
            }
            const bool low_g_read=norm<=1000000&&imu_board.diagnostics().identical_samples<250;   // the chip answered, the acceleration is ~0: a fall, not a fault
            imu_valid=norm>1000000&&imu_board.diagnostics().identical_samples<250&&
                rider::control::updateImu(s.imu,raw,12,sample_us,now,actions.state().mode)&&
                std::isfinite(s.imu.pitch_deg)&&std::isfinite(s.imu.roll_deg)&&std::isfinite(s.imu.yaw_deg)&&std::isfinite(s.imu.pitch_rate_rad_s);
            imu_low_g_only=!imu_valid&&low_g_read;
            if(imu_valid)s.imu_ms=now;else ++s.imu_rejected;
            // A tilt while already stopped is only the operator handling the
            // robot. Requesting mode 0 here would strand it (no arm button in
            // the phone app); only stop an actually driving robot.
            if(s.imu.tilt_fault&&driving){tilt_fault_stop=true;actions.requestMode(0);}
        }else {
            imu_valid=false;imu_low_g_only=false;
            rider2::fall::Input fin;fin.fresh=false;fall_state=rider2::fall::update(fall_state,fall_config,fin);   // a missing read is not evidence
            fall_deep_state=rider2::fall::update(fall_deep_state,fall_deep_config,fin);
        }
        s.imu_diag=imu_board.diagnostics();board.poll(millis());
        switch(board.takeCalibrationEvent()){
        case rider::board::CalibrationSwitchEvent::Enter:enterCalibration();break;
        case rider::board::CalibrationSwitchEvent::Save:if(leg_calibrating)saveCalibration();break;
        default:break;
        }
        const auto power=board.state();
        if(power.neutralize){auto& a=actions.state();a.command.forward=a.command.turn=a.command.roll=128;a.command.height=0;a.height_phase=a.roll_phase=0;actions.cancel(millis());a.command.height=0;}
        if(power.stop_required)actions.requestMode(250,true);
        if(imu_valid)odometry.update(millis(),bus.sample(21).encoder_ticks,bus.sample(11).filtered_speed_mm_s,s.imu.yaw_deg);
        // Original runaway criterion is count based and uses wheel11.
        if(actions.state().mode==1&&std::fabs(bus.sample(11).filtered_speed_mm_s)>300&&
           std::abs(int(s.motion.wheels.id11))>=701&&std::fabs(s.motion.command.forward)<20)++runaway_count;
        else runaway_count=0;
        if(runaway_count>350)actions.requestMode(0);
        if(++sensor_loops==21){sensor_loops=0;trimTick();if(!manual_stop&&!leg_calibrating&&!power.low_stage)actions.tick(millis());}
        s.imu_timing.end(micros());
    }
    void trimTick() {
#if !RIDER2_RL_ONLY
        if(!trim_stage)return;
        if(trim_stage==1){actions.enable(millis());s.calibration.pitch_trim=0;trim_stage=2;trim_stable=0;}
        auto& a=actions.state();a.command.forward=a.command.turn=a.command.roll=128;a.command.height=144;
        a.action_id=0;a.roll_phase=a.height_phase=0;
        if(!driving||a.mode!=1||outputGuard(guard(millis()))!=StopReason::None){trim_stable=0;return;}
        const float speed=(bus.sample(11).filtered_speed_mm_s+bus.sample(21).filtered_speed_mm_s)*.5f;
        if(controller.holdMode()==2&&std::fabs(speed)<8)++trim_stable;else trim_stable=0;
        if(trim_stable>29){trim_to_save=-double(controller.holdCorrection())*.125;save_trim_pending=true;trim_stage=0;}
#endif
    }
    void persistence() {
        if(save_legs_pending&&uint32_t(millis()-save_started)>=1500) {
            save_legs_pending=false;
            if(stopForFlash()) {
                const bool ok=rider2::saveLegCalibration(save_a,save_b);s.calibration=rider2::readCalibration();loadExtendedTrim();
                leg_calibrating=false;save_legs_pending=false;
                if(ok) {
                    // The phone app has no separate arm/stand control. Match the
                    // app contract ("完成标定...并恢复站姿"): clear the stop and
                    // re-arm the learned controller immediately after the save.
                    manual_stop=false;afterFlash();arm();
                    result("leg calibration saved and verified; standing restored");
                } else {
                    manual_stop=true;afterFlash();
                    result("leg calibration write failed; remains stopped");
                }
            }
        }
        if(save_trim_pending) {
            if(stopForFlash()) {
                const bool ok=rider2::savePitchTrim(trim_to_save);save_trim_pending=false;
                if(ok){s.calibration.pitch_trim=trim_to_save;s.calibration.pitch_trim_valid=true;}
                manual_stop=true;afterFlash();result(ok?"pitch trim saved and verified; arm to resume":"pitch trim rejected or write failed");
            }else {save_trim_pending=false;manual_stop=true;}
        }
    }
    rl::Decision runPolicy(uint32_t us,bool moving) {
        const uint32_t now=millis();rl::State state;state.now_us=us;
        // Free fall (the deep detector sees reads below 0.061 g): the IMU is alive, only its attitude update is skipped for a few ms.  Without this the
        // policy's own freshness test stopped the robot first, as "policy_fault" (first v6 real drops, 2026-10-24); the fall guard stops it as a fall.
        const bool deep_low=fall_guard_enabled&&fall_deep_state.low;
        state.fresh=(imu_valid||deep_low)&&(uint32_t(now-s.imu_ms)<=30||deep_low)&&
            bus.sample(11).fresh(now,100)&&bus.sample(21).fresh(now,100)&&
            bus.sample(12).fresh(now,250)&&bus.sample(22).fresh(now,250);
        state.pitch_deg=s.imu.pitch_deg;state.roll_deg=s.imu.roll_deg;
        state.pitch_rate_rad_s=s.imu.pitch_rate_rad_s;
        state.roll_rate_deg_s=s.imu.gyro_roll_filtered;
        state.yaw_rate_deg_s=s.imu.gyro_deg_s[2]-s.imu.gyro_bias[2];
        state.speed11_mm_s=bus.sample(11).wheel_speed_mm_s*rl::kWheelScale*speed_cal;
        state.speed21_mm_s=bus.sample(21).wheel_speed_mm_s*rl::kWheelScale*speed_cal;
        state.position_m=rider::control::wheelDistance(bus.sample(11).encoder_ticks,bus.sample(21).encoder_ticks)*rl::kWheelScale;
        state.leg12=bus.sample(12).position_ticks;state.leg22=bus.sample(22).position_ticks;
        state.zero12=s.calibration.zero_a;state.zero22=s.calibration.zero_b;
        if(moving){
            state.forward_mm_s=s.motion.command.filtered_forward;state.turn_deg_s=s.motion.command.filtered_turn;
            state.height_units=s.motion.command.filtered_height;
            state.roll_command_deg=s.motion.command.roll;
        }
        {   // dead-zone preload settings written by the console
            const float mx=dz_max_units,hy=dz_hysteresis,rp=dz_ramp,ini=dz_initial,gm=dz_grow_mean;
            const auto& cur=policy.deadZoneConfig();
            if(mx!=cur.max_units||hy!=cur.hysteresis||rp!=cur.ramp_units_per_s||ini!=cur.initial_units||gm!=cur.grow_mean_u){
                auto config=cur;config.max_units=mx;config.hysteresis=hy;config.ramp_units_per_s=rp;config.initial_units=ini;config.grow_mean_u=gm;
                policy.setDeadZone(config);
            }
            if(dz_reset_request!=dz_reset_seen){dz_reset_seen=dz_reset_request;policy.resetDeadZoneLearning();}
        }
#if RIDER2_RL_ONLY
        return policy.step(state,policyClock);
#else
        return policy.step(state,moving?s.motion.wheels.id11:0,moving?s.motion.wheels.id21:0,policyClock);
#endif
    }
    void traceTick() {
        if((++trace_div%3)!=0)return;
        const auto q=[](float value,float scale){float x=value*scale;
            if(x>32767.0f)x=32767.0f;if(x<-32768.0f)x=-32768.0f;return int16_t(x);};
        TraceSample t;
        t.pitch=q(s.imu.pitch_deg,100);t.pitch_rate=q(s.imu.pitch_rate_rad_s,1000);
        t.roll=q(s.imu.roll_deg,100);t.yaw_rate=q(s.imu.gyro_deg_s[2]-s.imu.gyro_bias[2],100);
        t.cmd11=s.motion.wheels.id11;t.cmd21=s.motion.wheels.id21;
        t.sp11=q(bus.sample(11).wheel_speed_mm_s,1);t.sp21=q(bus.sample(21).wheel_speed_mm_s,1);
        t.fsp11=q(bus.sample(11).filtered_speed_mm_s,1);t.fsp21=q(bus.sample(21).filtered_speed_mm_s,1);
        t.leg_a=int16_t(s.motion.legs.servo_a);t.leg_b=int16_t(s.motion.legs.servo_b);
        t.a0=q(policy.status().command[0],1);t.a1=q(policy.status().command[1],1);
        rider2::trace::Extra extra;
        extra.ms=millis();
        extra.load11=bus.sample(11).load_raw;extra.load21=bus.sample(21).load_raw;
        extra.pos11=bus.sample(11).position_ticks;extra.pos21=bus.sample(21).position_ticks;
        extra.acc_x=q(s.imu.acceleration[0],100);extra.acc_y=q(s.imu.acceleration[1],100);extra.acc_z=q(s.imu.acceleration[2],100);
        {
            const uint32_t at=millis();
            const auto age=[&](uint8_t id){const uint32_t a=bus.sample(id).ageMs(at);return int16_t(a>32767u?32767u:a);};
            extra.age11=age(11);extra.age21=age(21);extra.age12=age(12);extra.age22=age(22);
            extra.dt_us=uint16_t(control_dt_us>65535u?65535u:control_dt_us);
            extra.load12=bus.sample(12).load_raw;extra.load22=bus.sample(22).load_raw;
            extra.pos12=bus.sample(12).position_ticks;extra.pos22=bus.sample(22).position_ticks;
            extra.vbat_adc=board.state().adc_raw;
        }
        {
            const auto& pst=policy.status();
            extra.flags=int16_t((lift_acting?1:0)|(lift_state.active?2:0)|(lift_state.fast?4:0)|
                (dz_max_units>0&&pst.dz_preload>0.5f?8:0)|(pst.dz_sign>0?16:0)|(pst.dz_sign<0?32:0)|(fall_state.low?64:0)|(fall_state.falling?128:0));
        }
        const uint32_t slot=trace_head%kTraceCapacity;
        trace_samples[slot]=t;trace_extra[slot]=extra;++trace_head;
    }
    void control(uint32_t us) {
        const bool overrun=driving&&uint32_t(us-last_control_us)>30000;
        control_dt_us=uint32_t(us-last_control_us);
        last_control_us=us;s.control_timing.start(us);const uint32_t now=millis();
        GuardInput g=guard(now,overrun);StopReason reason=stopReasonWhileDriving(outputGuard(g),driving);auto& a=actions.state();
#if RIDER2_RL_ONLY
        if(driving&&tilt_fault_stop&&reason==StopReason::None)reason=StopReason::Tilt;
#endif
        if(reason!=StopReason::None)a.upright_control_ticks=0;
        if(a.mode==253&&!manual_stop&&!leg_calibrating&&board.state().battery_valid&&uprightForStart(g)){
            actions.cancel(now);actions.requestMode(1,true);
#if RIDER2_RL_ONLY
            // ActionEngine::controlTick neutralises height while the robot is
            // waiting to start; restore the trained leg-height centre here.
            if(a.command.height==0)a.command.height=128;
#endif
            controller.reset(now,s.imu.yaw_deg);result("balance enabled");
        }
#if RIDER2_RL_ONLY
        // A protective stop clears the actor and latches manual_stop. For stops
        // that only mean the robot toppled, allow the learned controller to be
        // re-selected after the robot has been set back upright and still, so a
        // physical recovery does not need a phone/USB command each time. Manual
        // stops, low battery, calibration and hardware faults stay latched.
        if(!driving&&!leg_calibrating&&!save_legs_pending&&!save_trim_pending&&
           policy.status().mode==rl::Mode::Off&&rearmableStop(s.last_stop_reason)) {
            GuardInput candidate=g;candidate.manual_stop=false;
            const bool steady=outputGuard(candidate)==StopReason::None&&
                std::fabs(candidate.pitch)<8&&std::fabs(candidate.roll)<5&&
                std::fabs(bus.sample(11).filtered_speed_mm_s)<30.0f&&
                std::fabs(bus.sample(21).filtered_speed_mm_s)<30.0f;
            if(!steady)auto_rearm_ticks=0;
            else if(++auto_rearm_ticks>=kAutoRearmTicks) {
                auto_rearm_ticks=0;
                if(policy.arm()) {
                    manual_stop=false;actions.cancel(now);actions.requestMode(253,true);
                    if(a.command.height==0)a.command.height=128;
                    controller.reset(now,s.imu.yaw_deg);
                    result("RL auto re-armed after protective stop; waiting for upright start");
                }
            }
        } else auto_rearm_ticks=0;
#endif
        if(driving&&reason!=StopReason::None) {
            if(reason==StopReason::Tilt||reason==StopReason::Fall)actions.requestMode(0,true);
            else if(reason==StopReason::LowBattery)actions.requestMode(250,true);
            else if(reason==StopReason::Actuators)actions.requestMode(251,true);
            else if(reason!=StopReason::Calibration&&reason!=StopReason::Manual)actions.requestMode(252,true);
        }
        if(reason==StopReason::None&&!manual_stop&&!leg_calibrating)actions.controlTick(now,s.imu.pitch_deg,s.imu.roll_deg);
        const auto ev=actions.takeEvents();
#if !RIDER2_RL_ONLY
        if(ev.has(rider2::HoldReset))controller.resetHold(a.last_reset_ms);
        if(ev.has(rider2::SpeedPitchPidReset))controller.resetStoppedPid();
#else
        (void)ev; // No wheel PID/position-hold histories exist in this build.
#endif
        if(reason!=StopReason::None||a.mode!=1){
#if RIDER2_RL_ONLY
            const bool latch=driving&&reason!=StopReason::None;
            stop(reason==StopReason::None?StopReason::Startup:reason,latch);
#else
            stop(reason==StopReason::None?StopReason::Startup:reason,false);
#endif
            // Shadow is also usable while physically stopped; it cannot write motors.
            if(policy.status().mode==rl::Mode::Shadow)runPolicy(us,false);
            s.control_timing.end(micros());return;
        }
        MotionInput in;in.now_ms=now;in.mode=a.mode;in.calibrated=s.imu.calibrated;
        in.pitch_deg=s.imu.pitch_deg;in.roll_deg=s.imu.roll_deg;in.yaw_deg=s.imu.yaw_deg;in.pitch_rate_rad_s=s.imu.pitch_rate_rad_s;
        in.speed11=bus.sample(11).filtered_speed_mm_s;in.speed21=bus.sample(21).filtered_speed_mm_s;
        in.encoder11=bus.sample(11).encoder_ticks;in.encoder21=bus.sample(21).encoder_ticks;in.leg12_position=bus.sample(12).position_ticks;
        in.zero_a=s.calibration.zero_a;in.zero_b=s.calibration.zero_b;in.pitch_trim=s.calibration.pitch_trim;
        request.command=a.command;
#if RIDER2_RL_ONLY
        // Training saw height bytes 55..200; keep the phone slider inside that band.
        if(request.command.height<55)request.command.height=55;
        else if(request.command.height>200)request.command.height=200;
#endif
        request.roll_phase=a.roll_phase;request.height_phase=a.height_phase;
        request.leveling=a.leveling;request.heading_hold=a.heading_hold;request.hold_protection=a.hold_protection;request.heading_term_limit=heading_limit;
        s.motion=controller.step(in,request);a.roll_phase=request.roll_phase;a.height_phase=request.height_phase;
        if(!s.motion.valid){actions.requestMode(252,true);stop(StopReason::InvalidControl,true);result("invalid control output; stop latched");}
        else {
            const auto decision=runPolicy(us,true);
            if(decision.stop){actions.requestMode(252,true);stop(StopReason::Policy,true);result("RL output unavailable; stop latched");s.control_timing.end(micros());return;}
#if RIDER2_RL_ONLY
            if(!decision.applied){actions.requestMode(252,true);stop(StopReason::Policy,true);result("RL output not applied; stop latched");s.control_timing.end(micros());return;}
#endif
            if(!driving) {
                // An old urgent stop/torque-off sequence must drain completely
                // before a new torque-enable command and motion mailbox exist.
                if(bus.writesPending()) {s.control_timing.end(micros());return;}
                if(!bus.setLegTorque(true)) {
                    actions.requestMode(252,true);stop(StopReason::Actuators,true);
                    result("leg torque enable queue failed; stop latched");
                    s.control_timing.end(micros());return;
                }
            }
            {
                rider2::lift::Input lift_in;
                lift_in.speed11=bus.sample(11).filtered_speed_mm_s;lift_in.speed21=bus.sample(21).filtered_speed_mm_s;
                lift_in.pitch_deg=s.imu.pitch_deg;lift_in.roll_deg=s.imu.roll_deg;
                lift_in.pitch_rate_deg_s=s.imu.pitch_rate_rad_s*57.2957795f;lift_in.roll_rate_deg_s=s.imu.gyro_roll_filtered;
                lift_state=rider2::lift::update(lift_state,lift_config,lift_in);
            }
            float wheel11=float(decision.wheel11),wheel21=float(decision.wheel21);
            wheel11*=wheel_gain;wheel21*=wheel_gain;
            const uint32_t filter_dt=uint32_t(us-last_filter_us);last_filter_us=us;
            if(!driving)wheel_filter_init=false;
            if(wheel_smooth_ms&&filter_dt<50000) {
                const float alpha=float(filter_dt)/(float(wheel_smooth_ms)*1000.0f+float(filter_dt));
                if(!wheel_filter_init){wheel_filtered[0]=wheel11;wheel_filtered[1]=wheel21;wheel_filter_init=true;}
                else {wheel_filtered[0]+=(wheel11-wheel_filtered[0])*alpha;wheel_filtered[1]+=(wheel21-wheel_filtered[1])*alpha;}
                wheel11=wheel_filtered[0];wheel21=wheel_filtered[1];
            } else {wheel_filtered[0]=wheel11;wheel_filtered[1]=wheel21;}
            float forward_command=(wheel11+wheel21)*.5f;
            float yaw_command=(wheel11-wheel21)*.5f;
            if(!driving||!yaw_filter_init){yaw_command_filtered=yaw_command;yaw_filter_init=true;}
            else {
                const float yaw_delta=yaw_slew_per_second*float(filter_dt)*1e-6f;
                yaw_command_filtered+=std::max(-yaw_delta,std::min(yaw_delta,yaw_command-yaw_command_filtered));
            }
            wheel11=forward_command+yaw_command_filtered;wheel21=forward_command-yaw_command_filtered;
            wheel11=std::max(-1023.0f,std::min(1023.0f,wheel11));
            wheel21=std::max(-1023.0f,std::min(1023.0f,wheel21));
            s.motion.wheels.id11=int16_t(std::lround(wheel11));s.motion.wheels.id21=int16_t(std::lround(wheel21));
            // Wheels free in the air: the speed feedback has no ground to push against and runs away (see lift_guard.h).
            // Hold the wheel command at zero and keep the policy's history clean so nothing winds up meanwhile.
            lift_acting=lift_guard_enabled&&lift_state.active;
            if(lift_acting){s.motion.wheels.id11=0;s.motion.wheels.id21=0;policy.resetHistory();}
#if RIDER2_RL_ONLY
            // Apply the actor's two leg-extension offsets to the position
            // controller's targets before the bus write.
            rider::control::offsetLegExtension(s.motion.legs,decision.leg12_m,decision.leg22_m,
                s.calibration.zero_a,s.calibration.zero_b);
#endif
            driving=true;stop_sent=false;s.reason=StopReason::None;tilt_fault_stop=false;
            bus.setOutputs(s.motion.wheels.id11,s.motion.wheels.id21,s.motion.legs.servo_a,s.motion.legs.servo_b);
            if(decision.applied)++s.rl_output_ticks;
            traceTick();
        }
        s.control_timing.end(micros());
    }
    void publish() {
        s.ms=millis();s.driving=driving;s.manual_stop=manual_stop;s.leg_calibrating=leg_calibrating;s.imu_valid=imu_valid;
        s.pitch_calibrating=trim_stage;s.mode=actions.state().mode;s.action=actions.state().action_id;
        s.battery=board.state();s.bus=bus.stats();for(size_t n=0;n<4;++n)s.actuators[n]=bus.sample(kIds[n]);
        s.policy=policy.status();s.lift_active=lift_state.active;s.lift_acting=lift_acting;s.lift_trips=lift_state.trips;s.fall_low=fall_state.low;s.fall_falling=fall_state.falling;s.fall_trips=fall_state.trips;s.fall_deep_trips=fall_deep_state.trips;s.fall_min_g=fall_state.min_g;s.fall_longest_low_ms=fall_state.longest_low_ms;
        auto& t=s.telemetry;t.mode=s.mode;t.battery_percent=s.battery.percent;
        t.pitch_deg=s.imu.pitch_deg;t.roll_deg=s.imu.roll_deg;t.yaw_deg=s.imu.yaw_deg;
        for(size_t n=0;n<3;++n)t.accel[n]=s.imu.acceleration[n];
        t.odometry[0]=odometry.x;t.odometry[1]=odometry.y;t.odometry[2]=odometry.heading;t.odometry[3]=odometry.distance;
        t.leg12_position=bus.sample(12).position_ticks;t.leg22_position=bus.sample(22).position_ticks;
        t.leg12_zero=s.calibration.zero_a;t.leg22_zero=s.calibration.zero_b;
        t.policy_mode=uint8_t(s.policy.mode);t.policy_model_id=policy.model().id;
        t.policy_trained=s.policy.trained;t.policy_valid=s.policy.model_valid;
        t.policy_action_space=uint8_t(policy.model().action_space);
        t.policy_contract=rl::kContractVersion;
#if RIDER2_RL_ONLY
        t.controller_kind=rider_rl_only_build==0x524c4f31?1:255;
#endif
        t.wheel_source=driving?(policy.status().mode==rl::Mode::Active?2:1):0;
        t.last_stop_reason=uint8_t(s.last_stop_reason);t.rl_output_ticks=s.rl_output_ticks;
        t.policy_calls=s.policy.calls;t.policy_failures=s.policy.failures;
        portENTER_CRITICAL(&stateLock);shared=s;portEXIT_CRITICAL(&stateLock);
    }
    void run() {
        begin();
        for(;;) {
            bus.poll(millis());events();actions.poll(millis());
            uint32_t us=micros();if(uint32_t(us-last_imu_us)>=kImuPeriodUs){last_imu_us=us;sensor(us);}
            us=micros();if(uint32_t(us-last_control_us)>=kControlPeriodUs)control(us);
            persistence();bus.poll(millis());
            const uint32_t now=millis();
            rider::board::LedControl led;led.robot_mode=actions.state().mode;led.pitch_trim_stage=trim_stage;
            led.low_blink=board.state().low_blink;led.height_period=actions.state().height_phase;led.roll_period=actions.state().roll_phase;
            led.height_request=s.motion.command.height;led.roll_request=s.motion.command.roll;
            led.forward_request=s.motion.command.forward;led.turn_request=s.motion.command.turn;
            led.fault=ledFaultFor(driving,s.last_stop_reason);   // the servos-released indication (red/amber) while stopped after a fault
            board.updateLeds(led,now);
            if(uint32_t(now-last_publish_ms)>=10){last_publish_ms=now;publish();}
            vTaskDelay(1);
        }
    }
};
void Robot::loadExtendedTrim() {
    // Implemented through device_settings, preserving legacy fallback.
    const auto trim=rider2::loadPitchTrim();
    s.calibration.pitch_trim_valid=trim.valid;
    if(trim.valid)s.calibration.pitch_trim=trim.value;
}
Robot& robotInstance(){static Robot robot;return robot;}
void robotTask(void*){robotInstance().run();}

void route(Source source,const uint8_t* bytes,size_t size) {
    if(!size)return;
    if(source==Source::Usb)Serial.write(bytes,size);
    else if(source==Source::Aux)commandSerial.write(bytes,size);
    else rider2::notifyBle(bytes,size);
}
void receive(Source source,uint8_t byte) {
    CommandEvent c;
    if(!parser.feed(source,byte,millis(),c))return;
    if(c.isRead()) {
        const auto s=snapshot();uint8_t out[64];const size_t n=buildReadResponse(c,s.telemetry,out,sizeof out);route(source,out,n);
    } else {Event event;event.command=c;enqueue(event);}
}
void printStatus() {
    const auto s=snapshot();const uint32_t now=millis();
    // JSON has no NaN/Infinity tokens. Sanitize presentation only; the runtime
    // guard still sees and rejects the original nonfinite control values.
    const auto printable=[](double value){return std::isfinite(value)?value:0.0;};
    Serial.printf("{\"firmware\":\"Rider2-recovered-full\",\"ms\":%u,\"snapshot_age_ms\":%u,\"mode\":%u,\"driving\":%s,\"reason\":\"%s\",\"action\":%u,\"manual_stop\":%s,\"pitch\":%.4f,\"roll\":%.4f,\"yaw\":%.4f,\"imu_valid\":%s,\"imu_age_ms\":%u,\"imu_error\":\"%s\",\"imu_calibrated\":%s,\"battery_percent\":%.2f,\"battery_voltage\":%.3f,\"zeros\":[%u,%u],\"calibration_valid\":%s,\"pitch_trim\":%.8f,\"leg_calibrating\":%s,\"pitch_calibrating\":%u,\"wheel_command\":[%d,%d],\"legs\":[%u,%u],\"physical_distance_m\":%.5f,\"feedback\":[",
        now,uint32_t(now-s.ms),s.mode,s.driving&&uint32_t(now-s.ms)<30?"true":"false",stopReasonName(s.reason),s.action,s.manual_stop?"true":"false",
        printable(s.imu.pitch_deg),printable(s.imu.roll_deg),printable(s.imu.yaw_deg),s.imu_valid&&uint32_t(now-s.imu_ms)<=30?"true":"false",uint32_t(now-s.imu_ms),rider2::imuErrorName(s.imu_diag.error),s.imu.calibrated?"true":"false",
        printable(s.battery.percent),printable(s.battery.voltage),s.calibration.zero_a,s.calibration.zero_b,s.calibration.marker_valid&&s.calibration.zeros_valid?"true":"false",
        printable(s.calibration.pitch_trim),s.leg_calibrating?"true":"false",s.pitch_calibrating,s.motion.wheels.id11,s.motion.wheels.id21,
        s.motion.legs.servo_a,s.motion.legs.servo_b,printable(s.motion.physical_position));
    for(size_t n=0;n<4;++n){const auto& a=s.actuators[n];if(n)Serial.print(',');Serial.printf("{\"id\":%u,\"age\":%u,\"speed\":%.3f,\"position\":%d,\"status\":%u,\"load\":%d,\"raw_speed\":%d}",kIds[n],a.ageMs(now),printable(a.filtered_speed_mm_s),a.position_ticks,a.last_status,int(a.load_raw),int(a.raw_speed));}
    Serial.printf("],\"bus_timeouts\":%u,\"bus_checksum\":%u,\"bus_status\":%u,\"bus_rx_bytes\":%u,\"bus_length_errors\":%u,\"bus_position_errors\":%u,\"bus_unexpected_frames\":%u,\"bus_extended_frames\":%u,\"writes\":%u,\"control_count\":%u,\"control_min_us\":%u,\"control_max_us\":%u,\"control_mean_us\":%u,\"ble_connected\":%s,\"result\":\"%s\"",
        s.bus.response_timeouts,s.bus.checksum_errors,s.bus.status_errors,s.bus.rx_bytes,
        s.bus.length_errors,s.bus.position_errors,s.bus.unexpected_frames,s.bus.valid_extended_frames,
        s.bus.writes_sent,s.control_timing.count,
        s.control_timing.count>1?s.control_timing.min_us:0,s.control_timing.max_us,
        s.control_timing.count>1?uint32_t(s.control_timing.sum_us/(s.control_timing.count-1)):0,rider2::bleConnected()?"true":"false",s.last_result);
    const auto& p=s.policy;const auto& model=rl::weights::compiledModel();
    const bool rl_only=s.telemetry.controller_kind==1;
    const char* wheel_source=s.driving&&uint32_t(now-s.ms)<30?(s.telemetry.wheel_source==2?"rl":"pid"):"stopped";
    Serial.printf(",\"controller\":\"%s\",\"wheel_source\":\"%s\",\"legacy_wheel_pid\":%s,\"rl_output_ticks\":%u,\"last_stop_reason\":\"%s\",\"speed_cal\":%.3f,\"accel\":[%.3f,%.3f,%.3f],\"build\":\"%s\"",
        rl_only?"rl_only":"mixed",wheel_source,rl_only?"false":"true",s.rl_output_ticks,stopReasonName(s.last_stop_reason),printable(speed_cal),
        printable(s.imu.acceleration[0]),printable(s.imu.acceleration[1]),printable(s.imu.acceleration[2]),kBuildId);
    {
        const rider2::lift::Config lc;
        Serial.printf(",\"lift_guard\":{\"enabled\":%s,\"active\":%s,\"acting\":%s,\"trips\":%u,\"fast_mm_s\":%.0f,\"fast_ticks\":%u,\"calm_ticks\":%u}",
            lift_guard_enabled?"true":"false",s.lift_active?"true":"false",s.lift_acting?"true":"false",s.lift_trips,
            printable(lc.fast_speed_mm_s),unsigned(lc.fast_ticks),unsigned(lc.calm_ticks));
    }
    {
        const rider2::fall::Config fc;
        Serial.printf(",\"fall_guard\":{\"enabled\":%s,\"falling\":%s,\"low\":%s,\"trips\":%u,\"deep_trips\":%u,\"freefall_g\":%.2f,\"freefall_ms\":%.0f,\"min_g\":%.3f,\"longest_low_ms\":%.0f}",
            fall_guard_enabled?"true":"false",s.fall_falling?"true":"false",s.fall_low?"true":"false",s.fall_trips,s.fall_deep_trips,
            printable(fc.freefall_g),printable(fall_guard_ms),printable(s.fall_min_g),printable(s.fall_longest_low_ms));
        // how often and how fast the IMU is read (the interval between sensor() calls), how long one read+fusion takes, and what the chip returned
        Serial.printf(",\"imu_timing\":{\"count\":%u,\"min_us\":%u,\"mean_us\":%u,\"max_us\":%u,\"max_work_us\":%u,\"read_failures\":%u,\"blank\":%u,\"identical\":%u}",
            unsigned(s.imu_timing.count),s.imu_timing.count>1?unsigned(s.imu_timing.min_us):0u,s.imu_timing.count>1?unsigned(s.imu_timing.sum_us/(s.imu_timing.count-1)):0u,
            unsigned(s.imu_timing.max_us),unsigned(s.imu_timing.max_work_us),unsigned(s.imu_diag.read_failures),unsigned(s.imu_diag.blank_samples),unsigned(s.imu_diag.identical_samples));
    }
    Serial.printf(",\"dzcomp\":{\"max\":%.1f,\"hysteresis\":%.1f,\"ramp\":%.2f,\"initial\":%.1f,\"grow_mean\":%.1f,\"preload\":%.2f,\"sign\":%d,\"flip_rate\":%.2f,\"trips\":%u,\"locked\":%s}",
        printable(dz_max_units),printable(dz_hysteresis),printable(dz_ramp),printable(dz_initial),printable(dz_grow_mean),printable(s.policy.dz_preload),int(s.policy.dz_sign),
        printable(s.policy.dz_flip_rate),s.policy.dz_trips,s.policy.dz_locked?"true":"false");
    Serial.printf(",\"policy\":{\"mode\":\"%s\",\"error\":\"%s\",\"model\":\"%s\",\"model_id\":%u,\"trained\":%s,\"model_valid\":%s,\"action_space\":\"%s\",\"action_scale\":%.3f,\"calls\":%u,\"failures\":%u,\"last_us\":%u,\"max_us\":%u,\"last_interval_us\":%u,\"action\":[%.6f,%.6f],\"command\":[%.3f,%.3f],\"yaw_integral\":%.6f,\"yaw_integral_gain\":%.6f,\"observation\":[",
        rl::modeName(p.mode),rl::errorName(p.error),p.model_valid?model.name:"invalid",model.id,
        p.trained?"true":"false",p.model_valid?"true":"false",rl::actionSpaceName(model.action_space),printable(model.action_scale),
        p.calls,p.failures,p.last_us,p.max_us,p.last_interval_us,
        printable(p.action[0]),printable(p.action[1]),printable(p.command[0]),printable(p.command[1]),
        printable(p.yaw_integral),printable(model.yaw_integral_gain));
    for(unsigned n=0;n<rl::kInputs;++n){if(n)Serial.print(',');Serial.printf("%.7f",printable(p.observation[n]));}
    Serial.println("]}}");
}
void protocolCommand(CommandKind kind,const uint8_t* data,size_t length) {
    Event e;e.command.source=Source::Usb;e.command.reg=uint8_t(kind);e.command.length=uint8_t(length);
    if(length>sizeof e.command.data)return;
    std::memcpy(e.command.data,data,length);
    if(!validCommand(e.command)||!enqueue(e))Serial.println("ERR command rejected or queue full");
}
void valueCommand(CommandKind kind,int value){if(value<0||value>255){Serial.println("ERR expected byte 0..255");return;}uint8_t v=uint8_t(value);protocolCommand(kind,&v,1);}
void console(const char* line) {
    if(!strcmp(line,"!status")){printStatus();return;}
    if(!strcmp(line,"!policy status")){printStatus();return;}
    if(!strcmp(line,"!policy off")){valueCommand(CommandKind::PolicyMode,0);return;}
    if(!strcmp(line,"!policy shadow")){valueCommand(CommandKind::PolicyMode,1);return;}
    if(!strcmp(line,"!policy active")){valueCommand(CommandKind::PolicyMode,2);return;}
    if(!strcmp(line,"!stop")){emergencyStop();Serial.println("stop requested");return;}
    Event e;
    if(!strcmp(line,"!arm")){e.kind=EventKind::Arm;enqueue(e);return;}
    if(!strcmp(line,"!neutral")){e.kind=EventKind::Neutral;enqueue(e);return;}
    if(!strcmp(line,"!odom reset")){e.kind=EventKind::ResetOdometry;enqueue(e);return;}
    if(!strcmp(line,"!cal begin")){valueCommand(CommandKind::LegCalibration,1);return;}
    if(!strcmp(line,"!cal save")){valueCommand(CommandKind::LegCalibration,0);return;}
    if(!strcmp(line,"!trim")){valueCommand(CommandKind::PitchCalibration,1);return;}
    if(!strcmp(line,"!trace on")){trace_stream_ble=true;Serial.println("trace streaming enabled");return;}
    if(!strcmp(line,"!trace off")){trace_stream_ble=false;Serial.println("trace streaming disabled");return;}
    // USB text export of the same ring (plus servo load, wheel positions and the accelerometer); serviced from loop().
    if(!strcmp(line,"!trace dump")){trace_usb.dump(trace_head,kTraceCapacity);Serial.println(rider2::trace::kCsvHeader);return;}
    if(!strcmp(line,"!trace usb on")){trace_usb.stream(trace_head);Serial.println(rider2::trace::kCsvHeader);return;}
    if(!strcmp(line,"!trace usb off")){trace_usb.stop();Serial.println("# trace usb off");return;}
    if(!strcmp(line,"!fallguard on")){fall_guard_enabled=true;Serial.printf("fall guard on: free fall (below 0.5 g for %.0f ms) stops the robot and releases the servos before the impact\n",double(fall_guard_ms));return;}
    if(!strcmp(line,"!fallguard clear")){fall_clear_request=true;Serial.println("fall guard statistics and trip counters cleared");return;}
    if(!strcmp(line,"!fallguard off")){fall_guard_enabled=false;Serial.println("fall guard off (the detector keeps recording)");return;}
    if(!strcmp(line,"!fallguard status")){const auto snap=snapshot();Serial.printf("fall guard %s, hold %.0f ms, falling %s, trips %u\n",fall_guard_enabled?"on":"off",double(fall_guard_ms),snap.fall_falling?"yes":"no",snap.fall_trips);return;}
    {   // !fallguard ms [N]: the hold time (continuous ms below 0.5 g); RAM only, back to the default at every boot
        float hold=fall_guard_ms;const int parsed=rider2::fall::parseHoldCommand(line,hold);
        if(parsed==1){Serial.printf("fall guard hold %.0f ms (allowed %.0f..%.0f, default %.0f)\n",double(fall_guard_ms),double(rider2::fall::kMinHoldMs),double(rider2::fall::kMaxHoldMs),double(rider2::fall::Config().freefall_ms));return;}
        if(parsed==2){fall_guard_ms=hold;Serial.printf("fall guard hold %.0f ms (RAM only; %.0f again after a reset)\n",double(hold),double(rider2::fall::Config().freefall_ms));return;}
        if(parsed<0){Serial.printf("ERR !fallguard ms N, N = %.0f..%.0f\n",double(rider2::fall::kMinHoldMs),double(rider2::fall::kMaxHoldMs));return;}
    }
    if(!strcmp(line,"!liftguard on")){lift_guard_enabled=true;Serial.println("lift guard on: the wheel command is held at zero while the wheels have run away in the air");return;}
    if(!strcmp(line,"!liftguard off")){lift_guard_enabled=false;Serial.println("lift guard off (the detector keeps recording)");return;}
    if(!strcmp(line,"!liftguard status")){const auto snap=snapshot();Serial.printf("lift guard %s, detector %s, acting %s, trips %u\n",lift_guard_enabled?"on":"off",snap.lift_active?"active":"idle",snap.lift_acting?"yes":"no",snap.lift_trips);return;}
    // Dead-zone preload (src/policy/dead_zone_comp.h).  Settings are applied by the control task; what was learned survives !dzcomp off.
    if(!strcmp(line,"!dzcomp")||!strcmp(line,"!dzcomp status")){
        const auto snap=snapshot();const auto& p=snap.policy;
        Serial.printf("dead-zone preload %s: max %.1f, hysteresis %.1f, ramp %.2f/s, start %.1f, grows at mean |command| >= %.1f | now %.2f (sign %d), flip rate %.2f/s, chatter trips %u%s\n",
            dz_max_units>0?"on":"off",double(dz_max_units),double(dz_hysteresis),double(dz_ramp),double(dz_initial),double(dz_grow_mean),double(p.dz_preload),int(p.dz_sign),
            double(p.dz_flip_rate),unsigned(p.dz_trips),p.dz_locked?" (locked)":"");
        return;
    }
    if(!strcmp(line,"!dzcomp off")){dz_max_units=0;Serial.println("dead-zone preload off (the learned value is kept; !dzcomp on resumes)");return;}
    if(!strcmp(line,"!dzcomp on")){dz_max_units=rl::kDeadZoneDefaultMaxUnits;dz_hysteresis=8.0f;dz_ramp=rl::kDeadZoneDefaultRamp;dz_initial=rl::kDeadZoneDefaultInitial;dz_grow_mean=rl::kDeadZoneDefaultGrowMean;Serial.println("dead-zone preload on (defaults)");return;}
    if(!strcmp(line,"!dzcomp reset")){++dz_reset_request;Serial.println("dead-zone preload forgotten; it learns again while the robot stands quietly");return;}
    {
        float mx=0,hy=dz_hysteresis,rp=dz_ramp,ini=dz_initial,gm=dz_grow_mean;
        const int n=sscanf(line,"!dzcomp %f %f %f %f %f",&mx,&hy,&rp,&ini,&gm);
        if(n>=1){
            if(mx>=0&&mx<=rl::kMaxDeadZoneUnits&&hy>=2&&hy<=40&&rp>=0&&rp<=20&&ini>=0&&ini<=rl::kMaxDeadZoneUnits&&gm>=0&&gm<=60){
                dz_max_units=mx;dz_hysteresis=hy;dz_ramp=rp;dz_initial=ini;dz_grow_mean=gm;
                Serial.printf("dead-zone preload max %.1f, hysteresis %.1f, ramp %.2f/s, start %.1f, grows while mean |command| >= %.1f (start applies after !dzcomp reset)\n",double(mx),double(hy),double(rp),double(ini),double(gm));
            }
            else Serial.printf("ERR dzcomp MAX 0..%.0f [HYSTERESIS 2..40 [RAMP 0..20 [START 0..%.0f [GROW 0..60]]]]\n",double(rl::kMaxDeadZoneUnits),double(rl::kMaxDeadZoneUnits));
            return;
        }
    }
    {float g=0;if(sscanf(line,"!gain %f",&g)==1){wheel_gain=g;Serial.printf("wheel gain %.3f\n",g);return;}}
    {float g=0;if(sscanf(line,"!speedcal %f",&g)==1){if(g>=.2f&&g<=4.f&&!shared.driving){speed_cal=g;Serial.printf("policy speed scale %.3f\n",g);}else Serial.println("ERR speedcal 0.2..4.0 and standstill");return;}}
    {int ms=0;if(sscanf(line,"!smooth %d",&ms)==1&&ms>=0){wheel_smooth_ms=unsigned(ms);Serial.printf("wheel smoothing %d ms\n",ms);return;}}
    if(!strncmp(line,"!name ",6)){protocolCommand(CommandKind::BleSuffix,reinterpret_cast<const uint8_t*>(line+6),strlen(line+6));return;}
    char cmd[20]={};int a=0,b=0,c=0,d=0;
    const int count=sscanf(line,"%19s %d %d %d %d",cmd,&a,&b,&c,&d);
    if(count==3&&!strcmp(cmd,"!move")){if(a<0||a>255||b<0||b>255){Serial.println("ERR expected bytes 0..255");return;}valueCommand(CommandKind::Forward,a);valueCommand(CommandKind::Turn,b);return;}
    if(count==5&&!strcmp(cmd,"!rgb")) {if(a<0||a>3||b<0||b>255||c<0||c>255||d<0||d>255){Serial.println("ERR RGB index0..3/channel0..255");return;}uint8_t rgb[]={uint8_t(b),uint8_t(c),uint8_t(d)};protocolCommand(CommandKind(uint8_t(CommandKind::Rgb0)+a),rgb,3);return;}
    if(count==2){
        const struct{const char* text;CommandKind kind;} maps[]={
            {"!action",CommandKind::Action},{"!loop",CommandKind::ActionLoop},{"!height",CommandKind::Height},
            {"!roll",CommandKind::Roll},{"!level",CommandKind::Leveling},{"!heading",CommandKind::HeadingHold},
            {"!rollwave",CommandKind::RollWave},{"!heightwave",CommandKind::HeightWave},
            {"!maxforward",CommandKind::MaxForward},{"!maxturn",CommandKind::MaxTurn}};
        for(const auto& m:maps)if(!strcmp(cmd,m.text)){valueCommand(m.kind,a);return;}
    }
    Serial.println("!status | !policy off/shadow/active/status | !stop | !arm | !neutral | !move FORWARD TURN (bytes,128 neutral) | !height BYTE | !roll BYTE | !action 0..6/255 | !loop 0/1 | !level 0/1 | !heading 0/1 | !heightwave BYTE | !rollwave BYTE | !maxforward BYTE | !maxturn BYTE | !rgb INDEX R G B | !cal begin | !cal save | !trim | !name SUFFIX | !odom reset | !trace dump | !trace usb on/off | !liftguard on/off/status | !fallguard on/off/status/clear/ms N | !dzcomp [off|on|reset|MAX [HYST [RAMP [START [GROW]]]]]");
}
}
void setup() {
    Serial.begin(115200);commandSerial.begin(115200,SERIAL_8N1,4,5);
    if(!robotInstance().prepareMotorBus()){
        Serial.println("FATAL startup motor stop did not drain reliably; control task disabled");return;
    }
    commands=xQueueCreate(24,sizeof(Event));
    if(!commands){Serial.println("FATAL command queue allocation; control task disabled");return;}
    // BLE initialization must finish before starting the time-critical task.
    const auto name=rider2::readBleName();bleReady=rider2::beginBle(name.full_name);
    Serial.printf("Rider2 application; wheel controller=%s; BLE=%s name=%s. !help for console.\n",
#if RIDER2_RL_ONLY
        "RL only; leg position control retained",
#else
        "mixed legacy/policy",
#endif
        bleReady?"ready":"failed",name.full_name);
    robotTaskStarted=xTaskCreatePinnedToCore(robotTask,"rider_control",16384,nullptr,3,nullptr,1)==pdPASS;
    if(!robotTaskStarted)Serial.println("FATAL robot task creation; control task disabled");
}
void loop() {
    // If startup failed, retry only pending safety packets. No control task
    // exists, so this remains the bus's sole owner and cannot enable motion.
    if(!robotTaskStarted)robotInstance().bus.poll(millis());
    static char line[96];static size_t used=0;static bool lineMode=false,overflow=false;
    unsigned budget=256;
    while(Serial.available()&&budget--) {
        parser.expire(Source::Usb,millis());
        const uint8_t b=uint8_t(Serial.read());
        if(!lineMode&&b=='!'&&!parser.receiving(Source::Usb)){lineMode=true;used=0;overflow=false;}
        if(lineMode){
            if(b=='\r'||b=='\n'){if(!overflow){line[used]=0;console(line);}else Serial.println("ERR line too long");lineMode=false;used=0;}
            else if(used<sizeof(line)-1)line[used++]=char(b);else overflow=true;
        }else receive(Source::Usb,b);
    }
    budget=256;while(commandSerial.available()&&budget--)receive(Source::Aux,uint8_t(commandSerial.read()));
    static bool previousBle=false;static uint32_t previousDrops=0;
    const bool connected=rider2::bleConnected();const uint32_t dropped=rider2::bleDroppedBytes();
    if(previousBle!=connected||previousDrops!=dropped){parser.reset(Source::Ble);if(previousBle&&!connected){Event e;e.kind=EventKind::Neutral;enqueue(e);}previousBle=connected;previousDrops=dropped;}
    uint8_t ble[64];const size_t n=rider2::readBle(ble,sizeof ble);for(size_t i=0;i<n;++i)receive(Source::Ble,ble[i]);
    rider2::pollBle();
    static uint32_t lastStream=0;const uint32_t now=millis();
    // Trace streaming is rate-limited here, outside the control loop. Two
    // 28-byte samples plus a sequence header fit in one notify payload.
    static uint32_t lastTraceNotify=0,traceSent=0;
    if(trace_stream_ble&&rider2::bleConnected()&&uint32_t(now-lastTraceNotify)>=20) {
        lastTraceNotify=now;
        if(trace_head-traceSent>kTraceCapacity)traceSent=trace_head-kTraceCapacity;
        const uint32_t available=trace_head-traceSent;
        if(available) {
            uint8_t packet[64];packet[0]=0x54;packet[1]=uint8_t(traceSent);
            packet[2]=uint8_t(traceSent>>8);packet[3]=uint8_t(traceSent>>16);
            const size_t count=available>=2?2:1;
            std::memcpy(packet+4,&trace_samples[traceSent%kTraceCapacity],count*sizeof(TraceSample));
            if(rider2::notifyBle(packet,4+count*sizeof(TraceSample)))traceSent+=count;
        }
    }
    // Events while the USB trace is on: one text line whenever the driving state, the stop reason or the lift guard
    // changes, so a gap in the trace can be explained afterwards.
    {
        static uint32_t lastEventCheck=0;static bool seen=false,wasDriving=false,wasLift=false;
        static StopReason wasReason=StopReason::Startup,wasLast=StopReason::Startup;
        if(trace_usb.mode&&uint32_t(now-lastEventCheck)>=4) {
            lastEventCheck=now;const auto snap=snapshot();
            if(!seen||snap.driving!=wasDriving||snap.reason!=wasReason||snap.last_stop_reason!=wasLast||snap.lift_active!=wasLift) {
                seen=true;wasDriving=snap.driving;wasReason=snap.reason;wasLast=snap.last_stop_reason;wasLift=snap.lift_active;
                Serial.printf("# ev ms=%u drv=%d reason=%s last_stop=%s lift=%d acting=%d trips=%u bus_to=%u bus_st=%u bus_len=%u bus_chk=%u\n",
                    snap.ms,snap.driving?1:0,stopReasonName(snap.reason),stopReasonName(snap.last_stop_reason),snap.lift_active?1:0,
                    snap.lift_acting?1:0,snap.lift_trips,snap.bus.response_timeouts,snap.bus.status_errors,snap.bus.length_errors,
                    snap.bus.checksum_errors);
            }
        }
    }
    // USB trace export: a few text lines per pass, so a console command (!stop) is never held up for long.
    {
        uint32_t index=0;char text[256];
        for(int lines=0;lines<3&&trace_usb.take(trace_head,kTraceCapacity,index);++lines) {
            const size_t n=rider2::trace::formatLine(text,sizeof text,index,trace_samples[index%kTraceCapacity],trace_extra[index%kTraceCapacity]);
            if(n)Serial.write(reinterpret_cast<const uint8_t*>(text),n);
        }
        if(trace_usb.finished){trace_usb.finished=false;Serial.printf("# end %u\n",trace_usb.sent);}
    }
    if(uint32_t(now-lastStream)>20){lastStream=now;const auto s=snapshot();if(s.stream&&s.stream_source!=Source::Ble&&uint32_t(now-s.ms)<50){uint8_t out[48];route(s.stream_source,out,buildStreamTelemetry(s.telemetry,out,sizeof out));}}
    delay(1);
}
