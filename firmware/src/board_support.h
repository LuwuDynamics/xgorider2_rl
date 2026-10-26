#pragma once
#include <array>
#include <cstdint>

namespace rider { namespace board {
constexpr int kCalibrationPin=21, kBatteryPin=33, kLedPin=27;
enum class CalibrationSwitchEvent : std::uint8_t { None, Enter, Save };
struct BoardState {
    std::uint16_t adc_raw=0;
    float voltage=0,percent=100;
    bool battery_valid=false;
    std::uint32_t battery_updates=0;
    std::uint8_t low_stage=0;
    bool low_blink=false,neutralize=false,stop_required=false,release_legs=false;
    bool calibration_pressed=false;
};
float batteryVoltage(std::uint16_t adc);
float batteryPercent(float voltage);
// Original block-average/outlier rejection and count-based low-battery state.
class BatteryMonitor {
public:
    void update(std::uint16_t adc,std::uint32_t uptime_ms);
    const BoardState& state() const { return state_; }
private:
    BoardState state_;
    float readings_[15]={};
    std::uint8_t index_=0;
    std::uint16_t low_count_=0,blink_count_=0;
};
class CalibrationSwitch {
public:
    CalibrationSwitchEvent update(bool high);
    bool pressed() const { return !stable_high_; }
private:
    // Firmware BSS starts at zero; a first high transition is harmless unless
    // the runtime has already entered calibration through another command.
    bool stable_high_=false;
    std::uint8_t mismatches_=0;
};
struct Rgb { std::uint8_t r=0,g=0,b=0; };
using LedPixels=std::array<Rgb,4>;
// RL-only addition (the original firmware has no such thing): why the servos are released.  The original leaves the LEDs
// untouched in most stopped modes, so a protective stop could pass unnoticed, or an old colour stayed on.
enum class LedFault : std::uint8_t { None, Tilt, Fall, Other, Manual };
struct LedControl {
    std::uint8_t robot_mode=0,pitch_trim_stage=0;
    bool low_blink=false;
    float height_period=0,roll_period=0,height_request=0,roll_request=0;
    float forward_request=0,turn_request=0;
    LedFault fault=LedFault::None;
};
Rgb paletteColor(std::uint8_t index);
// Returns false where the original leaves the previous LED frame untouched.
bool renderLeds(const LedControl&,std::uint32_t now_ms,const LedPixels &user,LedPixels &out);
// The servos-released indication, drawn instead of renderLeds() while `fault` is set and the robot is not balancing:
// solid red = fallen over (tilt stop), fast red flicker (5 Hz) = free fall, slow red blink (1 Hz) = another protective stop (servo or IMU
// lost, policy fault, ...), solid amber = stopped by a command.  Returns false when nothing is due (no fault, balancing, low battery).
bool renderFaultLeds(const LedControl&,std::uint32_t now_ms,LedPixels &out);
std::uint8_t brightnessScale(std::uint8_t channel); // original brightness128

#ifdef ARDUINO
class BoardSupport {
public:
    bool begin();
    void poll(std::uint32_t now_ms);
    BoardState state() const;
    CalibrationSwitchEvent takeCalibrationEvent();
    bool setUserLed(std::uint8_t index,Rgb color);
    bool setUserLed(std::uint8_t index,std::uint8_t r,std::uint8_t g,std::uint8_t b) {
        return setUserLed(index,Rgb{r,g,b});
    }
    void fillPalette(std::uint8_t first,std::uint8_t count,std::uint8_t color);
    void updateLeds(const LedControl&,std::uint32_t now_ms);
    bool ledsReady() const { return leds_ready_; }
private:
    void show(const LedPixels&);
    BatteryMonitor battery_;
    CalibrationSwitch switch_;
    CalibrationSwitchEvent event_=CalibrationSwitchEvent::None;
    LedPixels user_{},shown_{};
    std::uint32_t last_poll_ms_=0,last_led_ms_=0;
    bool started_=false,leds_ready_=false;
};
#endif
}} // namespace rider::board
