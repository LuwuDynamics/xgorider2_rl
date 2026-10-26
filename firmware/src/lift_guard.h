#pragma once
// "Picked up" guard for the learned wheel controller.
//
// Why it exists: the policy (like any state-feedback balance controller) feeds the wheel speed back with the same sign
// as the lean.  On the ground the robot's own dynamics close that loop stably; with the wheels in the air there is no
// ground reaction, the loop is open and unstable, and a small disturbance (the user tilting the robot while lifting it,
// or the command creeping past the wheel's dead zone) makes the wheels run away to full speed within ~0.2 s.  Measured
// on the robot 2026-10-22: wheel speed 0 -> 540 mm/s (register units, ~2 m/s true), command saturated at +-1023, the
// body barely moves (pitch +-4 deg, no horizontal acceleration).  A standing or pushed robot stays far below that: the
// mean of the two wheel speeds peaked at 222 in hard pushes and 56 under phone control.
//
// What it does: the mean (or the difference) of the two filtered wheel speeds above `fast_speed` for `fast_ticks` control
// ticks in a row means "the wheels are free".  The caller then holds the wheel command at zero (legs stay powered,
// nothing else changes) until the robot is upright, not being moved by hand (small body rates) and the wheels are still
// for `calm_ticks` -- the same picture as a robot standing on the floor.  If it trips again within `retrip_window_ticks`
// of being released it is still in the air, and the next release waits `retrip_calm_ticks`.  Dead time matters: a hand
// that lets go while the wheels are still held off drops the robot.  It is a safety guard on
// the output, not a second controller.
//
// Header-only and free of hardware calls so tests/test_lift_guard.cpp can drive it on the host.
#include <cmath>
#include <cstdint>

namespace rider2 { namespace lift {

struct Config {
    float fast_speed_mm_s = 300.0f;      // |mean| or |half difference| of the filtered wheel speeds (register units)
    std::uint16_t fast_ticks = 6;        // consecutive control ticks above it (about 30 ms at the 5 ms control period)
    float calm_speed_mm_s = 30.0f;       // "still": both measures below this ...
    float calm_rate_deg_s = 8.0f;        // ... the body is not being moved by hand (pitch and roll rates) ...
    float upright_pitch_deg = 8.0f;      // ... and upright, as for the normal start
    float upright_roll_deg = 5.0f;
    std::uint16_t calm_ticks = 60;       // about 0.3 s of that before the wheels are released again
    std::uint16_t retrip_window_ticks = 200;   // tripping again within about 1 s of a release: still in the air
    std::uint16_t retrip_calm_ticks = 100;     // so the next release waits about 0.5 s (a longer wait let a robot fall when the
                                               // hand let go just before it ran out -- measured 2026-10-22)
};

struct State {
    bool active = false;                 // the wheels have run away: hold their command at zero
    bool fast = false;                   // this tick is above the speed limit (for the trace)
    std::uint16_t fast_count = 0, calm_count = 0, calm_needed = 60;
    std::uint16_t since_release = 65535; // ticks since the last release (saturating)
    std::uint32_t trips = 0;
};

struct Input {
    float speed11 = 0, speed21 = 0;      // filtered wheel speeds, register units (same sign for the same direction)
    float pitch_deg = 0, roll_deg = 0;
    float pitch_rate_deg_s = 0, roll_rate_deg_s = 0;
};

inline std::uint16_t saturatingIncrement(std::uint16_t value) { return value < 65535 ? std::uint16_t(value + 1) : value; }

inline State update(State s, const Config& c, const Input& in) {
    const float common = 0.5f * (in.speed11 + in.speed21), difference = 0.5f * (in.speed11 - in.speed21);
    const float speed = std::fmax(std::fabs(common), std::fabs(difference));
    const bool finite = std::isfinite(speed) && std::isfinite(in.pitch_deg) && std::isfinite(in.roll_deg) &&
                        std::isfinite(in.pitch_rate_deg_s) && std::isfinite(in.roll_rate_deg_s);
    s.fast = finite && speed > c.fast_speed_mm_s;
    if (!s.active) {
        s.since_release = saturatingIncrement(s.since_release);
        s.fast_count = s.fast ? saturatingIncrement(s.fast_count) : std::uint16_t(0);
        if (s.fast_count >= c.fast_ticks) {
            s.calm_needed = s.since_release <= c.retrip_window_ticks ? c.retrip_calm_ticks : c.calm_ticks;
            s.active = true;
            s.calm_count = 0;
            ++s.trips;
        }
    } else {
        const bool calm = finite && speed < c.calm_speed_mm_s && std::fabs(in.pitch_deg) < c.upright_pitch_deg &&
                          std::fabs(in.roll_deg) < c.upright_roll_deg && std::fabs(in.pitch_rate_deg_s) < c.calm_rate_deg_s &&
                          std::fabs(in.roll_rate_deg_s) < c.calm_rate_deg_s;
        s.calm_count = calm ? saturatingIncrement(s.calm_count) : std::uint16_t(0);
        if (s.calm_count >= s.calm_needed) {
            s.active = false;
            s.fast_count = 0;
            s.calm_count = 0;
            s.since_release = 0;
        }
    }
    return s;
}

}}  // namespace rider2::lift
