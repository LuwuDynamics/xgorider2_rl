#pragma once
// Free-fall guard: let go of the servos BEFORE the robot hits the ground.
//
// Why it exists: a robot that falls off the table lands with its servos powered and holding position, and a powered servo
// that is hit through its output shaft strips its gear teeth (reported 2026-10-22).  Today every protective stop releases
// the legs and zeroes the wheels (ActuatorCommandQueue::stop), but the tilt stop only fires after the landing, when the
// body has already tipped past 45 degrees.
//
// What it does: the accelerometer measures specific force, so it reads about 1 g standing, 0.7..1.8 g when the robot is
// handled or pushed, and about 0 g in free fall.  Total acceleration below `freefall_g` CONTINUOUSLY for `freefall_ms`
// means the robot is falling.  The caller then stops it like any other protective stop (wheels to zero, leg servos
// released) and the usual automatic re-arm brings it back once it stands upright and still after the landing.  Lifting
// the robot by hand does not trigger it: the acceleration stays near 1 g.
//
// Thresholds come from the recordings of 2026-10-22 (560 s of standing, pushes, hand lifts and put-downs, USB traces at
// 55 Hz): the deepest dips were 0.23 g and 0.29 g, each ONE sample (<= ~36 ms), all at contact (a push, the wheels touching
// down); nothing stayed below 0.6 g for two consecutive samples.  A table drop (80 cm) is 0.4 s of free fall: detection
// after 60 ms leaves about 0.34 s before the impact; for a 20 cm drop (0.2 s) about 0.14 s.
//
// The hold time (`freefall_ms`) defaults to 60 ms and can be changed at run time with `!fallguard ms N` (20..200 ms, RAM only:
// it is back to 60 at every boot).  80 ms was the first choice; the third robot's hand drops (2026-10-23, trace
// logs/new-robot-20261023/drop-test-1) dipped to 0.13 g for only about 60..75 ms, and a hold of 60 ms still leaves ~24 ms over the longest
// dip that is not a fall (<= ~36 ms).  Going down to 40 ms makes hand-moved drops trip (for testing) but leaves only a few ms over those dips.
//
// The accelerometer's full scale is +-2 g (1 g = 16384 counts), the existing IMU validity test (|raw| > 1000 counts, i.e.
// more than 0.06 g) only flags a dead sensor or an exact 0 g, and while it fails the IMU state is not updated -- so the
// caller must feed THIS guard from the raw sample, not from the (stale) fused state.
//
// Header-only and free of hardware calls so tests/test_fall_guard.cpp can drive it on the host.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace rider2 { namespace fall {

struct Config {
    float freefall_g = 0.5f;        // total acceleration below this ...
    float freefall_ms = 60.0f;      // ... for this long without a break means falling (`!fallguard ms N` changes it at run time)
    float landed_g = 0.8f;          // the fall is over once the acceleration is back above this ...
    float landed_ms = 40.0f;        // ... for this long
    float max_dt_ms = 30.0f;        // a longer gap between samples is a stall, not evidence: the count starts again
};

struct State {
    float min_g = 1000.0f;          // lowest usable sample since the state was created (diagnostics: how close did a drop come)
    float longest_low_ms = 0.0f;    // longest continuous stretch below `freefall_g` since then
    float low_ms = 0.0f;            // how long the acceleration has been below `freefall_g`
    float high_ms = 0.0f;           // how long it has been above `landed_g` (only used to end a fall)
    bool falling = false;           // latched from the trip until the landing
    bool low = false;               // this sample is below the limit (for the trace)
    std::uint32_t trips = 0;
};

struct Input {
    float accel_g = 1.0f;           // |a| of the raw accelerometer sample, in g
    float dt_ms = 5.0f;             // time since the previous sample
    bool fresh = true;              // a sample was actually read
};

inline State update(State s, const Config& c, const Input& in) {
    const bool usable = in.fresh && std::isfinite(in.accel_g) && std::isfinite(in.dt_ms) && in.dt_ms > 0.0f && in.dt_ms <= c.max_dt_ms;
    if (!usable) {
        s.low = false;
        s.low_ms = 0.0f;
        s.high_ms = 0.0f;
        return s;
    }
    s.low = in.accel_g < c.freefall_g;
    if (in.accel_g < s.min_g) s.min_g = in.accel_g;
    if (s.low) {
        s.low_ms += in.dt_ms;
        s.high_ms = 0.0f;
        if (s.low_ms > s.longest_low_ms) s.longest_low_ms = s.low_ms;
    } else {
        s.low_ms = 0.0f;
        s.high_ms = in.accel_g > c.landed_g ? s.high_ms + in.dt_ms : 0.0f;
    }
    if (!s.falling) {
        if (s.low_ms >= c.freefall_ms) {
            s.falling = true;
            ++s.trips;
        }
    } else if (s.high_ms >= c.landed_ms) {
        s.falling = false;
        s.high_ms = 0.0f;
    }
    return s;
}

// The second, "deep" detector.  A real free fall reads below 1000 counts (0.061 g): exactly where the IMU validity gate in main.cpp
// declares a sample unusable.  Before 2026-10-24 that made the IMU look dead and the robot stopped ~30 ms later with the reason
// "imu_missing_stale_or_nonfinite" (no LED, not re-armable); now such a fall is reported as a fall.  Same landing rule as the main detector.
inline Config deepConfig() {
    Config c;
    c.freefall_g = 1000.0f / 16384.0f;
    c.freefall_ms = 6.0f;           // two IMU reads (~3 ms apart) in a row: no single-sample glitch counts, no noticeable delay
    return c;
}

// The IMU fusion (main.cpp sensor()) restarts after an invalid read or when no valid read came for more than 30 ms.  A read that only showed ~0 g
// (the chip answered, the robot is falling) must not restart it unless that lasted > 30 ms: the restart clears the "calibrated" flag, and before 2026-10-24
// that made the guard report "startup" (no LED) a read before the fall detector had latched (third real-fall test, the fall after the capture).
inline bool imuFusionRestart(bool last_read_valid, bool last_read_low_g_only, std::uint32_t ms_since_valid) {
    return (!last_read_valid && !last_read_low_g_only) || ms_since_valid > 30;
}

// What `!fallguard ms N` accepts (ms of continuous low acceleration that count as a fall).
constexpr float kMinHoldMs = 20.0f, kMaxHoldMs = 200.0f;
inline bool validHoldMs(float ms) { return std::isfinite(ms) && ms >= kMinHoldMs && ms <= kMaxHoldMs; }

// The console line "!fallguard ms" (query) or "!fallguard ms N" (set).  Returns 0 when the line is something else, 1 for the
// query, 2 when N is a valid hold time (stored in `ms`), -1 when the number is missing, malformed, followed by text, or out of range.
inline int parseHoldCommand(const char* line, float& ms) {
    static const char kPrefix[] = "!fallguard ms";
    const std::size_t n = sizeof(kPrefix) - 1;
    if (!line || std::strncmp(line, kPrefix, n) != 0) return 0;
    const char* p = line + n;
    if (*p != '\0' && *p != ' ') return 0;
    while (*p == ' ') ++p;
    if (*p == '\0') return 1;
    char* end = nullptr;
    const float value = std::strtof(p, &end);
    if (end == p) return -1;
    while (*end == ' ') ++end;
    if (*end != '\0' || !validHoldMs(value)) return -1;
    ms = value;
    return 2;
}

}}  // namespace rider2::fall
