#pragma once
// Wheel dead-zone pre-load for the learned wheel controller.
//
// Why it exists: the hub servos do nothing for wheel commands below about +-45 units (measured on the robot: the wheel
// starts to turn at |command| 40-50 of +-1023; gearbox friction and the drive's start threshold).  The network was
// trained on a plant without that dead zone.  A wheeled inverted pendulum is unstable, so at small amplitudes the dead
// zone removes the loop gain the pendulum needs: the lean grows until the command clears the dead zone, then the wheel is
// kicked, overshoots and the same happens the other way.  In the simulator (with the measured dead zone) this is a
// self-sustained slow stick-slip cycle: creep for ~6 s, kick, creep back, +-65 mm, 7.5 s period, for any friction above
// ~0.002 N*m; on the robot it shows up as small steps back and forth while standing.
//
// What it does: adds a constant `preload` in the direction the controller is pushing, so that the first unit of command
// beyond the preload moves the wheel (a Schmitt-trigger sign with hysteresis: no sign flips from noise, no chatter at
// zero).  The preload is NOT a fixed number, because a preload larger than the real dead zone makes the wheels buzz at
// 10-20 Hz (simulated: 30x more high-frequency command energy in the frictionless case).  It is learned in place:
//   * it only grows while the robot stands quietly (no remote command, small body and wheel speeds) AND the controller is
//     pushing beyond the hysteresis -- exactly when the dead zone blocks it; a robot that stands still never gets a preload;
//   * it grows slowly (`ramp_units_per_s`) up to `max_units`;
//   * the moment the sign flips faster than `chatter_rate` per second with a small command swing (preload too large: the
//     relay is chattering; pushes and hands, which swing the command widely, do not count) it is cut by `chatter_backoff`
//     and frozen for the rest of the session; repeated chatter cuts it again.
// It does not have to start from zero: `initial_units` is the value this robot needed when it was measured (12 on the
// second robot), so it stands still from the first second; `grow_mean_u` makes the learning wait for evidence that the
// preload is too small -- the controller pushing hard (mean |command| over 1 s) without the wheel giving way -- instead of
// probing upwards until it chatters at every boot.
// So the preload settles just below the dead zone the robot really has (simulated: 0.8-1.0 of it for dead zones of 25-70
// units, capped at `max_units`), whatever that is.  It is an
// actuator-side offset on the common-mode wheel command (the two wheels keep their difference, so yaw is untouched), not
// a second controller: it never looks at anything the network does not see except the quiet flag that gates the learning.
//
// Header-only and free of hardware calls so tests/test_dead_zone_comp.cpp can drive it on the host.
#include <cmath>
#include <cstdint>

namespace rider { namespace policy {

struct DeadZoneConfig {
    float max_units = 0.0f;            // upper bound of the learned preload, command units; 0 switches the feature off
    float hysteresis = 8.0f;           // the sign follows the command only beyond +-this (about 3x the command noise at rest)
    float ramp_units_per_s = 1.5f;     // growth rate while quiet and pushing
    float chatter_rate = 8.0f;         // sign flips per second (leaky average) that count as chattering
    float chatter_backoff = 0.8f;      // the preload is multiplied by this at every chatter trip
    float chatter_max_u = 30.0f;       // chatter is small: while the network's command swings wider than this (0.5 s peak) the
                                       // robot is being pushed or handled and sign flips are not counted
    float initial_units = 0.0f;        // preload at (re)start, 0..max_units: what this robot needs, so that nothing has to be
                                       // learned at every boot (the learning only corrects it)
    float grow_mean_u = 0.0f;          // grow only while the 1 s mean of |command| is above this, i.e. the controller keeps
                                       // pushing without the wheel giving way; 0 = grow whenever it pushes beyond the hysteresis
};

struct DeadZoneState {
    float preload = 0.0f;              // learned, command units
    float flip_rate = 0.0f;            // leaky flips per second
    float u_peak = 0.0f;               // leaky peak of |u_common| (0.5 s): large = pushed or handled
    float u_abs_mean = 0.0f;           // leaky mean of |u_common| (1 s): large = the dead zone blocks the controller
    std::int8_t sign = 0;              // +1 / -1: direction of the last push beyond the hysteresis, 0 before the first one
    bool locked = false;               // chatter seen: no further growth until reset
    std::uint32_t trips = 0;           // chatter trips since reset
};

constexpr float kFlipTauS = 0.5f;      // time constant of the flip-rate average
constexpr float kMeanTauS = 1.0f;      // time constant of the mean |command|

// Returns the offset to ADD to the common mode of the wheel command (the caller adds +x to wheel 11 and -x to wheel 21,
// the wheels being mirrored).  `u_common` is the network's common-mode command (units), `quiet` the "standing, nobody
// is driving it" flag, `dt` the control period in seconds.
inline float applyDeadZonePreload(DeadZoneState& s, const DeadZoneConfig& c, float u_common, bool quiet, float dt) {
    if (!(c.max_units > 0.0f) || !std::isfinite(u_common) || !(dt > 0.0f)) {
        s.sign = 0;
        s.flip_rate = 0.0f;
        return 0.0f;
    }
    const std::int8_t before = s.sign;
    if (u_common > c.hysteresis) s.sign = 1;
    else if (u_common < -c.hysteresis) s.sign = -1;
    const float decay = dt / kFlipTauS < 1.0f ? dt / kFlipTauS : 1.0f;
    s.flip_rate -= s.flip_rate * decay;
    s.u_peak -= s.u_peak * decay;
    s.u_abs_mean += (std::fabs(u_common) - s.u_abs_mean) * (dt / kMeanTauS < 1.0f ? dt / kMeanTauS : 1.0f);
    if (std::fabs(u_common) > s.u_peak) s.u_peak = std::fabs(u_common);
    // A push or a hand on the robot makes the loop ring through +-hysteresis several times a second with a wide swing
    // (measured on the robot 2026-10-22: two such events cut a learned preload from 15.5 to 10 and locked it there);
    // the relay chatter of an oversized preload is small.  Only small swings count.
    if (s.u_peak < c.chatter_max_u && before != 0 && s.sign != before) s.flip_rate += 1.0f / kFlipTauS;
    if (s.preload > 0.5f && s.flip_rate > c.chatter_rate) {
        s.preload *= c.chatter_backoff;
        s.flip_rate = 0.0f;
        s.locked = true;
        ++s.trips;
    } else if (!s.locked && quiet && s.sign != 0 && std::fabs(u_common) > c.hysteresis && s.u_abs_mean >= c.grow_mean_u) {
        s.preload += c.ramp_units_per_s * dt;
    }
    if (s.preload > c.max_units) s.preload = c.max_units;
    return s.sign * s.preload;
}

// Forget the relay state and the flip average (stop, arm, lift guard) but keep what was learned: the dead zone is a
// property of the robot, not of the last run.
inline void resetDeadZoneRelay(DeadZoneState& s) {
    s.sign = 0;
    s.flip_rate = 0.0f;
    s.u_peak = 0.0f;
    s.u_abs_mean = 0.0f;
}

// The state at the start of a session, and after `!dzcomp reset`: nothing learned, the preload at its starting value.
inline void startDeadZone(DeadZoneState& s, const DeadZoneConfig& c) {
    s = DeadZoneState();
    s.preload = c.max_units > 0.0f ? (c.initial_units < c.max_units ? c.initial_units : c.max_units) : 0.0f;
    if (!(s.preload > 0.0f)) s.preload = 0.0f;
}

// Forget everything learned (console `!dzcomp reset`), back to the zero starting value.
inline void resetDeadZoneLearning(DeadZoneState& s) { s = DeadZoneState(); }

}}  // namespace rider::policy
