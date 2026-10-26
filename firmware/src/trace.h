#pragma once
// Diagnostic trace record shared by the control task (writer), the BLE stream and the USB export.
// Header-only so the text format can be unit-tested on the host (tests/test_trace.cpp).
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace rider2 { namespace trace {

// 28-byte sample streamed unchanged over BLE (register 0xb3, tools/capture_trace.py).  Do not reorder.
struct Sample {
    std::int16_t pitch, pitch_rate, roll, yaw_rate, cmd11, cmd21;
    std::int16_t sp11, sp21, fsp11, fsp21, leg_a, leg_b, a0, a1;
};
static_assert(sizeof(Sample) == 28, "the BLE trace sample layout is fixed");

// USB-only companion written at the same ring index: time, wheel servo load/position, the raw accelerometer, how
// fresh each servo's feedback is and how long the control period was.
// load* is the servo's present-load register as read (bit 10 is the direction in this servo family);
// acc_* are body-axis specific force in cm/s^2; age* are ms since that servo (ids 11, 21, 12, 22) last answered
// (saturated at 32767); dt_us is the control period that ended with this sample (saturated at 65535);
// flags: bit 0 = wheel output held at zero by the lift guard, bit 1 = lift detector active (also when only observing),
// bit 2 = wheel speed above the lift limit on this tick, bit 3 = a dead-zone preload (src/policy/dead_zone_comp.h) is learned
// and enabled, bit 4 / bit 5 = its relay is on the positive / negative side (cmd11 - a0 is then the preload), bit 6 = the raw
// acceleration of this sample is below the free-fall limit, bit 7 = a free fall is detected (src/fall_guard.h).  load12/load22/pos12/pos22 are the leg servos' present load and
// position as read; vbat_adc is the raw 12-bit battery ADC (about 2.5 mV per count at the cell input, unfiltered) so a
// supply dip when the servos enable shows up.
struct Extra {
    std::uint32_t ms;
    std::int16_t load11, load21, pos11, pos21, acc_x, acc_y, acc_z, flags;
    std::int16_t age11, age21, age12, age22;
    std::uint16_t dt_us;
    std::int16_t load12, load22, pos12, pos22;
    std::uint16_t vbat_adc;
};

constexpr const char* kCsvHeader =
    "# trace seq,ms,pitch_cdeg,pitch_rate_mrad_s,roll_cdeg,yaw_rate_cdeg_s,cmd11,cmd21,sp11,sp21,fsp11,fsp21,"
    "leg_a,leg_b,a0,a1,load11,load21,pos11,pos21,acc_x_cm_s2,acc_y_cm_s2,acc_z_cm_s2,flags,age11,age21,age12,age22,dt_us,load12,load22,pos12,pos22,vbat_adc";

// One "T,..." text line per sample (same field order as kCsvHeader).  Returns the length, or 0 when it does not fit.
inline std::size_t formatLine(char* out, std::size_t capacity, std::uint32_t seq, const Sample& s, const Extra& x) {
    const int n = std::snprintf(out, capacity,
        "T,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d,%u\n",
        static_cast<unsigned>(seq), static_cast<unsigned>(x.ms),
        s.pitch, s.pitch_rate, s.roll, s.yaw_rate, s.cmd11, s.cmd21, s.sp11, s.sp21, s.fsp11, s.fsp21,
        s.leg_a, s.leg_b, s.a0, s.a1, x.load11, x.load21, x.pos11, x.pos21, x.acc_x, x.acc_y, x.acc_z, x.flags,
        x.age11, x.age21, x.age12, x.age22, static_cast<unsigned>(x.dt_us), x.load12, x.load22, x.pos12, x.pos22,
        static_cast<unsigned>(x.vbat_adc));
    return n > 0 && static_cast<std::size_t>(n) < capacity ? static_cast<std::size_t>(n) : 0;
}

// Cursor of the USB export: mode 0 off, 1 one-shot dump of the ring, 2 continuous stream.  Pure bookkeeping so the
// "never read a slot the writer has already reused" rule can be tested without hardware.
struct UsbCursor {
    std::uint8_t mode = 0;
    std::uint32_t next = 0, end = 0, sent = 0;
    bool finished = false;       // set when a dump has printed its last sample; cleared by the caller
    void dump(std::uint32_t head, std::uint32_t capacity) {
        end = head;
        next = head > capacity ? head - capacity : 0;
        sent = 0;
        finished = false;
        mode = 1;
    }
    void stream(std::uint32_t head) {
        next = head;
        sent = 0;
        finished = false;
        mode = 2;
    }
    void stop() { mode = 0; }
    // Next sample index to print, or false when there is nothing to print right now.  Finishes a dump by switching
    // the mode off (finished = true); a lagging reader skips to the oldest sample the ring still holds.
    bool take(std::uint32_t head, std::uint32_t capacity, std::uint32_t& index) {
        if (mode == 0) return false;
        if (static_cast<std::int32_t>(head - next) > static_cast<std::int32_t>(capacity)) next = head - capacity;
        const std::uint32_t limit = mode == 1 ? end : head;
        if (static_cast<std::int32_t>(limit - next) <= 0) {
            if (mode == 1) {
                mode = 0;
                finished = true;
            }
            return false;
        }
        index = next++;
        ++sent;
        return true;
    }
};

}}  // namespace rider2::trace
