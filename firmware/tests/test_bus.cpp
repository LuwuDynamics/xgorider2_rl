#include "../src/actuator_bus.h"
#include <array>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstdio>

using namespace rider::bringup;
using Frame = std::array<std::uint8_t, 12>;
using ExtendedFrame = std::array<std::uint8_t, 15>;
static unsigned checks = 0;
static void check(bool value) {
    ++checks;
    if (!value) { std::fprintf(stderr, "bus check %u FAILED\n", checks); std::abort(); }
}
template<std::size_t N> static void checksum(std::array<std::uint8_t, N> &f) {
    unsigned sum = 0;
    for (std::size_t i = 2; i + 1 < N; ++i) sum += f[i];
    f[N - 1] = std::uint8_t(~sum);
}
static Frame frame(std::uint8_t id, int position, int speed, int load = 0, int status = 0) {
    Frame f{{255, 255, id, 8, std::uint8_t(status), std::uint8_t(position),
             std::uint8_t(position >> 8), std::uint8_t(speed), std::uint8_t(speed >> 8),
             std::uint8_t(load), std::uint8_t(load >> 8), 0}};
    checksum(f);
    return f;
}
static ExtendedFrame extendedFrame(std::uint8_t id, int position, int speed, int load = 0, int status = 0) {
    // Nonzero prefix must never be mistaken for position/speed/load.
    ExtendedFrame f{{255, 255, id, 11, std::uint8_t(status), 0xa5, 0x5a, 0xc3,
                     std::uint8_t(position), std::uint8_t(position >> 8),
                     std::uint8_t(speed), std::uint8_t(speed >> 8),
                     std::uint8_t(load), std::uint8_t(load >> 8), 0}};
    checksum(f);
    return f;
}
template<std::size_t N> static bool feed(ActuatorDecoder &d, const std::array<std::uint8_t, N> &f, std::uint32_t now) {
    bool result = false;
    for (auto byte : f) result = d.feed(byte, now) || result;
    return result;
}
static bool transact(ActuatorDecoder &d, std::uint8_t id, int position, int speed,
                     std::uint32_t now, int load = 0) {
    d.expect(id);
    return feed(d, frame(id, position, speed, load), now);
}
int main() {
    std::uint8_t request[8]{};
    for (unsigned id = 0; id < 256; ++id) {
        const bool allowed = id == 11 || id == 21 || id == 12 || id == 22;
        check((makeActuatorReadRequest(std::uint8_t(id), request) == 8) == allowed);
        if (allowed) {
            check(request[0] == 255 && request[1] == 255 && request[2] == id);
            check(request[3] == 4 && request[4] == 2 && request[5] == 0x24 && request[6] == 6);
            unsigned sum = 0; for (unsigned i = 2; i < 8; ++i) sum += request[i];
            check((sum & 255) == 255);
        }
    }
    check(makeActuatorReadRequest(11, nullptr) == 0);
    ActuatorDecoder d;
    check(!d.sample(11).valid && d.sample(11).ageMs(100) == UINT32_MAX);
    check(!d.sample(99).valid);
    check(!feed(d, frame(11, 512, 100), 1)); // unsolicited rejected
    check(d.stats().unexpected_frames == 1);
    for (std::uint8_t id : {11, 21, 12, 22}) {
        check(transact(d, id, 512, -123, 100, -456));
        const auto &s = d.sample(id);
        check(s.valid && s.id == id && s.position_ticks == 512);
        check(s.raw_speed == -123 && s.load_raw == -456 && s.sample_count == 1);
        check(s.last_update_ms == 100 && s.fresh(150, 50) && !s.fresh(151, 50));
        check(s.relative_encoder_ticks == 0);
    }
    check(d.sample(11).encoder_ticks == 512 && d.sample(21).encoder_ticks == -512);
    check(d.sample(11).wheel_speed_mm_s < 0 && d.sample(21).wheel_speed_mm_s > 0);
    check(d.sample(12).wheel_speed_mm_s == 0);
    check(d.stats().valid_extended_frames == 0);
    // OEM LENGTH=11 responses use the same final six bytes as LENGTH=8.
    // Compare full decoded math for both wheels, signed words, and tick limits.
    for (std::uint8_t id : {11, 21}) {
        ActuatorDecoder extended, ordinary;
        std::uint32_t now = 1;
        for (int position : {0, 1, 512, 1023}) for (int velocity : {-32768, -123, 0, 123, 32767}) {
            const int load = velocity == -32768 ? 32767 : -velocity;
            extended.expect(id);
            check(feed(extended, extendedFrame(id, position, velocity, load), now));
            check(transact(ordinary, id, position, velocity, now, load));
            const auto &a = extended.sample(id), &b = ordinary.sample(id);
            check(a.position_ticks == position && a.raw_speed == velocity && a.load_raw == load);
            check(a.encoder_ticks == b.encoder_ticks && a.relative_encoder_ticks == b.relative_encoder_ticks);
            check(a.wheel_speed_mm_s == b.wheel_speed_mm_s && a.filtered_speed_mm_s == b.filtered_speed_mm_s);
            check(a.sample_count == now && a.last_update_ms == now);
            check(extended.stats().valid_extended_frames == now && ordinary.stats().valid_extended_frames == 0);
            ++now;
        }
    }
    // Independent known value: raw 100 means approximately 80.98664 mm/s.
    ActuatorDecoder speed;
    check(transact(speed, 11, 0, 100, 0));
    check(std::fabs(speed.sample(11).wheel_speed_mm_s - 80.986640625f) < 0.00002f);
    check(std::fabs(speed.sample(11).filtered_speed_mm_s - 20.24666015625f) < 0.00001f);
    check(transact(speed, 11, 0, 100, 1));
    check(std::fabs(speed.sample(11).filtered_speed_mm_s - 35.4316552734f) < 0.00002f);
    // Every 16-bit wire word must decode with two's-complement, not sign/magnitude.
    ActuatorDecoder words;
    for (unsigned raw = 0; raw < 65536; ++raw) {
        const int expected = raw < 32768 ? int(raw) : int(raw) - 65536;
        check(transact(words, 12, 512, int(raw), raw, int(raw)));
        check(words.sample(12).raw_speed == expected && words.sample(12).load_raw == expected);
    }
    // Reject every unsupported length without accepting or indexing beyond a frame.
    for (unsigned length = 0; length < 256; ++length) if (length != 8 && length != 11 && length != 2) {
        ActuatorDecoder bad;
        bad.expect(11);
        auto f = frame(11, 512, 10); f[3] = std::uint8_t(length); checksum(f);
        check(!feed(bad, f, 0));
        check(!bad.sample(11).valid && bad.stats().length_errors == 1);
    }
    for (unsigned bit = 0; bit < 8; ++bit) for (unsigned index = 4; index < 12; ++index) {
        ActuatorDecoder bad;
        bad.expect(11);
        auto f = frame(11, 512, 10); f[index] ^= std::uint8_t(1u << bit);
        check(!feed(bad, f, 0));
        check(!bad.sample(11).valid && bad.stats().checksum_errors == 1);
    }
    for (unsigned bit = 0; bit < 8; ++bit) for (unsigned index = 4; index < 15; ++index) {
        ActuatorDecoder bad;
        bad.expect(21);
        auto f = extendedFrame(21, 512, -10, -456); f[index] ^= std::uint8_t(1u << bit);
        check(!feed(bad, f, 0));
        check(!bad.sample(21).valid && bad.stats().checksum_errors == 1);
        check(bad.stats().valid_extended_frames == 0);
    }
    for (unsigned status = 1; status < 256; ++status) {
        ActuatorDecoder bad; bad.expect(11);
        check(!feed(bad, frame(11, 512, 0, 0, int(status)), 50));
        check(!bad.sample(11).valid && bad.stats().status_errors == 1);
        check(bad.sample(11).last_status == status && bad.sample(11).last_status_ms == 50);
        bad.expect(21);
        check(!feed(bad, extendedFrame(21, 512, -10, 0, int(status)), 60));
        check(!bad.sample(21).valid && bad.stats().status_errors == 2);
        check(bad.sample(21).last_status == status && bad.stats().valid_extended_frames == 0);
    }
    for (int position : {-32768, -1, 1024, 32767}) {
        ActuatorDecoder bad; bad.expect(11);
        check(!feed(bad, extendedFrame(11, position, 0), 0));
        check(bad.stats().position_errors == 1 && bad.stats().valid_extended_frames == 0);
    }
    ActuatorDecoder extended_validation;
    extended_validation.expect(11);
    check(!feed(extended_validation, extendedFrame(99, 512, 10), 0));
    check(extended_validation.stats().id_errors == 1 && extended_validation.stats().valid_extended_frames == 0);
    check(!feed(extended_validation, extendedFrame(21, 512, 10), 0));
    check(extended_validation.stats().unexpected_frames == 1 && extended_validation.stats().valid_extended_frames == 0);
    check(feed(extended_validation, extendedFrame(11, 512, -10), 1));
    check(extended_validation.stats().valid_extended_frames == 1);
    check(!feed(extended_validation, extendedFrame(11, 513, -20), 2));
    check(extended_validation.stats().valid_extended_frames == 1 && extended_validation.sample(11).last_update_ms == 1);
    // Every truncated extended prefix must remain incomplete; its expiry cannot
    // refresh feedback, and a subsequent intact packet must recover cleanly.
    for (std::size_t prefix = 1; prefix < 15; ++prefix) {
        ActuatorDecoder truncated;
        truncated.expect(21);
        check(feed(truncated, extendedFrame(21, 500, -123), 99));
        truncated.expect(21);
        const auto f = extendedFrame(21, 600, -456);
        for (std::size_t i = 0; i < prefix; ++i) check(!truncated.feed(f[i], 100));
        check(truncated.stats().valid_extended_frames == 1 && truncated.sample(21).last_update_ms == 99);
        truncated.expirePartial(104);
        check(truncated.stats().partial_timeouts == 1);
        check(feed(truncated, f, 105));
        check(truncated.stats().valid_extended_frames == 2 && truncated.sample(21).position_ticks == 600);
    }
    ActuatorDecoder validation;
    { const std::uint8_t ack[]={255,255,12,2,0,241};
      for(auto byte:ack)validation.feed(byte,0);
      check(validation.stats().ack_frames==1&&!validation.sample(12).valid);
    }
    validation.expect(11);
    check(!feed(validation, frame(99, 512, 0), 0) && validation.stats().id_errors == 1);
    check(!feed(validation, frame(21, 512, 0), 0) && validation.stats().unexpected_frames == 1);
    check(!feed(validation, frame(11, -1, 0), 0) && validation.stats().position_errors == 1);
    validation.expect(12);
    check(!feed(validation, frame(12, 1024, 0), 0) && validation.stats().position_errors == 2);
    check(transact(validation, 11, 1, 0, 2));
    validation.expect(11);
    check(!feed(validation, frame(11, 2, 0, 0, 4), 200));
    check(validation.sample(11).last_update_ms == 2 && validation.sample(11).position_ticks == 1);
    // Prefix/noise recovery, split packet expiry, and unsigned timestamp rollover.
    ActuatorDecoder stream; stream.expect(11);
    for (std::uint8_t byte : {3, 255, 0, 255, 255, 255, 255}) stream.feed(byte, 0);
    check(feed(stream, frame(11, 512, 0), 0));
    stream.expect(11); stream.feed(255, 1); stream.feed(255, 1); stream.feed(11, 1);
    stream.expirePartial(5);
    check(stream.stats().partial_timeouts == 1);
    check(feed(stream, frame(11, 513, 0), 5));
    check(transact(stream, 11, 514, 0, UINT32_MAX - 2));
    check(stream.sample(11).ageMs(2) == 5);
    check(transact(stream, 11, 515, 0, 2));
    // Original boundary/wrap behavior and ID21 direction.
    for (std::uint8_t id : {11, 21}) {
        const int sign = id == 11 ? 1 : -1;
        ActuatorDecoder enc;
        check(transact(enc, id, 1000, 0, 1));
        check(transact(enc, id, 10, 0, 2));
        check(enc.sample(id).relative_encoder_ticks == sign * 33);
        check(transact(enc, id, 1000, 0, 3));
        check(enc.sample(id).relative_encoder_ticks == 0);
        check(transact(enc, id, 999, 0, 104));
        check(enc.sample(id).encoder_continuity_lost);
        ActuatorDecoder threshold;
        check(transact(threshold, id, 0, 0, 0));
        check(transact(threshold, id, 800, 0, 1));
        check(threshold.sample(id).relative_encoder_ticks == sign * 800);
        check(transact(threshold, id, 0, 0, 2));
        check(threshold.sample(id).relative_encoder_ticks == 0);
        check(transact(threshold, id, 801, 0, 3));
        check(threshold.sample(id).relative_encoder_ticks == sign * -222);
    }
    check(transact(speed, 21, 0, -32768, 1));
    check(speed.sample(21).wheel_speed_mm_s < 0); // original int16 negation edge
    std::printf("{\"test\":\"actuator_bus\",\"checks\":%u,\"result\":\"PASS\"}\n", checks);
}
