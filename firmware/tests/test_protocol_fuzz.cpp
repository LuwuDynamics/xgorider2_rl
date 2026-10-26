// Randomised robustness test of the binary command parser, the response builders and the "!fallguard ms" parser.
// tools/run_host_tests.py builds it with AddressSanitizer + UBSan.  The generator is seeded, so a failure reproduces.
//
// Properties checked:
//   * whatever bytes arrive on any transport (random, near-valid, truncated, mutated), an accepted command is valid and bounded;
//   * after any garbage the parser recovers: a good frame sent after the 100 ms inter-byte timeout is always accepted, exactly as sent;
//   * response frames never exceed the 64-byte buffer the firmware gives them, for any telemetry value (NaN, inf, huge);
//   * "!fallguard ms N" never accepts a value outside the allowed range.
#include "command_protocol.h"
#include "fall_guard.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace rider2::protocol;

#define CHECK(condition) do { if (!(condition)) { std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition); return 1; } } while (0)

static const uint8_t kKnownRegisters[] = {0x03, 0x04, 0x05, 0x06, 0x08, 0x09, 0x0a, 0x13, 0x30, 0x32, 0x35, 0x36, 0x39, 0x3e, 0x60, 0x61,
                                          0x69, 0x6a, 0x6b, 0x6c, 0x82, 0xa0, 0xa1, 0xb0, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8,
                                          0x01, 0x02, 0x07, 0x0b, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0xb1, 0xb2};

static bool acceptedIsSane(const CommandEvent& event) {
    if (event.length > sizeof event.data) return false;
    if (!validCommand(event)) return false;
    if (event.isRead()) {
        Telemetry telemetry;
        uint8_t out[64];
        const size_t n = buildReadResponse(event, telemetry, out, sizeof out);
        if (n > sizeof out || n == 0) return false;
    }
    return true;
}

int main() {
    std::mt19937_64 rng(20261024);
    const Source sources[] = {Source::Usb, Source::Aux, Source::Ble};
    uint64_t accepted = 0;

    // 1) random byte streams, biased toward the bytes a frame is made of
    {
        StreamParser parser;
        uint32_t now = 0;
        for (uint64_t i = 0; i < 6000000ull; ++i) {
            uint8_t b;
            const uint32_t r = uint32_t(rng() % 100);
            if (r < 8) b = 0x55; else if (r < 16) b = 0x00; else if (r < 24) b = 0xAA; else if (r < 30) b = uint8_t(8 + rng() % 25); else b = uint8_t(rng());
            now += (rng() % 50 == 0) ? 150 : 1;
            CommandEvent event;
            if (parser.feed(sources[i % 3], b, now, event)) { ++accepted; CHECK(acceptedIsSane(event)); }
        }
    }

    // 2) every register / type / length as a well-formed frame
    uint64_t wellFormed = 0;
    for (int reg = 0; reg < 256; ++reg) for (int type = 0; type < 4; ++type) for (int len = 0; len <= 30; ++len) {
        uint8_t payload[32];
        for (int k = 0; k < 32; ++k) payload[k] = uint8_t(rng());
        uint8_t frame[64];
        const size_t n = buildFrame(uint8_t(type), uint8_t(reg), payload, size_t(len), frame, sizeof frame);
        if (n == 0) continue;
        StreamParser parser;
        CommandEvent event;
        bool got = false;
        for (size_t k = 0; k < n; ++k) got = parser.feed(Source::Ble, frame[k], 1000 + uint32_t(k), event) || got;
        if (got) { ++wellFormed; CHECK(acceptedIsSane(event)); CHECK(event.reg == reg && event.type == type); }
    }
    CHECK(wellFormed > 100);

    // 3) near-valid input: mutate / truncate / pad valid frames, then prove the parser recovers
    for (int iteration = 0; iteration < 300000; ++iteration) {
        StreamParser parser;
        uint32_t now = 5000;
        const Source source = sources[iteration % 3];
        uint8_t payload[24];
        for (auto& v : payload) v = uint8_t(rng());
        const uint8_t reg = kKnownRegisters[rng() % sizeof kKnownRegisters];
        const size_t length = rng() % 25;
        uint8_t frame[64];
        size_t n = buildFrame(uint8_t(rng() % 3), reg, payload, length, frame, sizeof frame);
        if (n == 0) continue;
        std::vector<uint8_t> bytes(frame, frame + n);
        switch (rng() % 5) {
        case 0: for (unsigned k = 1 + rng() % 3; k; --k) bytes[rng() % bytes.size()] ^= uint8_t(1u << (rng() % 8)); break;
        case 1: bytes.resize(1 + rng() % bytes.size()); break;
        case 2: for (unsigned k = rng() % 12; k; --k) bytes.push_back(uint8_t(rng())); break;
        case 3: bytes.insert(bytes.begin(), uint8_t(0x55)); bytes.insert(bytes.begin(), uint8_t(0x55)); break;
        default: bytes.insert(bytes.begin() + long(rng() % bytes.size()), uint8_t(rng())); break;
        }
        for (uint8_t b : bytes) {
            now += 1;
            CommandEvent event;
            if (parser.feed(source, b, now, event)) { ++accepted; CHECK(acceptedIsSane(event)); }
        }
        // recovery: after the inter-byte timeout a good frame must be accepted exactly as sent
        const uint8_t goodPayload = uint8_t(rng());
        uint8_t good[16];
        const size_t m = buildFrame(0, 0x30, &goodPayload, 1, good, sizeof good);
        now += 200;
        int seen = 0;
        CommandEvent last;
        for (size_t k = 0; k < m; ++k) { CommandEvent event; if (parser.feed(source, good[k], now + uint32_t(k), event)) { ++seen; last = event; } }
        CHECK(seen == 1);
        CHECK(last.reg == 0x30 && last.length == 1 && last.data[0] == goodPayload && last.source == source);
    }

    // 4) extreme telemetry values never overflow a response, whatever the register
    {
        Telemetry t;
        const float values[] = {0.f, -1.f, 1e30f, -1e30f, NAN, INFINITY, -INFINITY, 2147483648.f, -2147483904.f, 255.5f};
        const int registers[] = {0x01, 0x02, 0x07, 0x0b, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0xb0, 0xb1, 0xb2};
        for (float a : values) for (float b : values) {
            t.battery_percent = a; t.roll_deg = b; t.pitch_deg = a; t.yaw_deg = b;
            for (int k = 0; k < 3; ++k) t.accel[k] = a;
            for (int k = 0; k < 4; ++k) t.odometry[k] = b;
            uint8_t out[64];
            for (int reg : registers) {
                CommandEvent event; event.type = 2; event.reg = uint8_t(reg); event.length = 1;
                const size_t n = buildReadResponse(event, t, out, sizeof out);
                CHECK(n <= sizeof out && n > 0);
            }
            CHECK(buildStreamTelemetry(t, out, sizeof out) == 45);
        }
    }

    // 5) "!fallguard ms N": only values inside the allowed range are ever accepted
    {
        const char alphabet[] = "!fallguard ms0123456789.eE+-nainf \t";
        for (int i = 0; i < 1000000; ++i) {
            char line[48];
            std::memcpy(line, "!fallguard ms", 13);
            size_t pos = 13;
            const size_t extra = rng() % 30;
            for (size_t k = 0; k < extra && pos < 46; ++k) line[pos++] = alphabet[rng() % (sizeof alphabet - 1)];
            line[pos] = 0;
            float ms = 0;
            const int result = rider2::fall::parseHoldCommand(line, ms);
            if (result == 2) CHECK(rider2::fall::validHoldMs(ms));
        }
    }

    std::printf("protocol fuzz ok: %llu commands accepted from random and mutated streams, %llu well-formed frames\n",
                (unsigned long long)accepted, (unsigned long long)wellFormed);
    return 0;
}
