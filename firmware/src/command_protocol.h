#pragma once
#include <cstddef>
#include <cstdint>

namespace rider2 { namespace protocol {
enum class Source : uint8_t { Usb = 0, Aux = 1, Ble = 3 };
enum class CommandKind : uint8_t {
    ActionLoop = 0x03, LegCalibration = 0x04, ReservedNoop = 0x05,
    UnknownSetting06 = 0x06, TelemetryStream = 0x08, PitchCalibration = 0x09,
    HeadingHold = 0x0a, BleSuffix = 0x13, Forward = 0x30,
    Turn = 0x32, Height = 0x35, Roll = 0x36, RollWave = 0x39,
    Action = 0x3e, PositionHoldProtection = 0x60, Leveling = 0x61,
    Rgb0 = 0x69, Rgb1 = 0x6a, Rgb2 = 0x6b, Rgb3 = 0x6c,
    HeightWave = 0x82, MaxForward = 0xa0, MaxTurn = 0xa1,
    PolicyMode = 0xb0,
    TraceStream = 0xb3, WheelGain = 0xb4, WheelSmoothMs = 0xb5,
    WheelSlew = 0xb6, YawSlew = 0xb7, SpeedCal = 0xb8,
    Unknown = 0xff
};
struct CommandEvent {
    Source source = Source::Usb;
    uint8_t type = 0, reg = 0, length = 0;
    uint8_t data[24] = {};
    bool isRead() const { return type == 2; }
};
struct ParserStats {
    uint32_t frames = 0, bad_checksum = 0, bad_length = 0,
             bad_trailer = 0, unsupported = 0, timeouts = 0;
};
class StreamParser {
public:
    // One parser slot per transport; callbacks must not call this concurrently.
    bool feed(Source source, uint8_t byte, uint32_t now_ms, CommandEvent &event);
    void reset(Source source);
    void expire(Source source, uint32_t now_ms);
    bool receiving(Source source) const;
    const ParserStats &stats() const { return stats_; }
private:
    struct Slot { uint8_t bytes[32] = {}; uint8_t used = 0, expected = 0; uint32_t last_ms = 0; };
    Slot slots_[3];
    ParserStats stats_;
};
CommandKind commandKind(const CommandEvent &event);
const char *commandName(CommandKind kind);
bool validCommand(const CommandEvent &event);

struct Telemetry {
    float battery_percent = 0;
    uint8_t mode = 0;
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    float accel[3] = {};
    float odometry[4] = {};  // x, y, heading, integrated distance
    int32_t leg12_position = 512, leg22_position = 512;
    uint16_t leg12_zero = 512, leg22_zero = 512;
    uint8_t policy_mode=0,policy_action_space=0;
    uint8_t policy_contract=6;
    bool policy_trained=false,policy_valid=false;
    uint32_t policy_model_id=0;
    // Read 0xb2: build/source/mode/last-stop, then three LE uint32 counters.
    uint8_t controller_kind=0,wheel_source=0,last_stop_reason=0;
    uint32_t policy_calls=0,policy_failures=0,rl_output_ticks=0;
    char version[10] = {'R','-','1','.','1','.','6',0,0,0};
};
// Original read response: 55 00 length 12 reg payload checksum 00 AA.
// Canonical lengths are used; malformed request lengths cannot expose memory.
size_t buildReadResponse(const CommandEvent &request, const Telemetry &telemetry,
                         uint8_t *out, size_t capacity);
// 0x400d40ec: fixed 45 bytes, 55 00 battery + 10 LE floats + 00 AA.
// This distinct stream packet has neither length nor checksum fields.
size_t buildStreamTelemetry(const Telemetry &telemetry,uint8_t *out,size_t capacity);
// Ordinary framed response helper; also used by host tests / tools.
size_t buildFrame(uint8_t type, uint8_t reg, const uint8_t *payload, size_t length,
                  uint8_t *out, size_t capacity);
} }
