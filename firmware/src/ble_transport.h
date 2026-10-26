#pragma once
#include <cstddef>
#include <cstdint>
namespace rider2 {
// Must be called outside the time-critical control loop. BLE callback only
// enqueues RX bytes; all parser, motion and flash operations run in caller task.
bool beginBle(const char *name);
size_t readBle(uint8_t *out, size_t capacity);
bool notifyBle(const uint8_t *bytes, size_t length);
bool bleConnected();
uint32_t bleDroppedBytes();
// Call from application task to restart advertising after a disconnect.
void pollBle();
}
