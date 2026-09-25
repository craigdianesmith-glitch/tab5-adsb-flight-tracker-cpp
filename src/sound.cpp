#include "sound.h"

#include <M5Unified.h>

#include "config.h"

namespace {

bool g_ready = false;
bool g_muted = false;

// Everything shares one virtual channel and queues rather than interrupting,
// so a new-flight blip that lands during the boot rise waits its turn instead
// of cutting it off.
constexpr uint8_t CHANNEL = 0;

}  // namespace

void soundInit() {
    if (!SOUND_ENABLED) {
        return;
    }
    g_ready = M5.Speaker.begin();
    if (!g_ready) {
        Serial.println("[sound] speaker failed to start");
        return;
    }
    M5.Speaker.setVolume(SOUND_VOLUME);
    Serial.printf("[sound] speaker ready, volume %d\n", SOUND_VOLUME);
}

void soundSetMuted(bool muted) { g_muted = muted; }

bool soundMuted() { return g_muted; }

void soundBoot() {
    if (!g_ready || g_muted) {
        return;
    }
    M5.Speaker.tone(880, 90, CHANNEL, false);
    M5.Speaker.tone(1320, 140, CHANNEL, false);
}

void soundNewFlight() {
    if (!g_ready || g_muted) {
        return;
    }
    M5.Speaker.tone(1568, 70, CHANNEL, false);
}
