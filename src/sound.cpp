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
// Key clicks get a channel of their own and cut off the one before, so fast
// typing stays in step with the fingers instead of queueing behind itself -
// or behind an arrival blip on the shared channel.
constexpr uint8_t CLICK_CHANNEL = 1;
// Alerts are several notes long. Queued on the shared channel, tone() would
// block the UI loop until there was room for each note in turn, so they are
// fed to a channel of their own one at a time instead, from soundTick().
constexpr uint8_t ALERT_CHANNEL = 2;

struct Note {
    uint16_t hz;
    uint16_t ms;
};
constexpr Note CHIME[] = {{1319, 110}, {1568, 110}, {2093, 200}};
constexpr Note WARBLE[] = {{988, 160}, {740, 160}, {988, 160}, {740, 160}, {988, 160}, {740, 240}};
const Note *g_notes = nullptr;
int g_noteCount = 0;
int g_nextNote = 0;

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

void soundKeyClick() {
    if (!g_ready || g_muted) {
        return;
    }
    M5.Speaker.tone(KEY_CLICK_HZ, KEY_CLICK_MS, CLICK_CHANNEL, true);
}

void soundNewFlight() {
    if (!g_ready || g_muted) {
        return;
    }
    M5.Speaker.tone(1568, 70, CHANNEL, false);
}

void soundAlert(bool emergency) {
    if (!g_ready || g_muted) {
        return;
    }
    g_notes = emergency ? WARBLE : CHIME;
    g_noteCount = emergency ? (int)(sizeof(WARBLE) / sizeof(Note)) : (int)(sizeof(CHIME) / sizeof(Note));
    g_nextNote = 0;
}

void soundTick() {
    if (g_notes == nullptr || M5.Speaker.isPlaying(ALERT_CHANNEL)) {
        return;
    }
    if (g_nextNote >= g_noteCount || g_muted) {
        g_notes = nullptr;
        return;
    }
    const Note &n = g_notes[g_nextNote++];
    M5.Speaker.tone(n.hz, n.ms, ALERT_CHANNEL, true);
}
