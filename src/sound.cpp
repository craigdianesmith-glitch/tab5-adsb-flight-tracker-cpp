#include "sound.h"

#include <M5Unified.h>

#include "alerts.h"
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

// The words, made by tools/gen_voice.py: a second of 16-bit sound each, at
// the voice's own rate, played from flash by the speaker's own task, so
// saying one costs the UI no more than a note does.
#include "voice_table.inc"

struct Clip {
    const int16_t *samples;
    size_t count;
};
#define CLIP(a) Clip{a, sizeof(a) / sizeof(a[0])}

// Said once the notes are done; nullptr for nothing to say.
const Clip *g_word = nullptr;

// The reason the banner names, in the same order - see alertReasonText().
const Clip *wordFor(uint8_t reasons) {
    static const Clip EMERGENCY = CLIP(VOICE_EMERGENCY), WATCHLIST = CLIP(VOICE_WATCHLIST), RARE = CLIP(VOICE_RARE),
                      MILITARY = CLIP(VOICE_MILITARY);
    if (reasons & ALERT_EMERGENCY) return &EMERGENCY;
    if (reasons & ALERT_WATCHLIST) return &WATCHLIST;
    if (reasons & ALERT_RARE) return &RARE;
    if (reasons & ALERT_MILITARY) return &MILITARY;
    return nullptr;
}

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

void soundAlert(uint8_t reasons) {
    if (!g_ready || g_muted) {
        return;
    }
    bool emergency = (reasons & ALERT_EMERGENCY) != 0;
    g_word = wordFor(reasons);
    g_notes = emergency ? WARBLE : CHIME;
    g_noteCount = emergency ? (int)(sizeof(WARBLE) / sizeof(Note)) : (int)(sizeof(CHIME) / sizeof(Note));
    g_nextNote = 0;
}

void soundTick() {
    if (g_notes == nullptr || M5.Speaker.isPlaying(ALERT_CHANNEL)) {
        return;
    }
    if (g_nextNote >= g_noteCount || g_muted) {
        // The notes done, what it is - unless muted since.
        if (g_word != nullptr && !g_muted) {
            M5.Speaker.playRaw(g_word->samples, g_word->count, VOICE_RATE, false, 1, ALERT_CHANNEL, true);
        }
        g_word = nullptr;
        g_notes = nullptr;
        return;
    }
    const Note &n = g_notes[g_nextNote++];
    M5.Speaker.tone(n.hz, n.ms, ALERT_CHANNEL, true);
}
