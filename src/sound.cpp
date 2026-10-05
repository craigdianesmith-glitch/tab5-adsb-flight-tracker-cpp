#include "sound.h"

#include <M5Unified.h>

#include "aircraft_db.h"
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

// What is said once the notes are done, a clip at a time - the reason, then
// the callsign spelt out - each with the pause to leave after it.
struct Spoken {
    Clip clip;
    uint16_t pauseMs;
};
// The reason, an 8-character callsign, and a type - its maker, a model of
// up to 8 characters and its variants - with room to spare.
constexpr int MAX_SPOKEN = 32;
constexpr uint16_t AFTER_REASON_MS = 250, BETWEEN_LETTERS_MS = 60, AFTER_CALLSIGN_MS = 250, AFTER_MAKER_MS = 120;
Spoken g_spoken[MAX_SPOKEN];
int g_spokenCount = 0, g_nextSpoken = 0;
uint32_t g_speakAtMs = 0;  // not before then: the pause after the clip before

// The reason the banner names, in the same order - see alertReasonText().
bool wordFor(uint8_t reasons, Clip &out) {
    if (reasons & ALERT_EMERGENCY) out = CLIP(VOICE_EMERGENCY);
    else if (reasons & ALERT_WATCHLIST) out = CLIP(VOICE_WATCHLIST);
    else if (reasons & ALERT_RARE) out = CLIP(VOICE_RARE);
    else if (reasons & ALERT_MILITARY) out = CLIP(VOICE_MILITARY);
    else return false;
    return true;
}

// Queues `clip` to be said, then `pauseMs` of quiet.
void say(const Clip &clip, uint16_t pauseMs) {
    if (g_spokenCount < MAX_SPOKEN) {
        g_spoken[g_spokenCount++] = {clip, pauseMs};
    }
}

// Spells `text` a letter or digit at a time; anything else in it - a space,
// a hyphen - left unsaid.
void spell(const String &text) {
    for (size_t i = 0; i < text.length(); i++) {
        char c = toupper((unsigned char)text[i]);
        if (c >= 'A' && c <= 'Z') {
            say({VOICE_LETTERS[c - 'A'], VOICE_LETTER_LENS[c - 'A']}, BETWEEN_LETTERS_MS);
        } else if (c >= '0' && c <= '9') {
            say({VOICE_DIGITS[c - '0'], VOICE_DIGIT_LENS[c - '0']}, BETWEEN_LETTERS_MS);
        }
    }
}

// A type name from the aircraft table, said as its maker and its model
// spelt: "Airbus A380-800" as "Airbus... Alpha three eight zero". The model
// is the first word after the maker with a digit in it, up to a variant
// after a hyphen or a slash - 737 of 737-800, 800 of 800/850 - but not a
// hyphen or slash after letters alone, which are part of it: C-130J, F/A-18.
// A neo, a MAX or a Dreamliner is said after it. False for a name with no
// maker the table knows, as an unknown type's bare code is.
bool sayType(const String &name) {
    const VoiceMaker *maker = nullptr;
    for (const VoiceMaker &m : VOICE_MAKERS) {
        size_t n = strlen(m.name);
        if (name.startsWith(m.name) && (name.length() == n || name[n] == ' ') &&
            (maker == nullptr || n > strlen(maker->name))) {
            maker = &m;
        }
    }
    if (maker == nullptr) {
        return false;
    }
    say({maker->samples, maker->count}, AFTER_MAKER_MS);
    String rest = name.substring(strlen(maker->name));
    bool neo = false, max = false, dreamliner = false, haveModel = false;
    String model;
    int from = 0;
    while (from < (int)rest.length()) {
        int space = rest.indexOf(' ', from);
        String word = rest.substring(from, space < 0 ? rest.length() : space);
        from = space < 0 ? rest.length() : space + 1;
        max = max || word == "MAX";
        dreamliner = dreamliner || word == "Dreamliner";
        if (haveModel || word.length() == 0) {
            continue;
        }
        bool digits = false;
        for (size_t i = 0; i < word.length(); i++) {
            digits = digits || isdigit((unsigned char)word[i]);
        }
        if (!digits) {
            continue;
        }
        if (word.endsWith("neo")) {
            neo = true;
            word.remove(word.length() - 3);
        }
        // Cut at the first hyphen or slash with a digit before it.
        for (size_t i = 0; i < word.length(); i++) {
            if ((word[i] == '-' || word[i] == '/') && i > 0 && isdigit((unsigned char)word[i - 1])) {
                word.remove(i);
                break;
            }
        }
        model = word;
        haveModel = true;
    }
    spell(model);
    if (neo) say(CLIP(VOICE_VARIANT_NEO), BETWEEN_LETTERS_MS);
    if (max) say(CLIP(VOICE_VARIANT_MAX), BETWEEN_LETTERS_MS);
    if (dreamliner) say(CLIP(VOICE_VARIANT_DREAMLINER), BETWEEN_LETTERS_MS);
    return true;
}

// The airport zoom's announcements waiting their turn, behind an alert or
// one another - a few at most, as an airport's movements come minutes apart.
struct Announcement {
    String callsign;
    String runway;
    bool takeoff;
};
constexpr int MAX_ANNOUNCEMENTS = 3;
Announcement g_announcements[MAX_ANNOUNCEMENTS];
int g_announcementCount = 0;
// An announcement has no chime: only words, played as an alert's are.
constexpr Note NO_NOTES[] = {{0, 0}};

// Queues "easyJet five three Tango Hotel, runway two three, cleared for
// take-off": the airline by its name where the table has it and a clip for
// it, then the rest of the callsign spelt - or, an airline it doesn't know,
// all of it - then the runway, its number a digit at a time and its side.
void startAnnouncement(const Announcement &a) {
    g_spokenCount = g_nextSpoken = 0;
    AirlineInfo airline;
    const VoiceMaker *voice = nullptr;
    if (lookupAirline(a.callsign, airline)) {
        for (const VoiceMaker &v : VOICE_AIRLINES) {
            if (airline.name == v.name) {
                voice = &v;
            }
        }
    }
    if (voice) {
        say({voice->samples, voice->count}, AFTER_MAKER_MS);
        spell(a.callsign.substring(3));
    } else {
        spell(a.callsign);
    }
    if (g_spokenCount > 0) {
        g_spoken[g_spokenCount - 1].pauseMs = AFTER_CALLSIGN_MS;
    }
    say(CLIP(VOICE_RUNWAY), BETWEEN_LETTERS_MS);
    for (size_t i = 0; i < a.runway.length(); i++) {
        char c = a.runway[i];
        if (c >= '0' && c <= '9') {
            say({VOICE_DIGITS[c - '0'], VOICE_DIGIT_LENS[c - '0']}, BETWEEN_LETTERS_MS);
        } else if (c == 'L') {
            say(CLIP(VOICE_SIDE_L), BETWEEN_LETTERS_MS);
        } else if (c == 'R') {
            say(CLIP(VOICE_SIDE_R), BETWEEN_LETTERS_MS);
        } else if (c == 'C') {
            say(CLIP(VOICE_SIDE_C), BETWEEN_LETTERS_MS);
        }
    }
    g_spoken[g_spokenCount - 1].pauseMs = AFTER_CALLSIGN_MS;
    say(a.takeoff ? CLIP(VOICE_CLEARED_TAKEOFF) : CLIP(VOICE_CLEARED_LAND), 0);
    Serial.printf("[sound] announcing %s, runway %s, cleared %s: %d spoken\n", a.callsign.c_str(), a.runway.c_str(),
                  a.takeoff ? "for take-off" : "to land", g_spokenCount);
    g_notes = NO_NOTES;
    g_noteCount = 0;
    g_nextNote = 0;
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

void soundAlert(uint8_t reasons, const String &callsign, const String &typeName) {
    if (!g_ready || g_muted) {
        Serial.printf("[sound] alert for %s not sounded: %s\n", callsign.c_str(), g_muted ? "muted" : "no speaker");
        return;
    }
    bool emergency = (reasons & ALERT_EMERGENCY) != 0;
    g_spokenCount = g_nextSpoken = 0;
    Clip word;
    if (wordFor(reasons, word)) {
        say(word, AFTER_REASON_MS);
    }
    // Spelt as it would be read out over the radio, and then what it is.
    spell(callsign);
    if (g_spokenCount > 0 && typeName.length()) {
        g_spoken[g_spokenCount - 1].pauseMs = AFTER_CALLSIGN_MS;
    }
    sayType(typeName);
    g_notes = emergency ? WARBLE : CHIME;
    // A line for each, as the speaker can go silent with nothing else to
    // show for it - see the README.
    Serial.printf("[sound] alert for %s: %s, then %d spoken\n", callsign.c_str(), emergency ? "warble" : "chime",
                  g_spokenCount);
    g_noteCount = emergency ? (int)(sizeof(WARBLE) / sizeof(Note)) : (int)(sizeof(CHIME) / sizeof(Note));
    g_nextNote = 0;
}

void soundAnnounce(const String &callsign, const String &runway, bool takeoff) {
    String cs = callsign;
    cs.trim();
    if (!g_ready || g_muted || cs.length() == 0 || runway.length() == 0) {
        return;
    }
    if (g_announcementCount == MAX_ANNOUNCEMENTS) {
        Serial.printf("[sound] announcement for %s dropped: %d waiting\n", cs.c_str(), g_announcementCount);
        return;
    }
    g_announcementCount++;
    g_announcements[g_announcementCount - 1] = {cs, runway, takeoff};
}

void soundTick() {
    // An alert, or the announcement before, finished: the next announcement.
    if (g_notes == nullptr && g_announcementCount > 0 && !M5.Speaker.isPlaying(ALERT_CHANNEL)) {
        Announcement a = g_announcements[0];
        for (int i = 1; i < g_announcementCount; i++) {
            g_announcements[i - 1] = g_announcements[i];
        }
        g_announcementCount--;
        if (!g_muted) {
            startAnnouncement(a);
        }
    }
    if (g_notes == nullptr || M5.Speaker.isPlaying(ALERT_CHANNEL)) {
        return;
    }
    if (g_nextNote >= g_noteCount || g_muted) {
        // The notes done, what it is, a clip at a time - unless muted since.
        if (g_nextSpoken < g_spokenCount && !g_muted) {
            if ((int32_t)(millis() - g_speakAtMs) < 0) {
                return;  // the pause after the clip before
            }
            const Spoken &s = g_spoken[g_nextSpoken++];
            M5.Speaker.playRaw(s.clip.samples, s.clip.count, VOICE_RATE, false, 1, ALERT_CHANNEL, true);
            g_speakAtMs = millis() + s.clip.count * 1000 / VOICE_RATE + s.pauseMs;
            return;
        }
        g_spokenCount = g_nextSpoken = 0;
        g_notes = nullptr;
        return;
    }
    const Note &n = g_notes[g_nextNote++];
    M5.Speaker.tone(n.hz, n.ms, ALERT_CHANNEL, true);
}
