#include "sound.h"

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <M5Unified.h>
#include <math.h>

#include <algorithm>
#include <new>

#include "aircraft_db.h"
#include "alerts.h"
#include "config.h"
#include "regions.h"
#include "splash_takeoff.h"

namespace {

bool g_ready = false;
bool g_muted = false;
TaskHandle_t g_soundTask = nullptr;

// What is to be said is handed to the speaker a note or a word at a time by a
// task of its own - see soundTask() - not by the UI loop between screens: the
// 3D view, opening on an alert's auto-follow, takes the loop for a second
// or more building its airfield, and the alert stopped mid-word until it
// was done. What is queued is the loop's to change and the task's to read,
// so both hold this while they do.
SemaphoreHandle_t g_lock = nullptr;
struct Held {
    Held() {
        if (g_lock != nullptr) {
            xSemaphoreTakeRecursive(g_lock, portMAX_DELAY);
        }
    }
    ~Held() {
        if (g_lock != nullptr) {
            xSemaphoreGiveRecursive(g_lock);
        }
    }
};

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
// The 3D view's engine noise, made as it plays - see soundEngine() - three
// blocks of it in turn, 40ms each: one playing, one queued behind it, one
// being made.
constexpr uint8_t ENGINE_CHANNEL = 3;
constexpr int ENGINE_BLOCK = engine_noise::RATE / 25;
// Nothing set for this long - the 3D view closed or covered without saying
// so - and it fades out.
constexpr uint32_t ENGINE_STALE_MS = 1000;
// What the 3D view last asked it to sound like, and when - its own lock, a
// moment's, not the sound's: the view sets it every frame, and was kept
// waiting while a block was made.
portMUX_TYPE g_engineMux = portMUX_INITIALIZER_UNLOCKED;
engine_noise::Sound g_engineWant;
uint32_t g_engineSetMs = 0;
// The splash screen's take-off - see soundTakeoff() - asked for, and when;
// under the same lock.
bool g_takeoffAsked = false;
uint32_t g_takeoffFromMs = 0;
// The sound task's alone.
engine_noise::Synth *g_engine = nullptr;
int16_t *g_engineBlocks[3] = {};
int g_engineNext = 0;
bool g_enginePlaying = false;  // something to play: set as it is asked for, cleared once faded out
bool g_engineFailed = false;   // no room for it
int g_takeoffBlock = -1;       // the splash's take-off: its next 40ms block, or none

struct Note {
    uint16_t hz;
    uint16_t ms;
};
constexpr Note CHIME[] = {{1319, 110}, {1568, 110}, {2093, 200}};
constexpr Note WARBLE[] = {{988, 160}, {740, 160}, {988, 160}, {740, 160}, {988, 160}, {740, 240}};
// An announcement or a reading has no chime: only words, played as an
// alert's are.
constexpr Note NO_NOTES[] = {{0, 0}};
const Note *g_notes = nullptr;
int g_noteCount = 0;
int g_nextNote = 0;

// The words, made by tools/gen_voice.py in one voice, English: a second or so
// of sound each, at the voice's own rate, as 4-bit IMA ADPCM - half what 8-bit
// samples took, 1.24MB of flash - turned back into samples as each is said
// (see playClip()), and played by the speaker's own task, so saying one costs
// the UI no more than a note does.
struct Clip {
    const uint8_t *adpcm;  // two samples a byte, the first in the low nibble
    size_t count;          // samples
};
struct NamedClip {
    const char *name;
    Clip clip;
};
// Every word in the voice, in the order tools/gen_voice.py writes them.
struct Voice {
    uint32_t rate;
    Clip emergency, military, rare, watchlist;
    Clip letters[26];
    Clip digits[10];
    const NamedClip *makers;  // by the name a type name begins with
    size_t makerCount;
    Clip neo, max, dreamliner;
    const NamedClip *airlines;  // by the airline's name in the airline table
    size_t airlineCount;
    Clip runway, left, right, centre, clearedTakeoff, clearedLand;
    Clip decimal, feet, thousand, hundred, flightLevel, knots, heading, climbing, descending, level, onGround, channel;
    Clip navaidTypes[6];          // in NavaidType's order
    Clip atcUses[ATC_USE_COUNT];  // in AtcUse's order
};
#include "voice_english.inc"
#include "click_table.inc"

// Everything is said in it, wherever it is about: Piper's en_GB-cori-medium,
// from public-domain recordings - see tools/gen_voice.py for why it is the
// only one.
using voice_english::VOICE;
bool g_readAloud = false;

// What is said once the notes are done, a clip at a time - for an alert, the
// reason, then the callsign spelt out - each with the pause to leave after
// it.
struct Spoken {
    Clip clip;
    uint16_t pauseMs;
};
// An airport's frequencies are the most there is to say: one for each of
// nine uses at most, a word and up to eight digits each.
constexpr int MAX_SPOKEN = 84;
constexpr uint16_t AFTER_REASON_MS = 250, BETWEEN_LETTERS_MS = 60, AFTER_CALLSIGN_MS = 250, AFTER_MAKER_MS = 120;
Spoken g_spoken[MAX_SPOKEN];
int g_spokenCount = 0, g_nextSpoken = 0;
uint32_t g_speakAtMs = 0;  // not before then: the pause after the clip before
// Whether what is being said is a reading - see soundReadAircraft() - which
// the next one cuts short, where it waits for an alert or an announcement.
bool g_readingNow = false;

// The reason the banner names, in the same order - see alertReasonText().
bool wordFor(uint8_t reasons, Clip &out) {
    if (reasons & ALERT_EMERGENCY) out = VOICE.emergency;
    else if (reasons & ALERT_WATCHLIST) out = VOICE.watchlist;
    else if (reasons & ALERT_RARE) out = VOICE.rare;
    else if (reasons & ALERT_MILITARY) out = VOICE.military;
    else return false;
    return true;
}

// Queues `clip` to be said, then `pauseMs` of quiet.
void say(const Clip &clip, uint16_t pauseMs = BETWEEN_LETTERS_MS) {
    if (g_spokenCount < MAX_SPOKEN) {
        g_spoken[g_spokenCount++] = {clip, pauseMs};
    }
}

// A longer pause after what has been queued so far, between one thing said
// and the next.
void pauseAfter(uint16_t pauseMs) {
    if (g_spokenCount > 0) {
        g_spoken[g_spokenCount - 1].pauseMs = pauseMs;
    }
}

// Spells `text` a letter or digit at a time; anything else in it - a space,
// a hyphen - left unsaid.
void spell(const String &text) {
    for (size_t i = 0; i < text.length(); i++) {
        char c = toupper((unsigned char)text[i]);
        if (c >= 'A' && c <= 'Z') {
            say(VOICE.letters[c - 'A']);
        } else if (c >= '0' && c <= '9') {
            say(VOICE.digits[c - '0']);
        }
    }
}

// A callsign as the zoom's announcements say it: the airline by its name
// where the airline table and the voice have it, the rest spelt - or all of
// it spelt.
void sayCallsign(const String &callsign) {
    AirlineInfo airline;
    const NamedClip *voice = nullptr;
    if (lookupAirline(callsign, airline)) {
        for (size_t i = 0; i < VOICE.airlineCount; i++) {
            if (airline.name == VOICE.airlines[i].name) {
                voice = &VOICE.airlines[i];
            }
        }
    }
    if (voice) {
        say(voice->clip, AFTER_MAKER_MS);
        spell(callsign.substring(3));
    } else {
        spell(callsign);
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
    const NamedClip *maker = nullptr;
    for (size_t i = 0; i < VOICE.makerCount; i++) {
        const NamedClip &m = VOICE.makers[i];
        size_t n = strlen(m.name);
        if (name.startsWith(m.name) && (name.length() == n || name[n] == ' ') &&
            (maker == nullptr || n > strlen(maker->name))) {
            maker = &m;
        }
    }
    if (maker == nullptr) {
        return false;
    }
    say(maker->clip, AFTER_MAKER_MS);
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
    if (neo) say(VOICE.neo);
    if (max) say(VOICE.max);
    if (dreamliner) say(VOICE.dreamliner);
    return true;
}

// A frequency as a controller gives it: "one one eight decimal eight", the
// MHz and then its fraction without the zeros at its end - all three digits
// of an 8.33kHz channel's, "one three two decimal eight zero five".
void sayFrequency(uint32_t khz) {
    spell(String(khz / 1000));
    say(VOICE.decimal);
    char fraction[4];
    snprintf(fraction, sizeof(fraction), "%03u", (unsigned)(khz % 1000));
    int n = 3;
    while (n > 1 && fraction[n - 1] == '0') {
        n--;
    }
    fraction[n] = 0;
    spell(fraction);
}

// A height as a controller gives it: below the transition altitude, in
// thousands and hundreds of feet, "four thousand four hundred feet"; at or
// above it, as a flight level, "flight level three five zero".
void sayAltitude(long ft, long transitionFt) {
    if (ft >= transitionFt) {
        say(VOICE.flightLevel);
        spell(String((ft + 50) / 100));
        return;
    }
    long hundreds = (ft + 50) / 100;
    if (hundreds >= 10) {
        spell(String(hundreds / 10));  // "one two thousand", as the digits
        say(VOICE.thousand);
    }
    if (hundreds % 10) {
        spell(String(hundreds % 10));
        say(VOICE.hundred);
    }
    if (hundreds == 0) {
        spell("0");
    }
    say(VOICE.feet);
}

// The airport zoom's announcements waiting their turn, behind an alert or
// one another - a few at most, as an airport's movements come minutes apart.
struct Announcement {
    String callsign;
    String runway;
    bool takeoff;
    uint32_t queuedMs;
};
// Waiting behind an alert's words longer than this, it is over by the time it
// would be said - an aircraft cleared for take-off is in the air - and dropped.
constexpr uint32_t ANNOUNCE_STALE_MS = 8000;
constexpr int MAX_ANNOUNCEMENTS = 3;
Announcement g_announcements[MAX_ANNOUNCEMENTS];
int g_announcementCount = 0;

// An alert's notes alone, waiting their turn - see soundAlert() - and since
// when; gone stale as an announcement does.
bool g_chimeWaiting = false;
bool g_chimeEmergency = false;
uint32_t g_chimeQueuedMs = 0;

// Begins an alert's notes - the warble for an emergency, else the chime -
// with what has been queued in g_spoken to say after them.
void startNotes(bool emergency) {
    g_notes = emergency ? WARBLE : CHIME;
    g_noteCount = emergency ? (int)(sizeof(WARBLE) / sizeof(Note)) : (int)(sizeof(CHIME) / sizeof(Note));
    g_nextNote = 0;
}

// Begins saying what has been queued in g_spoken: no notes, then the words.
void speakQueued(bool reading) {
    g_notes = NO_NOTES;
    g_noteCount = 0;
    g_nextNote = 0;
    g_readingNow = reading;
}

// Queues "easyJet five three Tango Hotel, runway two three, cleared for
// take-off": the callsign as sayCallsign() has it, then the runway, its
// number a digit at a time and its side.
void startAnnouncement(const Announcement &a) {
    g_spokenCount = g_nextSpoken = 0;
    sayCallsign(a.callsign);
    pauseAfter(AFTER_CALLSIGN_MS);
    say(VOICE.runway);
    for (size_t i = 0; i < a.runway.length(); i++) {
        char c = a.runway[i];
        if (c >= '0' && c <= '9') {
            say(VOICE.digits[c - '0']);
        } else if (c == 'L') {
            say(VOICE.left);
        } else if (c == 'R') {
            say(VOICE.right);
        } else if (c == 'C') {
            say(VOICE.centre);
        }
    }
    pauseAfter(AFTER_CALLSIGN_MS);
    say(a.takeoff ? VOICE.clearedTakeoff : VOICE.clearedLand, 0);
    Serial.printf("[sound] announcing %s, runway %s, cleared %s: %d spoken\n", a.callsign.c_str(), a.runway.c_str(),
                  a.takeoff ? "for take-off" : "to land", g_spokenCount);
    speakQueued(false);
}

// Read aloud - see soundReadAircraft(): the latest asked for, waiting until
// what is being said has been, unless that is a reading too.
enum class ReadKind : uint8_t { NONE, AIRCRAFT, NAVAID, AIRPORT };
struct Reading {
    ReadKind kind = ReadKind::NONE;
    Aircraft aircraft;
    const Navaid *navaid = nullptr;
    const AirportInfo *airport = nullptr;
    Region region = Region::ELSEWHERE;  // the aircraft's, for its height: see readAircraft()
};
Reading g_reading;

void queueReading(const Reading &r) {
    if (!g_ready || g_muted) {
        return;
    }
    g_reading = r;
    // Reading something else: cut short for this, as the card or the
    // aircraft it was about has gone.
    if (g_readingNow && g_notes != nullptr) {
        M5.Speaker.stop(ALERT_CHANNEL);
        g_notes = nullptr;
        g_spokenCount = g_nextSpoken = 0;
        g_readingNow = false;
    }
}

// "Ryanair six six eight Tango, Boeing seven three seven max, climbing, four
// thousand four hundred feet, one seven five knots, heading two four zero" -
// or, down, "... on the ground".
void readAircraft(const Aircraft &a, Region region) {
    String callsign = a.callsign;
    callsign.trim();
    if (callsign.length()) {
        sayCallsign(callsign);
    } else {
        spell(a.reg);
    }
    pauseAfter(AFTER_CALLSIGN_MS);
    if (a.type.length() && sayType(lookupAircraftType(a.type, a.military))) {
        pauseAfter(AFTER_CALLSIGN_MS);
    }
    if (a.altStr == "GND" || a.status == "GROUND" || a.status == "TAXI") {
        say(VOICE.onGround, 0);
        return;
    }
    if (a.status == "CLIMB") {
        say(VOICE.climbing, AFTER_MAKER_MS);
    } else if (a.status == "DESCEND") {
        say(VOICE.descending, AFTER_MAKER_MS);
    } else if (a.status == "LEVEL") {
        say(VOICE.level, AFTER_MAKER_MS);
    }
    long ft = a.altStr.toInt();
    if (a.altStr != "?" && ft > 0) {
        // Flight levels from 18,000ft over the United States, and from
        // 6,000ft - the most common in the UK - everywhere else.
        sayAltitude(ft, region == Region::USA ? 18000 : 6000);
        pauseAfter(AFTER_CALLSIGN_MS);
    }
    if (a.speedStr != "?") {
        spell(String(a.speedStr.toInt()));
        say(VOICE.knots, AFTER_CALLSIGN_MS);
    }
    if (a.hasTrack) {
        int heading = ((int)lroundf(a.track) % 360 + 360) % 360;
        char digits[8];
        snprintf(digits, sizeof(digits), "%03d", heading == 0 ? 360 : heading);
        say(VOICE.heading);
        spell(digits);
    }
    pauseAfter(0);
}

// "Golf Oscar Whiskey, V O R D M E, one one five decimal four" - or a
// TACAN's channel, "channel one zero one X-ray".
void readNavaid(const Navaid &n) {
    spell(n.ident);
    pauseAfter(AFTER_CALLSIGN_MS);
    say(VOICE.navaidTypes[(int)n.type], AFTER_CALLSIGN_MS);
    if (n.type == NavaidType::TACAN && n.channel[0]) {
        say(VOICE.channel);
        spell(n.channel);
    } else if (n.freqKhz && n.type == NavaidType::NDB_DME) {
        spell(String(n.freqKhz));  // an NDB's, in kHz
    } else if (n.freqKhz) {
        sayFrequency(n.freqKhz);
    }
    pauseAfter(0);
}

// A single set of the frequencies the card lists: one for each use, in
// the card's order - "Tower, one one eight decimal five; Ground, one two one
// decimal seven..." - the first where a busy airport has several, as
// Heathrow has three towers and four ATIS.
void readAirport(const AirportInfo &airport) {
    const AirportFrequency *f = airportFrequencies(airport);
    int lastUse = -1;
    for (int i = 0; i < airport.frequencyCount; i++) {
        if ((int)f[i].use == lastUse || f[i].use >= ATC_USE_COUNT) {
            continue;
        }
        lastUse = f[i].use;
        say(VOICE.atcUses[f[i].use], AFTER_MAKER_MS);
        sayFrequency(f[i].khz);
        pauseAfter(AFTER_CALLSIGN_MS);
    }
    pauseAfter(0);
}

void startReading(const Reading &r) {
    g_spokenCount = g_nextSpoken = 0;
    switch (r.kind) {
    case ReadKind::AIRCRAFT:
        readAircraft(r.aircraft, r.region);
        break;
    case ReadKind::NAVAID:
        readNavaid(*r.navaid);
        break;
    case ReadKind::AIRPORT:
        readAirport(*r.airport);
        break;
    case ReadKind::NONE:
        return;
    }
    static const char *const KINDS[] = {"", "aircraft", "navaid", "airport"};
    Serial.printf("[sound] reading %s: %d spoken\n", KINDS[(int)r.kind], g_spokenCount);
    if (g_spokenCount > 0) {
        speakQueued(true);
    }
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
    g_lock = xSemaphoreCreateRecursiveMutex();
    // A step above the loop on its core: it takes a moment every 10ms, and a
    // millisecond to ready each word. Its stack 6KB, the most the words have
    // taken some 2.5KB of it; making the 3D view's engine noise, as the
    // compiler counts it from soundTick() down, some 2.2KB.
    xTaskCreatePinnedToCore(
        [](void *) {
            for (;;) {
                soundTick();
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        },
        "sound", 6144, nullptr, 2, &g_soundTask, 1);
}

void soundSetMuted(bool muted) { g_muted = muted; }

void soundEngine(const engine_noise::Sound &sound) {
    taskENTER_CRITICAL(&g_engineMux);
    g_engineWant = sound;
    g_engineSetMs = millis();
    taskEXIT_CRITICAL(&g_engineMux);
}

bool soundMuted() { return g_muted; }

unsigned soundStackSpare() { return g_soundTask ? (unsigned)uxTaskGetStackHighWaterMark(g_soundTask) : 0u; }

void soundSetReadAloud(bool on) { g_readAloud = on; }

void soundTakeoff() {
    if (!g_ready || g_muted) {
        return;
    }
    // Made as it plays by the sound's task, with the 3D view's synthesiser -
    // see engineTick().
    taskENTER_CRITICAL(&g_engineMux);
    g_takeoffAsked = true;
    g_takeoffFromMs = millis();
    taskEXIT_CRITICAL(&g_engineMux);
}

// A phone keyboard's click rather than a beep - see tools/gen_click.py -
// from flash, as the words are.
void soundKeyClick() {
    if (!g_ready || g_muted) {
        return;
    }
    M5.Speaker.playRaw(KEY_CLICK, sizeof(KEY_CLICK) / sizeof(KEY_CLICK[0]), CLICK_RATE, false, 1, CLICK_CHANNEL, true);
}

void soundNewFlight() {
    if (!g_ready || g_muted) {
        return;
    }
    M5.Speaker.tone(1568, 70, CHANNEL, false);
}

void soundAlert(uint8_t reasons, const String &callsign, const String &typeName, bool spoken) {
    Held held;
    if (!g_ready || g_muted) {
        Serial.printf("[sound] alert for %s not sounded: %s\n", callsign.c_str(), g_muted ? "muted" : "no speaker");
        return;
    }
    bool emergency = (reasons & ALERT_EMERGENCY) != 0;
    if (!spoken) {
        g_chimeWaiting = true;
        g_chimeEmergency = emergency;
        g_chimeQueuedMs = millis();
        Serial.printf("[sound] alert for %s: %s alone, in its turn\n", callsign.c_str(), emergency ? "warble" : "chime");
        return;
    }
    g_chimeWaiting = false;  // this one's notes are its
    g_spokenCount = g_nextSpoken = 0;
    g_readingNow = false;
    Clip word;
    if (wordFor(reasons, word)) {
        say(word, AFTER_REASON_MS);
    }
    // Spelt as it would be read out over the radio, and then what it is.
    spell(callsign);
    if (typeName.length()) {
        pauseAfter(AFTER_CALLSIGN_MS);
    }
    sayType(typeName);
    startNotes(emergency);
    // A line for each, as the speaker can go silent with nothing else to
    // show for it - see the README.
    Serial.printf("[sound] alert for %s: %s, then %d spoken\n", callsign.c_str(), emergency ? "warble" : "chime",
                  g_spokenCount);
}

void soundAnnounce(const String &callsign, const String &runway, bool takeoff) {
    Held held;
    String cs = callsign;
    cs.trim();
    if (!g_ready || g_muted || cs.length() == 0 || runway.length() == 0) {
        return;
    }
    if (g_announcementCount == MAX_ANNOUNCEMENTS) {
        Serial.printf("[sound] announcement for %s dropped: %d waiting\n", cs.c_str(), g_announcementCount);
        return;
    }
    g_announcements[g_announcementCount++] = {cs, runway, takeoff, millis()};
}

void soundReadAircraft(const Aircraft &ac, bool always) {
    Held held;
    if (!g_readAloud && !always) {
        return;
    }
    Reading r;
    r.kind = ReadKind::AIRCRAFT;
    r.aircraft = ac;
    r.region = ac.hasPos ? regionNear(ac.lat, ac.lon) : Region::ELSEWHERE;
    queueReading(r);
}

void soundReadNavaid(const Navaid &navaid, bool always) {
    Held held;
    if (!g_readAloud && !always) {
        return;
    }
    Reading r;
    r.kind = ReadKind::NAVAID;
    r.navaid = &navaid;
    queueReading(r);
}

void soundReadAirport(const AirportInfo &airport, bool always) {
    Held held;
    if ((!g_readAloud && !always) || airport.frequencyCount == 0) {
        return;
    }
    Reading r;
    r.kind = ReadKind::AIRPORT;
    r.airport = &airport;
    queueReading(r);
}

void soundStopReading() {
    Held held;
    g_reading = Reading();
    if (g_readingNow && g_notes != nullptr) {
        M5.Speaker.stop(ALERT_CHANNEL);
        g_notes = nullptr;
        g_spokenCount = g_nextSpoken = 0;
        g_readingNow = false;
    }
}

namespace {

// IMA ADPCM's tables, as tools/ima_adpcm.py and tools/adpcm_search.c have them.
constexpr int16_t ADPCM_STEP[89] = {
    7,     8,     9,     10,    11,    12,    13,    14,    16,    17,    19,    21,    23,    25,    28,
    31,    34,    37,    41,    45,    50,    55,    60,    66,    73,    80,    88,    97,    107,   118,
    130,   143,   157,   173,   190,   209,   230,   253,   279,   307,   337,   371,   408,   449,   494,
    544,   598,   658,   724,   796,   876,   963,   1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
    2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,  7132,  7845,  8630,
    9493,  10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};
constexpr int8_t ADPCM_INDEX[8] = {-1, -1, -1, -1, 2, 4, 6, 8};

// A clip turned back into 16-bit samples, into `out`.
void decodeClip(const Clip &clip, int16_t *out) {
    int pred = 0, index = 0;
    for (size_t i = 0; i < clip.count; i++) {
        int code = (clip.adpcm[i >> 1] >> ((i & 1) * 4)) & 15;
        int step = ADPCM_STEP[index], delta = step >> 3;
        if (code & 4) {
            delta += step;
        }
        if (code & 2) {
            delta += step >> 1;
        }
        if (code & 1) {
            delta += step >> 2;
        }
        pred += (code & 8) ? -delta : delta;
        pred = pred < -32768 ? -32768 : pred > 32767 ? 32767 : pred;
        index += ADPCM_INDEX[code & 7];
        index = index < 0 ? 0 : index > 88 ? 88 : index;
        out[i] = (int16_t)pred;
    }
}

// A clip turned back into samples - into one of two buffers in PSRAM in
// turn, the speaker's task reading from the other until it has finished with
// the word before - and handed to the speaker. A second of a word is a
// millisecond's work.
void playClip(const Clip &clip, uint32_t rate) {
    static int16_t *buffers[2] = {nullptr, nullptr};
    static size_t sizes[2] = {0, 0};
    static int next = 0;
    int b = next;
    next ^= 1;
    if (sizes[b] < clip.count) {
        heap_caps_free(buffers[b]);
        buffers[b] = (int16_t *)heap_caps_malloc(clip.count * sizeof(int16_t), MALLOC_CAP_SPIRAM);
        sizes[b] = buffers[b] ? clip.count : 0;
        if (buffers[b] == nullptr) {
            return;
        }
    }
    decodeClip(clip, buffers[b]);
    M5.Speaker.playRaw(buffers[b], clip.count, rate, false, 1, ALERT_CHANNEL, true);
}

}  // namespace

namespace {

// The engine noise's next blocks made and queued, as there is room for them
// - under the words, if there are any. Its synthesiser and its blocks are
// this task's alone, made the first time it is asked for.
void engineTick() {
    taskENTER_CRITICAL(&g_engineMux);
    engine_noise::Sound want = g_engineWant;
    uint32_t setMs = g_engineSetMs, takeoffFromMs = g_takeoffFromMs;
    bool takeoffAsked = g_takeoffAsked;
    g_takeoffAsked = false;
    taskEXIT_CRITICAL(&g_engineMux);
    // The splash's take-off, from its first block, until it has climbed
    // away - or the 3D view asks for its own, or the speaker is muted.
    if (takeoffAsked) {
        g_takeoffBlock = 0;
    }
    bool viewAsks = (int32_t)(setMs - takeoffFromMs) > 0 && millis() - setMs <= ENGINE_STALE_MS;
    if (g_takeoffBlock >= 0 && (g_muted || viewAsks)) {
        g_takeoffBlock = -1;
    }
    bool takeoff = g_takeoffBlock >= 0;
    if (takeoff) {
        want = splash_takeoff::at(g_takeoffBlock * 0.04f);
    } else if (g_muted || millis() - setMs > ENGINE_STALE_MS) {
        want.kind = engine_noise::Kind::NONE;
    }
    if (g_engine == nullptr) {
        if (want.kind == engine_noise::Kind::NONE || g_engineFailed || !g_ready) {
            return;
        }
        for (int16_t *&b : g_engineBlocks) {
            b = b != nullptr ? b : (int16_t *)heap_caps_malloc(ENGINE_BLOCK * sizeof(int16_t), MALLOC_CAP_SPIRAM);
            if (b == nullptr) {
                Serial.println("[sound] no room for the engine noise");
                g_engineFailed = true;
                return;
            }
        }
        // In PSRAM, as its blocks are: made the first time it is asked for -
        // the splash's take-off, at boot - in the chip's own memory it lay
        // between the 3D view's buffers and split the room they come back to.
        void *room = heap_caps_malloc(sizeof(engine_noise::Synth), MALLOC_CAP_SPIRAM);
        if (room == nullptr) {
            Serial.println("[sound] no room for the engine noise");
            g_engineFailed = true;
            return;
        }
        g_engine = new (room) engine_noise::Synth();
    }
    if (!takeoff) {
        g_engine->set(want);
    }
    if (want.kind != engine_noise::Kind::NONE) {
        g_enginePlaying = true;
    }
    if (!g_enginePlaying) {
        return;
    }
    g_engine->duck(g_notes != nullptr || M5.Speaker.isPlaying(ALERT_CHANNEL) || M5.Speaker.isPlaying(CHANNEL));
    // For the log every half minute: what making it costs, and how often the
    // speaker ran out of it - this task kept from it 80ms - and was silent
    // a moment.
    static uint32_t madeUs = 0, made = 0, gaps = 0, saidMs = 0;
    static bool started = false;
    if (started && M5.Speaker.isPlaying(ENGINE_CHANNEL) == 0) {
        gaps++;
    }
    while (M5.Speaker.isPlaying(ENGINE_CHANNEL) < 2) {
        // The take-off set a block at a time, each as it is made, as the PC
        // preview's a320-splash is: the same sound, whenever the blocks are.
        if (g_takeoffBlock >= 0) {
            want = splash_takeoff::at(g_takeoffBlock * 0.04f);
            g_engine->set(want);
            g_takeoffBlock = want.kind == engine_noise::Kind::NONE ? -1 : g_takeoffBlock + 1;
        }
        int16_t *block = g_engineBlocks[g_engineNext];
        g_engineNext = (g_engineNext + 1) % 3;
        uint32_t t0 = micros();
        bool more = g_engine->render(block, ENGINE_BLOCK);
        madeUs += micros() - t0;
        made++;
        M5.Speaker.playRaw(block, ENGINE_BLOCK, engine_noise::RATE, false, 1, ENGINE_CHANNEL, false);
        started = true;
        if (!more) {
            g_enginePlaying = false;
            started = false;
            break;
        }
    }
    if (millis() - saidMs >= 30000 && made > 0) {
        // Every half minute for a developer; in a released image only when
        // there were gaps, which are heard.
#if defined(BENCH3D) || !defined(RELEASE_BUILD)
        bool say = true;
#else
        bool say = gaps > 0;
#endif
        if (say) {
            Serial.printf("[sound] engine noise: %.2fms a 40ms block, %.1f%% of a core; %u gaps\n",
                          madeUs / 1000.0f / made, madeUs / (made * 400.0f), (unsigned)gaps);
        }
        saidMs = millis();
        madeUs = made = gaps = 0;
    }
}

}  // namespace

void soundTick() {
    engineTick();
    Held held;
    // Nothing being said: an alert's notes waiting, which are news; the next
    // announcement - a clearance is only worth hearing as it happens, and
    // goes stale in seconds - or else a reading asked for, which can wait
    // the few seconds one takes.
    if (g_notes == nullptr && !M5.Speaker.isPlaying(ALERT_CHANNEL)) {
        if (g_chimeWaiting) {
            g_chimeWaiting = false;
            if (millis() - g_chimeQueuedMs > ANNOUNCE_STALE_MS) {
                Serial.printf("[sound] alert's %s dropped: %lus late\n", g_chimeEmergency ? "warble" : "chime",
                              (unsigned long)((millis() - g_chimeQueuedMs) / 1000));
            } else if (!g_muted) {
                g_spokenCount = g_nextSpoken = 0;
                g_readingNow = false;
                startNotes(g_chimeEmergency);
            }
        } else if (g_announcementCount > 0) {
            Announcement a = g_announcements[0];
            for (int i = 1; i < g_announcementCount; i++) {
                g_announcements[i - 1] = g_announcements[i];
            }
            g_announcementCount--;
            if (millis() - a.queuedMs > ANNOUNCE_STALE_MS) {
                Serial.printf("[sound] announcement for %s dropped: %lus late\n", a.callsign.c_str(),
                              (unsigned long)((millis() - a.queuedMs) / 1000));
            } else if (!g_muted) {
                startAnnouncement(a);
            }
        } else if (g_reading.kind != ReadKind::NONE) {
            Reading r = g_reading;
            g_reading = Reading();
            if (!g_muted) {
                startReading(r);
            }
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
            // A word well after its time - the speaker kept waiting - said in
            // the log, as a pause in the middle of an alert is easy to miss.
            uint32_t late = millis() - g_speakAtMs;
            if (g_nextSpoken > 0 && late > 150) {
                Serial.printf("[sound] word %d of %d %lums late\n", g_nextSpoken + 1, g_spokenCount,
                              (unsigned long)late);
            }
            const Spoken &s = g_spoken[g_nextSpoken++];
            playClip(s.clip, VOICE.rate);
            g_speakAtMs = millis() + s.clip.count * 1000 / VOICE.rate + s.pauseMs;
            return;
        }
        g_spokenCount = g_nextSpoken = 0;
        g_notes = nullptr;
        g_readingNow = false;
        return;
    }
    const Note &n = g_notes[g_nextNote++];
    M5.Speaker.tone(n.hz, n.ms, ALERT_CHANNEL, true);
}

#ifdef BENCH3D
// "##=s" (bench2d.h): the voice's words turned back into samples as they are
// said - every letter, digit, maker and airline - and ten seconds of an
// A320's sound in each of a few states, and of a Typhoon's in two, made as
// the sound's task makes it.
void soundBench() {
    size_t most = 0;
    for (const Clip &c : VOICE.letters) most = std::max(most, c.count);
    for (const Clip &c : VOICE.digits) most = std::max(most, c.count);
    for (size_t i = 0; i < VOICE.makerCount; i++) most = std::max(most, VOICE.makers[i].clip.count);
    for (size_t i = 0; i < VOICE.airlineCount; i++) most = std::max(most, VOICE.airlines[i].clip.count);
    int16_t *out = (int16_t *)heap_caps_malloc(most * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (out == nullptr) {
        Serial.println("[bench] no room to decode into");
        return;
    }
    uint64_t samples = 0;
    uint32_t clips = 0, t0 = micros();
    auto decode = [&](const Clip &c) {
        decodeClip(c, out);
        samples += c.count;
        clips++;
    };
    for (const Clip &c : VOICE.letters) decode(c);
    for (const Clip &c : VOICE.digits) decode(c);
    for (size_t i = 0; i < VOICE.makerCount; i++) decode(VOICE.makers[i].clip);
    for (size_t i = 0; i < VOICE.airlineCount; i++) decode(VOICE.airlines[i].clip);
    uint32_t us = micros() - t0;
    float seconds = (float)samples / VOICE.rate;
    Serial.printf("[bench] voice English: %u words, %.1fs of sound decoded in %.2fms - %.2fms a second, "
                  "%.0fns a sample\n",
                  (unsigned)clips, seconds, us / 1000.0f, us / 1000.0f / seconds, us * 1000.0f / samples);
    heap_caps_free(out);

    void *room = heap_caps_malloc(sizeof(engine_noise::Synth), MALLOC_CAP_SPIRAM);
    int16_t *block = (int16_t *)heap_caps_malloc(ENGINE_BLOCK * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (room == nullptr || block == nullptr) {
        heap_caps_free(room);
        heap_caps_free(block);
        Serial.println("[bench] no room for an engine");
        return;
    }
    // An A320's, as the PC's bench has them (tools/engine_preview/bench.cpp):
    // heard from 60m off its way as it goes by, set afresh each block as the
    // 3D view sets it - on its take-off roll at full power, the buzz-saw's
    // combs going; climbing out far off; on its approach, its gear and flaps
    // out and an older airframe's fuel vents whistling; touching down, its
    // reversers and spoilers out; taxiing with its APU. Each case's expect is
    // the sound the synth heard and approved (41bdaa4's, the resonator as
    // polished) makes on the P4, worked out under qemu from the tab5 build's
    // engine_noise.cpp.o - which, stripped of its debug info, is the BENCH3D
    // build's byte for byte; anything else prints CHANGED.
    struct Case {
        const char *name;
        engine_noise::Engine engine;
        float power, rollingMs, airspeedMs, x0, vx, z;
        bool gear, flaps, reverse, apu, fopp;
        float spoilers;
        uint32_t expect;
    };
    using E = engine_noise::Engine;
    const Case CASES[] = {
        {"take-off roll, CFM56", E::CFM56_5B, 1.0f, 50, 50, -600, 50, 0, false, false, false, false, false, 0,
         0xa3f95484u},
        {"take-off roll, V2500", E::V2500_A5, 1.0f, 50, 50, -600, 50, 0, false, false, false, false, false, 0,
         0xeff345bfu},
        {"take-off roll, LEAP-1A", E::LEAP_1A, 1.0f, 50, 50, -600, 50, 0, false, false, false, false, false, 0,
         0x6c1a8df4u},
        {"climb-out, CFM56, far", E::CFM56_5B, 1.0f, 0, 80, -2000, 80, 400, false, false, false, false, false, 0,
         0x91d23fb4u},
        {"approach, CFM56, vents", E::CFM56_5B, 0.45f, 0, 70, -1500, 70, 150, true, true, false, false, true, 0,
         0x2c7afe0du},
        {"touchdown, reversers", E::CFM56_5B, 0.75f, 60, 60, -50, 60, 0, true, true, true, false, false, 1,
         0x818fe1b8u},
        {"taxiing with its APU", E::CFM56_5B, 0.08f, 8, 8, -100, 8, 0, true, false, false, true, false, 0,
         0x7665e94bu},
    };
    for (const Case &e : CASES) {
        engine_noise::Synth *synth = new (room) engine_noise::Synth();
        constexpr int BLOCKS = 250;  // ten seconds
        uint32_t us = 0, sum = 2166136261u;  // and what it made, every sample, for a change to be checked by
        for (int i = 0; i < BLOCKS; i++) {
            engine_noise::Sound s;
            s.kind = engine_noise::Kind::TURBOFAN;
            s.engine = e.engine;
            s.fopp = e.fopp;
            s.who = 0x406a3c;
            s.power = e.power;
            s.reverse = e.reverse;
            s.rollingMs = e.rollingMs;
            s.airspeedMs = e.airspeedMs;
            s.gearDown = e.gear;
            s.flapsOut = e.flaps;
            s.spoilers = e.spoilers;
            s.apu = e.apu;
            s.placed = true;
            s.at[0] = e.x0 + e.vx * i * 0.04f, s.at[2] = e.z + 3;
            s.ear[1] = 60, s.ear[2] = 1.5f;
            s.sourceUpM = e.z + 3;
            s.earUpM = 1.5f;
            s.hardGround = true;
            s.ms = (uint32_t)i * 40;
            uint32_t t0 = micros();
            synth->set(s);
            synth->render(block, ENGINE_BLOCK);
            us += micros() - t0;
            for (int k = 0; k < ENGINE_BLOCK; k++) {
                sum = (sum ^ (uint16_t)block[k]) * 16777619u;
            }
        }
        synth->~Synth();
        Serial.printf("[bench] A320 %s: %.3fms a 40ms block, %.2f%% of a core; its sound %08lx, %s\n", e.name,
                      us / 1000.0f / BLOCKS, us / (BLOCKS * 400.0f), (unsigned long)sum,
                      sum == e.expect ? "as approved" : "CHANGED");
    }
    // A Typhoon's, as tools/engine_preview/eufi_cases.inc has them for the
    // PC's model of the P4's time, so the Tab5's own can be set beside it:
    // its take-off roll in reheat from 150m off the runway, and taxiing past
    // 35m off, the dearest of them all. Each one's expect as the A320's -
    // the same sound as the model's case.
    struct Fighter {
        const char *name;
        float power, reheat, rollingMs, airspeedMs, x0, vx, z;
        float earY, earZ;
        uint32_t expect;
    };
    const Fighter FIGHTERS[] = {
        {"take-off roll, reheat", 1.0f, 1, 50, 50, -250, 50, 0, 150, 4, 0x69276a23u},
        {"taxiing past", 0.14f, 0, 8, 8, -40, 8, 0, 35, 1.5f, 0x70194abcu},
    };
    for (const Fighter &e : FIGHTERS) {
        engine_noise::Synth *synth = new (room) engine_noise::Synth();
        constexpr int BLOCKS = 250;  // ten seconds
        uint32_t us = 0, sum = 2166136261u;
        for (int i = 0; i < BLOCKS; i++) {
            engine_noise::Sound s;
            s.kind = engine_noise::Kind::LOW_BYPASS;
            s.engine = engine_noise::Engine::EJ200;
            s.engines = 2;
            s.size = 0.425f;
            s.who = 0x43c6f1;
            s.power = e.power;
            s.reheat = e.reheat;
            s.afterburner = e.reheat > 0;
            s.rollingMs = e.rollingMs;
            s.airspeedMs = e.airspeedMs;
            s.gearDown = true;
            s.placed = true;
            s.at[0] = e.x0 + e.vx * i * 0.04f, s.at[2] = e.z + 1.5f;
            s.ear[1] = e.earY, s.ear[2] = e.earZ;
            s.earUpM = s.ear[2];
            s.sourceUpM = s.at[2];
            s.hardGround = e.rollingMs > 0;
            s.ms = (uint32_t)i * 40;
            uint32_t t0 = micros();
            synth->set(s);
            synth->render(block, ENGINE_BLOCK);
            us += micros() - t0;
            for (int k = 0; k < ENGINE_BLOCK; k++) {
                sum = (sum ^ (uint16_t)block[k]) * 16777619u;
            }
        }
        synth->~Synth();
        Serial.printf("[bench] Typhoon %s: %.3fms a 40ms block, %.2f%% of a core; its sound %08lx, %s\n", e.name,
                      us / 1000.0f / BLOCKS, us / (BLOCKS * 400.0f), (unsigned long)sum,
                      sum == e.expect ? "as the model's" : "CHANGED");
    }
    heap_caps_free(room);
    heap_caps_free(block);
}
#endif
