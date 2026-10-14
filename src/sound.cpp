#include "sound.h"

#include <M5Unified.h>
#include <math.h>

#include "aircraft_db.h"
#include "alerts.h"
#include "config.h"
#include "regions.h"

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
// An announcement or a reading has no chime: only words, played as an
// alert's are.
constexpr Note NO_NOTES[] = {{0, 0}};
const Note *g_notes = nullptr;
int g_noteCount = 0;
int g_nextNote = 0;

// The words, made by tools/gen_voice.py in each of three voices: a second or
// so of 8-bit sound each, at the voice's own rate, played from flash by the
// speaker's own task, so saying one costs the UI no more than a note does.
struct Clip {
    const int8_t *samples;
    size_t count;
};
struct NamedClip {
    const char *name;
    Clip clip;
};
// Every word in one voice, in the order tools/gen_voice.py writes them.
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
#include "voice_scottish.inc"
#include "voice_english.inc"
#include "voice_american.inc"
#include "jet_table.inc"
#include "click_table.inc"

// In Accent's order.
const Voice *const VOICES[] = {&voice_scottish::VOICE, &voice_english::VOICE, &voice_american::VOICE};
Accent g_elsewhere = DEFAULT_ELSEWHERE_ACCENT;
bool g_readAloud = false;

// What is said about somewhere is said in its voice: Scotland's Scottish,
// England's English, the United States' American, and everywhere else's the
// one chosen for it.
const Voice *voiceOf(Region region) {
    switch (region) {
    case Region::SCOTLAND: return VOICES[(int)Accent::SCOTTISH];
    case Region::ENGLAND: return VOICES[(int)Accent::ENGLISH];
    case Region::USA: return VOICES[(int)Accent::AMERICAN];
    default: return VOICES[(int)g_elsewhere];
    }
}

// What is said once the notes are done, a clip at a time - for an alert, the
// reason, then the callsign spelt out - each with the pause to leave after
// it, in the voice g_voice.
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
const Voice *g_voice = VOICES[0];
// Whether what is being said is a reading - see soundReadAircraft() - which
// the next one cuts short, where it waits for an alert or an announcement.
bool g_readingNow = false;

// The reason the banner names, in the same order - see alertReasonText().
bool wordFor(uint8_t reasons, Clip &out) {
    if (reasons & ALERT_EMERGENCY) out = g_voice->emergency;
    else if (reasons & ALERT_WATCHLIST) out = g_voice->watchlist;
    else if (reasons & ALERT_RARE) out = g_voice->rare;
    else if (reasons & ALERT_MILITARY) out = g_voice->military;
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
            say(g_voice->letters[c - 'A']);
        } else if (c >= '0' && c <= '9') {
            say(g_voice->digits[c - '0']);
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
        for (size_t i = 0; i < g_voice->airlineCount; i++) {
            if (airline.name == g_voice->airlines[i].name) {
                voice = &g_voice->airlines[i];
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
    for (size_t i = 0; i < g_voice->makerCount; i++) {
        const NamedClip &m = g_voice->makers[i];
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
    if (neo) say(g_voice->neo);
    if (max) say(g_voice->max);
    if (dreamliner) say(g_voice->dreamliner);
    return true;
}

// A frequency as a controller gives it: "one one eight decimal eight", the
// MHz and then its fraction without the zeros at its end - all three digits
// of an 8.33kHz channel's, "one three two decimal eight zero five".
void sayFrequency(uint32_t khz) {
    spell(String(khz / 1000));
    say(g_voice->decimal);
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
        say(g_voice->flightLevel);
        spell(String((ft + 50) / 100));
        return;
    }
    long hundreds = (ft + 50) / 100;
    if (hundreds >= 10) {
        spell(String(hundreds / 10));  // "one two thousand", as the digits
        say(g_voice->thousand);
    }
    if (hundreds % 10) {
        spell(String(hundreds % 10));
        say(g_voice->hundred);
    }
    if (hundreds == 0) {
        spell("0");
    }
    say(g_voice->feet);
}

// The airport zoom's announcements waiting their turn, behind an alert or
// one another - a few at most, as an airport's movements come minutes apart.
struct Announcement {
    String callsign;
    String runway;
    bool takeoff;
    uint32_t queuedMs;
    const Voice *voice;
};
// Waiting behind an alert's words longer than this, it is over by the time it
// would be said - an aircraft cleared for take-off is in the air - and dropped.
constexpr uint32_t ANNOUNCE_STALE_MS = 8000;
constexpr int MAX_ANNOUNCEMENTS = 3;
Announcement g_announcements[MAX_ANNOUNCEMENTS];
int g_announcementCount = 0;

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
    g_voice = a.voice;
    sayCallsign(a.callsign);
    pauseAfter(AFTER_CALLSIGN_MS);
    say(g_voice->runway);
    for (size_t i = 0; i < a.runway.length(); i++) {
        char c = a.runway[i];
        if (c >= '0' && c <= '9') {
            say(g_voice->digits[c - '0']);
        } else if (c == 'L') {
            say(g_voice->left);
        } else if (c == 'R') {
            say(g_voice->right);
        } else if (c == 'C') {
            say(g_voice->centre);
        }
    }
    pauseAfter(AFTER_CALLSIGN_MS);
    say(a.takeoff ? g_voice->clearedTakeoff : g_voice->clearedLand, 0);
    Serial.printf("[sound] announcing %s, runway %s, cleared %s: %d spoken\n", a.callsign.c_str(), a.runway.c_str(),
                  a.takeoff ? "for take-off" : "to land", g_spokenCount);
    speakQueued(false);
}

// Read aloud - see soundReadAircraft(): the latest asked for, waiting until
// what is being said has been, unless that is a reading too.
enum class ReadKind : uint8_t { NONE, AIRCRAFT, NAVAID, AIRPORT, SAMPLE };
struct Reading {
    ReadKind kind = ReadKind::NONE;
    Aircraft aircraft;
    const Navaid *navaid = nullptr;
    const AirportInfo *airport = nullptr;
    Region region = Region::ELSEWHERE;
    const Voice *voice = nullptr;
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
        say(g_voice->onGround, 0);
        return;
    }
    if (a.status == "CLIMB") {
        say(g_voice->climbing, AFTER_MAKER_MS);
    } else if (a.status == "DESCEND") {
        say(g_voice->descending, AFTER_MAKER_MS);
    } else if (a.status == "LEVEL") {
        say(g_voice->level, AFTER_MAKER_MS);
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
        say(g_voice->knots, AFTER_CALLSIGN_MS);
    }
    if (a.hasTrack) {
        int heading = (int)lroundf(a.track) % 360;
        char digits[4];
        snprintf(digits, sizeof(digits), "%03d", heading == 0 ? 360 : heading);
        say(g_voice->heading);
        spell(digits);
    }
    pauseAfter(0);
}

// "Golf Oscar Whiskey, V O R D M E, one one five decimal four" - or a
// TACAN's channel, "channel one zero one X-ray".
void readNavaid(const Navaid &n) {
    spell(n.ident);
    pauseAfter(AFTER_CALLSIGN_MS);
    say(g_voice->navaidTypes[(int)n.type], AFTER_CALLSIGN_MS);
    if (n.type == NavaidType::TACAN && n.channel[0]) {
        say(g_voice->channel);
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
        say(g_voice->atcUses[f[i].use], AFTER_MAKER_MS);
        sayFrequency(f[i].khz);
        pauseAfter(AFTER_CALLSIGN_MS);
    }
    pauseAfter(0);
}

void startReading(const Reading &r) {
    g_spokenCount = g_nextSpoken = 0;
    g_voice = r.voice;
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
    case ReadKind::SAMPLE:
        // "Runway two three, cleared for take-off."
        say(g_voice->runway);
        spell("23");
        pauseAfter(AFTER_CALLSIGN_MS);
        say(g_voice->clearedTakeoff, 0);
        break;
    case ReadKind::NONE:
        return;
    }
    static const char *const KINDS[] = {"", "aircraft", "navaid", "airport", "sample"};
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
}

void soundSetMuted(bool muted) { g_muted = muted; }

bool soundMuted() { return g_muted; }

void soundSetElsewhereAccent(Accent accent) {
    if ((int)accent < (int)(sizeof(VOICES) / sizeof(VOICES[0]))) {
        g_elsewhere = accent;
    }
}

void soundSetReadAloud(bool on) { g_readAloud = on; }

void soundTakeoff() {
    if (!g_ready || g_muted) {
        return;
    }
    // Played from flash by the speaker's own task, as the words are.
    M5.Speaker.playRaw(JET_TAKEOFF, sizeof(JET_TAKEOFF) / sizeof(JET_TAKEOFF[0]), JET_RATE, false, 1, CHANNEL, true);
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

void soundAlert(uint8_t reasons, const String &callsign, const String &typeName, float lat, float lon) {
    if (!g_ready || g_muted) {
        Serial.printf("[sound] alert for %s not sounded: %s\n", callsign.c_str(), g_muted ? "muted" : "no speaker");
        return;
    }
    bool emergency = (reasons & ALERT_EMERGENCY) != 0;
    g_spokenCount = g_nextSpoken = 0;
    g_voice = voiceOf(regionNear(lat, lon));
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
    g_notes = emergency ? WARBLE : CHIME;
    // A line for each, as the speaker can go silent with nothing else to
    // show for it - see the README.
    Serial.printf("[sound] alert for %s: %s, then %d spoken\n", callsign.c_str(), emergency ? "warble" : "chime",
                  g_spokenCount);
    g_noteCount = emergency ? (int)(sizeof(WARBLE) / sizeof(Note)) : (int)(sizeof(CHIME) / sizeof(Note));
    g_nextNote = 0;
}

void soundAnnounce(const String &callsign, const String &runway, bool takeoff, const char *airport) {
    String cs = callsign;
    cs.trim();
    if (!g_ready || g_muted || cs.length() == 0 || runway.length() == 0) {
        return;
    }
    if (g_announcementCount == MAX_ANNOUNCEMENTS) {
        Serial.printf("[sound] announcement for %s dropped: %d waiting\n", cs.c_str(), g_announcementCount);
        return;
    }
    const Voice *voice = voiceOf(airport ? airportRegion(airport) : Region::ELSEWHERE);
    g_announcements[g_announcementCount++] = {cs, runway, takeoff, millis(), voice};
}

void soundReadAircraft(const Aircraft &ac, bool always) {
    if (!g_readAloud && !always) {
        return;
    }
    Reading r;
    r.kind = ReadKind::AIRCRAFT;
    r.aircraft = ac;
    r.region = ac.hasPos ? regionNear(ac.lat, ac.lon) : Region::ELSEWHERE;
    r.voice = voiceOf(r.region);
    queueReading(r);
}

void soundReadNavaid(const Navaid &navaid, bool always) {
    if (!g_readAloud && !always) {
        return;
    }
    Reading r;
    r.kind = ReadKind::NAVAID;
    r.navaid = &navaid;
    r.voice = voiceOf(regionNear(navaid.lat, navaid.lon));
    queueReading(r);
}

void soundReadAirport(const AirportInfo &airport, bool always) {
    if ((!g_readAloud && !always) || airport.frequencyCount == 0) {
        return;
    }
    Reading r;
    r.kind = ReadKind::AIRPORT;
    r.airport = &airport;
    r.voice = voiceOf(airportRegion(airport.code));
    queueReading(r);
}

void soundStopReading() {
    g_reading = Reading();
    if (g_readingNow && g_notes != nullptr) {
        M5.Speaker.stop(ALERT_CHANNEL);
        g_notes = nullptr;
        g_spokenCount = g_nextSpoken = 0;
        g_readingNow = false;
    }
}

void soundSampleAccent(Accent accent) {
    if ((int)accent >= (int)(sizeof(VOICES) / sizeof(VOICES[0]))) {
        return;
    }
    Reading r;
    r.kind = ReadKind::SAMPLE;
    r.voice = VOICES[(int)accent];
    queueReading(r);
}

void soundTick() {
    // Nothing being said: the next announcement - a clearance is only worth
    // hearing as it happens, and goes stale in seconds - or else a reading
    // asked for, which can wait the few seconds one takes.
    if (g_notes == nullptr && !M5.Speaker.isPlaying(ALERT_CHANNEL)) {
        if (g_announcementCount > 0) {
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
            const Spoken &s = g_spoken[g_nextSpoken++];
            M5.Speaker.playRaw(s.clip.samples, s.clip.count, g_voice->rate, false, 1, ALERT_CHANNEL, true);
            g_speakAtMs = millis() + s.clip.count * 1000 / g_voice->rate + s.pauseMs;
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
