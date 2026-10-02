#pragma once

constexpr double DEFAULT_LAT = 55.9297;
constexpr double DEFAULT_LON = -4.4664;
constexpr const char *DEFAULT_LABEL = "Erskine, UK";
constexpr int DEFAULT_RADIUS_NM = 25;

// The radius slider runs to the military ceiling; civil traffic is only ever
// shown out to the lower one, however far the slider is pushed.
constexpr int RADIUS_MIN_NM = 5;
constexpr int CIVIL_MAX_RADIUS_NM = 60;
constexpr int MILITARY_MAX_RADIUS_NM = 150;

// How often the sky is refetched, set on the settings screen. Whole seconds
// in fives: below five the endpoint starts refusing, and past a minute the
// table is stale enough that a slower dial wouldn't be asked for.
constexpr int POLL_INTERVAL_MIN_S = 5;
constexpr int POLL_INTERVAL_MAX_S = 60;
constexpr int POLL_INTERVAL_STEP_S = 5;
// Thirty rather than ten: adsb.lol is free, volunteer-run infrastructure, and
// a default is what most devices will actually run at. Aircraft move perhaps
// three miles in that time, which at these ranges moves a blip by a few
// pixels - so the dial is there for anyone who wants it faster on their own
// account, rather than every device taking it by default.
constexpr int DEFAULT_POLL_INTERVAL_S = 30;

constexpr uint32_t FORGET_AFTER_MS = 120000;

// How long a cell that just changed value stays shaded.
constexpr uint32_t CELL_HIGHLIGHT_MS = 2000;

// Whether the speaker is wired up at all; muting from the header is separate
// and persists, where this is a build-time "never make a sound".
constexpr bool SOUND_ENABLED = true;
constexpr uint8_t SOUND_VOLUME = 96;  // 0-255
// Short and high, so it reads as a tick rather than a beep. The channel is
// already at full volume, so loudness comes from length: at 12ms it was too
// brief to register as loud as the master volume allows.
constexpr uint16_t KEY_CLICK_HZ = 3000;
constexpr uint32_t KEY_CLICK_MS = 25;

// What the radar plot is centred on: the configured location, or the airport
// nearest to it. Stored in NVS by value, so the numbers are not free to change.
enum class RadarCentre : uint8_t { HOME = 0, AIRPORT = 1 };
constexpr RadarCentre DEFAULT_RADAR_CENTRE = RadarCentre::AIRPORT;

// --- alerts and recording ---------------------------------------------------

// How long the alert banner stays up once it has been on screen, if nobody
// taps it first.
constexpr uint32_t ALERT_BANNER_MS = 20000;
// An auto recording runs on this long after the last alerted aircraft has
// gone, so a contact that drops out for a poll or two doesn't split one
// sighting into several files.
constexpr uint32_t AUTO_RECORD_TAIL_MS = 60000;
// Room for a dozen or so entries; it is typed on the on-screen keyboard.
constexpr size_t WATCHLIST_MAX_LEN = 160;
// A recording carries on in a new file - its next part - once the one it is
// writing reaches either of these. Opening a recording reads all of it to
// index it, so a file left to grow for a day would take most of a minute to
// open, and past half a million positions wouldn't fit the index at all. Size
// is what decides that, so it is capped directly; the time keeps the parts of
// a quiet recording to a length that reads sensibly in the list.
constexpr uint32_t RECORDING_SPLIT_MS = 4UL * 60 * 60 * 1000;
constexpr uint32_t RECORDING_SPLIT_BYTES = 4UL * 1024 * 1024;
// Where exported videos go: the top of the card rather than beside the
// recordings, to be easy to find on a computer. A recording's videos are named
// after it - 20261002-143155.rec gives 20261002-143155-4x.mp4.
constexpr const char *VIDEO_DIR = "/videos";
// How far back a contact's trail reaches during playback.
constexpr uint32_t REPLAY_TRAIL_MS = 10 * 60 * 1000;

constexpr int CLIMB_THRESHOLD_FPM = 150;
constexpr int DESCEND_THRESHOLD_FPM = -150;

// --- data sources ---------------------------------------------------------
//
// One source used to be enough, until adsb.lol spent an evening answering 200
// OK with an empty aircraft array: its feed had drained while its API stayed
// up, and at the point of parsing that is indistinguishable from an empty sky.
// So the source is a setting, and AUTO checks an empty result against a second
// one before it reaches the screen.
//
// A source is a pair of endpoints rather than a single URL, because providers
// disagree about more than their hostname: the array the aircraft arrive in is
// called `ac` by one and `aircraft` by another, only some carry readsb's
// dbFlags, and a military feed may be global rather than radius-limited.
struct AdsbEndpoint {
    const char *urlTemplate;  // {lat}, {lon} and {radius} are substituted in
    const char *arrayKey;     // top-level key the aircraft array sits under
    bool global;              // ignores {radius}; distance is filtered locally
    // readsb's dbFlags, whose bit 0 marks military. Where an endpoint omits it
    // there is nothing to test, so the endpoint itself is the answer: all of a
    // military feed is military, none of a civil one is.
    bool hasDbFlags;
};

struct AdsbProvider {
    const char *name;
    AdsbEndpoint civil;
    AdsbEndpoint military;
};

// adsb.lol's point endpoint carries dbFlags, so the one URL serves both
// filters. adsb.fi's doesn't, but its /v2/mil does - and returns it under `ac`
// where its point query uses `aircraft`, so those two really are separate
// endpoints rather than one path with a variant. /v2/mil is global, which the
// distance filter the app already applies takes care of.
constexpr AdsbProvider ADSB_PROVIDERS[] = {
    {"adsb.lol",
     {"https://api.adsb.lol/v2/point/{lat}/{lon}/{radius}", "ac", false, true},
     {"https://api.adsb.lol/v2/point/{lat}/{lon}/{radius}", "ac", false, true}},
    {"adsb.fi",
     {"https://opendata.adsb.fi/api/v2/lat/{lat}/lon/{lon}/dist/{radius}", "aircraft", false, false},
     {"https://opendata.adsb.fi/api/v2/mil", "ac", true, true}},
};
constexpr int ADSB_PROVIDER_COUNT = 2;

// Which source the tracker polls. AUTO moves on when one fails or reports an
// empty sky another disagrees with; the named values pin it, for when you want
// to know which one you are looking at rather than have it chosen for you.
// Stored in NVS by value, so the numbers are not free to change.
enum class AdsbSource : uint8_t { AUTO = 0, ADSB_LOL = 1, ADSB_FI = 2 };
constexpr AdsbSource DEFAULT_ADSB_SOURCE = AdsbSource::AUTO;

// Index into ADSB_PROVIDERS, or -1 for AUTO.
constexpr int adsbSourceIndex(AdsbSource s) { return s == AdsbSource::AUTO ? -1 : (int)s - 1; }
