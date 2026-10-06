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

// The airport zoom fetches the traffic around its airport this often, on top
// of the main poll. Faster than the main poll's default, since at a couple of
// miles across an airliner on final covers a third of the plot in thirty
// seconds - but only while the zoom is up, which stays until Back or the
// screen is left, and for a radius of a few miles, a far smaller answer than
// the main poll's. The two never go closer together than
// POLL_INTERVAL_MIN_S. Ten seconds rather than five: at five, sustained over
// a landing, adsb.lol began answering 429 and the poll fell back to adsb.fi.
constexpr int ZOOM_POLL_INTERVAL_S = 10;
// Between its polls the zoom - and the radar too - is redrawn this many
// times more, each contact moved on from where the last poll put it by its
// speed and track, and a followed aircraft by the estimate that takes it down
// a runway: so it moves in steps of a few seconds rather than jumping at each
// poll. At least every TWEEN_MAX_MS, so the radar's half-minute polls get
// more of them than the zoom's ten seconds. Not past two polls' worth: a feed
// that has stopped answering doesn't send the sky sailing on without it.
constexpr int TWEEN_FRAMES = 2;
constexpr uint32_t TWEEN_MAX_MS = 5000;
// Fetched past the zoom's range, so traffic a few miles out on final is in
// the list: off the plot, but it says which runway is in use before it lands.
constexpr int ZOOM_FETCH_EXTRA_NM = 4;
// A runway end stays marked as in use - its arrow dimmed, and listed as
// "last in use" - this long after anything in the air was last lined up on
// it: through a lull, it says which way the airport is working.
constexpr uint32_t RUNWAY_IN_USE_MEMORY_MS = 30UL * 60 * 1000;

// Follow me: the radar centred on one aircraft, at this range, fetched around
// it by the zoom's extra poll wherever it goes, in or out of the radius. On
// the radar that goes at the main poll's interval, so following costs one
// more request per poll; only once it is handed to an airport's zoom does it
// go at ZOOM_POLL_INTERVAL_S.
constexpr int FOLLOW_RANGE_NM = 20;
// Coming down to an airport, the range closes in on it a step at a time -
// and the fetch around the aircraft with it, so the answers shrink too. Below
// this height, and only then, an airport ahead is taken for its destination.
constexpr int FOLLOW_RANGE_STEPS_NM[] = {5, 10, 15};
// The radar's ZOOM IN and OUT step through these - so at the default 25nm,
// the same 5, 10, 15 and 20 as following closes in through on a landing.
// Inside the poll radius only what is shown changes, the poll, the table and
// the alerts going on covering the whole radius; out past it the radius
// itself widens, to the settings' ceiling; in from the closest is the
// airport's own zoom.
constexpr int RADAR_RANGE_STEPS_NM[] = {5, 10, 15, 20, 30, 40, 60, 80, 100, 120};
constexpr int FOLLOW_APPROACH_MAX_FT = 12000;
// Level rather than descending, only this low: above it, an airport ahead is
// one it is passing over.
constexpr int FOLLOW_APPROACH_LEVEL_FT = 5000;
// Handed to an airport's zoom once it is low near one and on the zoom's plot:
// within this height of the field, not climbing, and inside the zoom's range
// of its middle - or will be by the next fetch, at the speed it is doing. On
// the ground counts too, so a departure starts there.
constexpr int FOLLOW_ZOOM_AGL_FT = 2500;
constexpr float FOLLOW_ZOOM_SEARCH_NM = 10.0f;  // how far off an airport may be, to be considered
// And back to the radar once it is airborne and flying off the zoom's plot,
// or whichever way it is going, this many zoom ranges out.
constexpr float FOLLOW_UNZOOM_RANGES = 1.5f;
// Gone from the feed this long - transponder off at the gate, or out of
// every receiver's reach - and following stops.
constexpr uint32_t FOLLOW_LOST_MS = 90000;
// While it is lost, how often the estimate of where it is is redrawn.
constexpr uint32_t FOLLOW_ESTIMATE_DRAW_MS = 1000;

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
// An alert's recording that following takes over within this long of its
// start is removed rather than kept: the follow's own recording has it all.
constexpr uint32_t ALERT_STUB_MS = 60000;
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

// How old a last-known position may be and still be plotted, dimmed, when
// the feed has lost the current one.
constexpr float STALE_POSITION_MAX_S = 60.0f;
// Any contact this low - landing, taking off, on the ground - that the feed
// loses is drawn where it is estimated to be, dimmed, for up to this long:
// the same estimate as a followed aircraft's (see follow.h), which follows
// one the longer FOLLOW_LOST_MS. Higher up, a contact that goes missing has
// most likely flown out of range, and is let go.
constexpr int ESTIMATE_MAX_FT = 4000;
constexpr uint32_t ESTIMATE_LOST_MS = 90000;
// One whose take-off or landing is being simulated (see RunwaySim) is kept
// longer: stopped at the end of its rollout, it can be minutes before a
// receiver picks it up taxiing in.
constexpr uint32_t RUNWAY_SIM_LOST_MS = 180000;

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

// How long a write over USB serial waits on a host that isn't reading. The
// framework's 100ms is tried twenty times over before it gives up on the
// host, and that was a 2-second freeze of the screen and its touches each time
// a computer started reading the port - or was plugged into for power with
// nothing reading it. The screenshot dump puts the 100ms back while it runs.
constexpr uint32_t SERIAL_TX_TIMEOUT_MS = 10;
constexpr uint32_t SERIAL_TX_TIMEOUT_DUMP_MS = 100;
