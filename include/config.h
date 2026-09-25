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
constexpr int DEFAULT_POLL_INTERVAL_S = 10;

constexpr uint32_t FORGET_AFTER_MS = 120000;

// How long a cell that just changed value stays shaded.
constexpr uint32_t CELL_HIGHLIGHT_MS = 2000;

// Whether the speaker is wired up at all; muting from the header is separate
// and persists, where this is a build-time "never make a sound".
constexpr bool SOUND_ENABLED = true;
constexpr uint8_t SOUND_VOLUME = 96;  // 0-255

constexpr int CLIMB_THRESHOLD_FPM = 150;
constexpr int DESCEND_THRESHOLD_FPM = -150;

constexpr const char *ADSB_API_URL_FMT = "https://api.adsb.lol/v2/point/%.4f/%.4f/%d";
