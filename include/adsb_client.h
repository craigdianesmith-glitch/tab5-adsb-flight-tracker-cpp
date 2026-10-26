#pragma once

#include <Arduino.h>
#include <vector>

#include "config.h"

struct Aircraft {
    String hex;
    String callsign;
    String type;
    String altStr;
    String speedStr;
    bool hasDist = false;
    float distNm = 0;
    String status;   // CLIMB / DESCEND / LEVEL / TAXI / GROUND
    bool military = false;  // readsb dbFlags bit 0, or which feed it came from
    // readsb dbFlags bit 1: the feed's own database marks the airframe as
    // interesting. Only feeds that carry dbFlags can say so.
    bool dbInteresting = false;
    // AlertReason bits, set by the poll task once the aircraft has been
    // checked against the alert rules. 0 for an ordinary contact.
    uint8_t alert = 0;

    // Extra detail-view fields - not shown in the main table row.
    String reg;
    String squawk;
    String category;
    // Defaulted so a default-constructed Aircraft - a placeholder before a
    // lookup fills it - never carries garbage a has* flag would vouch for.
    bool hasTrack = false;
    float track = 0;      // degrees, ground track ("heading")
    bool hasVertRate = false;
    float vertRate = 0;   // ft/min, signed
    bool hasAltGeom = false;
    int altGeom = 0;      // ft
    bool hasPos = false;
    float lat = 0, lon = 0;  // aircraft's own position
    // The position is the last one the feed had rather than a current one:
    // drawn dimmed. See STALE_POSITION_MAX_S.
    bool posStale = false;
};

// Fetches aircraft near (lat, lon) within radiusNm from `source`, asking that
// provider's civil or military endpoint as `military` says. Returns false (out
// left untouched) on any network/HTTP/parse failure, so callers can keep
// showing the last-known-good data rather than clearing it on a transient
// error - and under AdsbSource::AUTO it only returns false once every provider
// has failed.
//
// usedProvider is set to the name of whichever provider actually answered,
// which under AUTO is not necessarily the first one asked. It points into
// ADSB_PROVIDERS and so outlives the call; it is left alone on failure.
// Great-circle distance in nautical miles: what a contact's distNm holds,
// measured from the point it was fetched around.
float haversineNm(double lat1, double lon1, double lat2, double lon2);

// The speed and height rule: whether a report with a height in the air and a
// groundspeed of `gsKt` is of nothing that flies - standing still up there,
// which only what can hover does (see canHover()). Anything else reported so
// is a transponder left on, parked, that doesn't say it is on the ground, or
// one reporting nonsense - a Typhoon was seen with a height and nought knots
// - and is left out of every poll, whatever its callsign or type, and of a
// replay of a recording made before it was.
bool stillInTheAir(float gsKt, const String &type, const String &category);

bool fetchAircraft(double lat, double lon, int radiusNm, bool military, AdsbSource source,
                   std::vector<Aircraft> &out, const char *&usedProvider);

// How the requests to the feed have gone lately, for the WiFi watchdog (see
// wifi_watch.h): when a provider last answered at all, and how many requests
// have failed at the network since. "Answered" is any HTTP status - a 429 or
// a 503 is a server that was reached, which is all the watchdog asks - and a
// reply that came but would not parse is answered too. A network failure is
// a request that never got a reply: the name not looked up, the connection
// or its TLS not made, nothing back in the time allowed, the connection lost
// part way, or a reply cut off by the 30s cap. A request the 429 backoff
// held back was never made, and counts as neither.
//
// Both the main poll and the zoom's go through here, and only the poll task
// makes requests, so only it should read this.
struct AdsbFeedHealth {
    uint32_t requests = 0;         // requests made since boot
    uint32_t lastAnswerMs = 0;     // millis() a provider last answered; 0 never
    uint32_t netFails = 0;         // network failures in a row since then
    uint32_t netFailsSinceMs = 0;  // when the first of them was
    uint32_t lastNetFailMs = 0;
    const char *lastNetFail = "";  // the last one, in words
};
const AdsbFeedHealth &adsbFeedHealth();

// Closes the kept-alive connection to the feed. For the watchdog, after it
// has rejoined the network or restarted the WiFi: the socket is from before,
// and the next request is better made afresh than left to find that out.
// From the poll task only, as every request is.
void adsbDropConnection();
