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

bool fetchAircraft(double lat, double lon, int radiusNm, bool military, AdsbSource source,
                   std::vector<Aircraft> &out, const char *&usedProvider);
