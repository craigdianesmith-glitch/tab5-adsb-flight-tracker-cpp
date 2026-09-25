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
    bool hasDist;
    float distNm;
    String status;   // CLIMB / DESCEND / LEVEL / TAXI / GROUND
    bool military;   // readsb dbFlags bit 0, or which feed it came from

    // Extra detail-view fields - not shown in the main table row.
    String reg;
    String squawk;
    String category;
    bool hasTrack;
    float track;      // degrees, ground track ("heading")
    bool hasVertRate;
    float vertRate;   // ft/min, signed
    bool hasAltGeom;
    int altGeom;       // ft
    bool hasPos;
    float lat, lon;    // aircraft's own position
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
bool fetchAircraft(double lat, double lon, int radiusNm, bool military, AdsbSource source,
                   std::vector<Aircraft> &out, const char *&usedProvider);
