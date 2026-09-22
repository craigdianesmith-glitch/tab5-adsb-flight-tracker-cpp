#pragma once

#include <Arduino.h>
#include <vector>

struct Aircraft {
    String hex;
    String callsign;
    String type;
    String altStr;
    String speedStr;
    bool hasDist;
    float distNm;
    String status;  // CLIMB / DESCEND / LEVEL / TAXI / GROUND

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

// Fetches aircraft near (lat, lon) within radiusNm. Returns false (out left
// untouched) on any network/HTTP/parse failure, so callers can keep showing
// the last-known-good data rather than clearing it on a transient error.
bool fetchAircraft(double lat, double lon, int radiusNm, std::vector<Aircraft> &out);
