#include "settings.h"

#include <Preferences.h>

#include "alerts.h"
#include "config.h"

namespace {
constexpr const char *NAMESPACE = "overhead";
}

AppSettings loadSettings() {
    Preferences prefs;
    prefs.begin(NAMESPACE, true);
    AppSettings s;
    // isKey() first for these, as for the WiFi keys below: a missing double
    // or string logs an ESP error line rather than quietly taking the default.
    s.lat = prefs.isKey("lat") ? prefs.getDouble("lat", DEFAULT_LAT) : DEFAULT_LAT;
    s.lon = prefs.isKey("lon") ? prefs.getDouble("lon", DEFAULT_LON) : DEFAULT_LON;
    s.label = prefs.isKey("label") ? prefs.getString("label", DEFAULT_LABEL) : String(DEFAULT_LABEL);
    s.traffic = prefs.getBool("milonly", false) ? TrafficFilter::MILITARY : TrafficFilter::CIVIL;
    s.radiusNm = prefs.getInt("radius", DEFAULT_RADIUS_NM);
    s.showRefresh = prefs.getBool("refresh", true);
    s.navaids = prefs.getBool("navaids", true);
    s.pollIntervalS = prefs.getInt("pollint", DEFAULT_POLL_INTERVAL_S);
    s.muted = prefs.getBool("muted", false);
    s.source = (AdsbSource)prefs.getUChar("source", (uint8_t)DEFAULT_ADSB_SOURCE);
    s.radarCentre = prefs.getUChar("radarctr", (uint8_t)DEFAULT_RADAR_CENTRE) == (uint8_t)RadarCentre::HOME
                        ? RadarCentre::HOME
                        : RadarCentre::AIRPORT;
    s.radarAirports = prefs.getBool("radarapt", false);
    s.radarRangeNm = prefs.getUShort("radarrng", 0);
    s.alertMask = prefs.getUChar("alerts", ALERT_ALL) & ALERT_ALL;
    s.watchlist = prefs.isKey("watch") ? prefs.getString("watch") : String("");
    s.autoRecord = prefs.getBool("autorec", false);
    s.autoFollow = prefs.getBool("autofollow", false);
    // isKey() first: getString() on a missing key logs an ESP error line, and
    // an unconfigured device would print two of them on every boot.
    s.wifiSsid = prefs.isKey("ssid") ? prefs.getString("ssid") : String("");
    s.wifiPass = prefs.isKey("pass") ? prefs.getString("pass") : String("");
    prefs.end();

    // A stored value from a build that listed more providers than this one
    // would index off the end of the table, so anything unrecognised falls
    // back to letting the tracker choose.
    if (adsbSourceIndex(s.source) >= ADSB_PROVIDER_COUNT) {
        s.source = AdsbSource::AUTO;
    }

    if (s.radiusNm < RADIUS_MIN_NM) {
        s.radiusNm = RADIUS_MIN_NM;
    }
    int maxNm = (s.traffic == TrafficFilter::MILITARY) ? MILITARY_MAX_RADIUS_NM : CIVIL_MAX_RADIUS_NM;
    if (s.radiusNm > maxNm) {
        s.radiusNm = maxNm;
    }
    // Snapped as well as clamped: a value saved by an older build, or a step
    // that has since changed, shouldn't leave the slider between detents.
    s.pollIntervalS = (s.pollIntervalS + POLL_INTERVAL_STEP_S / 2) / POLL_INTERVAL_STEP_S * POLL_INTERVAL_STEP_S;
    if (s.pollIntervalS < POLL_INTERVAL_MIN_S) {
        s.pollIntervalS = POLL_INTERVAL_MIN_S;
    }
    if (s.pollIntervalS > POLL_INTERVAL_MAX_S) {
        s.pollIntervalS = POLL_INTERVAL_MAX_S;
    }
    return s;
}

void saveLocation(double lat, double lon, const String &label) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putDouble("lat", lat);
    prefs.putDouble("lon", lon);
    prefs.putString("label", label);
    prefs.end();
}

void saveFilters(TrafficFilter traffic, int radiusNm, bool showRefresh, int pollIntervalS,
                 AdsbSource source) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("milonly", traffic == TrafficFilter::MILITARY);
    prefs.putInt("radius", radiusNm);
    prefs.putBool("refresh", showRefresh);
    prefs.putInt("pollint", pollIntervalS);
    prefs.putUChar("source", (uint8_t)source);
    prefs.end();
}

void saveNavaids(bool navaids) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("navaids", navaids);
    prefs.end();
}

void saveMuted(bool muted) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("muted", muted);
    prefs.end();
}

void saveRadarCentre(RadarCentre centre) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putUChar("radarctr", (uint8_t)centre);
    prefs.end();
}

void saveRadarAirports(bool airports) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("radarapt", airports);
    prefs.end();
}

void saveRadarRange(int rangeNm) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putUShort("radarrng", (uint16_t)rangeNm);
    prefs.end();
}

void saveWifi(const String &ssid, const String &pass) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
}

void saveAlerts(uint8_t alertMask, const String &watchlist, bool autoRecord, bool autoFollow) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putUChar("alerts", alertMask);
    prefs.putString("watch", watchlist);
    prefs.putBool("autorec", autoRecord);
    prefs.putBool("autofollow", autoFollow);
    prefs.end();
}
