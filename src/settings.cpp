#include "settings.h"

#include <Preferences.h>

#include "config.h"

namespace {
constexpr const char *NAMESPACE = "overhead";
}

AppSettings loadSettings() {
    Preferences prefs;
    prefs.begin(NAMESPACE, true);
    AppSettings s;
    s.lat = prefs.getDouble("lat", DEFAULT_LAT);
    s.lon = prefs.getDouble("lon", DEFAULT_LON);
    s.label = prefs.getString("label", DEFAULT_LABEL);
    s.traffic = prefs.getBool("milonly", false) ? TrafficFilter::MILITARY : TrafficFilter::CIVIL;
    s.radiusNm = prefs.getInt("radius", DEFAULT_RADIUS_NM);
    s.showRefresh = prefs.getBool("refresh", true);
    s.pollIntervalS = prefs.getInt("pollint", DEFAULT_POLL_INTERVAL_S);
    s.muted = prefs.getBool("muted", false);
    // isKey() first: getString() on a missing key logs an ESP error line, and
    // an unconfigured device would print two of them on every boot.
    s.wifiSsid = prefs.isKey("ssid") ? prefs.getString("ssid") : String("");
    s.wifiPass = prefs.isKey("pass") ? prefs.getString("pass") : String("");
    prefs.end();

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

void saveFilters(TrafficFilter traffic, int radiusNm, bool showRefresh, int pollIntervalS) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("milonly", traffic == TrafficFilter::MILITARY);
    prefs.putInt("radius", radiusNm);
    prefs.putBool("refresh", showRefresh);
    prefs.putInt("pollint", pollIntervalS);
    prefs.end();
}

void saveMuted(bool muted) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("muted", muted);
    prefs.end();
}

void saveWifi(const String &ssid, const String &pass) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
}
