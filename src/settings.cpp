#include "settings.h"

#include <Preferences.h>

#include <algorithm>

#include "alerts.h"
#include "config.h"

namespace {
constexpr const char *NAMESPACE = "overhead";

// Military mode, from before the filter had kinds: the same traffic as the
// filter's Military alone - once, before the radius is clamped to a ceiling
// that would otherwise come down under it, and the mode forgotten.
void migrateMilitaryMode() {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    bool military = prefs.isKey("milonly") && prefs.getBool("milonly", false);
    if (prefs.isKey("milonly")) {
        prefs.remove("milonly");
    }
    prefs.end();
    if (military) {
        TrafficFilterSet f = loadTrafficFilter();
        f.civil = false;
        f.cargo = false;
        f.military = true;
        saveTrafficFilter(f);
    }
}
}  // namespace

AppSettings loadSettings() {
    migrateMilitaryMode();
    Preferences prefs;
    prefs.begin(NAMESPACE, true);
    AppSettings s;
    // isKey() first for these, as for the WiFi keys below: a missing double
    // or string logs an ESP error line rather than quietly taking the default.
    s.lat = prefs.isKey("lat") ? prefs.getDouble("lat", DEFAULT_LAT) : DEFAULT_LAT;
    s.lon = prefs.isKey("lon") ? prefs.getDouble("lon", DEFAULT_LON) : DEFAULT_LON;
    s.label = prefs.isKey("label") ? prefs.getString("label", DEFAULT_LABEL) : String(DEFAULT_LABEL);
    s.radiusNm = prefs.getInt("radius", DEFAULT_RADIUS_NM);
    s.showRefresh = prefs.getBool("refresh", true);
    s.navaids = prefs.getBool("navaids", true);
    s.coast = prefs.getBool("coast", true);
    s.rings = prefs.getBool("rings", false);
    s.pollIntervalS = prefs.getInt("pollint", DEFAULT_POLL_INTERVAL_S);
    s.muted = prefs.getBool("muted", false);
    s.source = (AdsbSource)prefs.getUChar("source", (uint8_t)DEFAULT_ADSB_SOURCE);
    s.radarCentre = prefs.getUChar("radarctr", (uint8_t)DEFAULT_RADAR_CENTRE) == (uint8_t)RadarCentre::HOME
                        ? RadarCentre::HOME
                        : RadarCentre::AIRPORT;
    s.radarAirports = prefs.getBool("radarapt", false);
    s.radarRangeNm = prefs.getUShort("radarrng", 0);
    s.zoomCode = prefs.isKey("zoomapt") ? prefs.getString("zoomapt") : String("");
    s.zoomViewNm = prefs.getUShort("zoomview", 0) / 10.0f;  // in tenths
    s.alertMask = prefs.getUChar("alerts", ALERT_ALL) & ALERT_ALL;
    s.watchlist = prefs.isKey("watch") ? prefs.getString("watch") : String("");
    s.autoRecord = prefs.getBool("autorec", false);
    s.autoFollow = prefs.getBool("autofollow", false);
    s.readAloud = prefs.getBool("readaloud", false);
    uint8_t accent = prefs.getUChar("accent", (uint8_t)DEFAULT_ELSEWHERE_ACCENT);
    s.elsewhereAccent = accent <= (uint8_t)Accent::AMERICAN ? (Accent)accent : DEFAULT_ELSEWHERE_ACCENT;
    s.brightnessPct = prefs.getUChar("bright", 0);  // 0: never set - left as it comes up
    s.dimMinutes = prefs.getUChar("dimmin", 0);
    // From before there were zones, a clock turned off stays off.
    s.clockZone = prefs.getUChar("clockzone", prefs.getBool("clock", true) ? CLOCK_ZONE_AUTO : 0);
    if (s.clockZone != CLOCK_ZONE_AUTO && s.clockZone >= CLOCK_ZONE_COUNT) {
        s.clockZone = CLOCK_ZONE_AUTO;
    }
    s.autoZoom = prefs.getBool("autozoom", false);
    s.replayTracks = prefs.getBool("rtracks", true);
    uint8_t sort = prefs.getUChar("tsort", 4);  // DIST, nearest first
    s.tableSortCol = sort & 7;
    s.tableSortDesc = (sort & 8) != 0;
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
    int maxNm = radiusCeilingNm(loadTrafficFilter().active());
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

void saveFilters(int radiusNm, bool showRefresh, int pollIntervalS, AdsbSource source) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
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

void saveCoast(bool coast) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("coast", coast);
    prefs.end();
}

void saveRings(bool rings) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("rings", rings);
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

void saveRadarZoom(const String &code, float viewNm) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putString("zoomapt", code);
    prefs.putUShort("zoomview", (uint16_t)lroundf(viewNm * 10));
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

void saveDisplay(uint8_t brightnessPct, uint8_t dimMinutes, uint8_t clockZone, bool replayTracks) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putUChar("bright", brightnessPct);
    prefs.putUChar("dimmin", dimMinutes);
    prefs.putUChar("clockzone", clockZone);
    prefs.putBool("rtracks", replayTracks);
    prefs.end();
}

void saveAutoZoom(bool autoZoom) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("autozoom", autoZoom);
    prefs.end();
}

void saveTableSort(int col, bool descending) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putUChar("tsort", (uint8_t)((col & 7) | (descending ? 8 : 0)));
    prefs.end();
}

void saveVoice(bool readAloud, Accent elsewhere) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putBool("readaloud", readAloud);
    prefs.putUChar("accent", (uint8_t)elsewhere);
    prefs.end();
}

// Heights in hundreds of feet, to fit a short.
TrafficFilterSet loadTrafficFilter() {
    Preferences prefs;
    prefs.begin(NAMESPACE, true);
    TrafficFilterSet f;
    f.minFt = std::min<int>(prefs.getUShort("fminft", 0) * 100, FILTER_TOP_FT);
    f.maxFt = std::min<int>(prefs.getUShort("fmaxft", FILTER_TOP_FT / 100) * 100, FILTER_TOP_FT);
    f.minKt = std::min<int>(prefs.getUShort("fminkt", 0), FILTER_TOP_KT);
    f.maxKt = std::min<int>(prefs.getUShort("fmaxkt", FILTER_TOP_KT), FILTER_TOP_KT);
    f.minFt = std::min(f.minFt, f.maxFt);
    f.minKt = std::min(f.minKt, f.maxKt);
    f.civil = prefs.getBool("fciv", true);
    f.cargo = prefs.getBool("fcargo", true);
    f.military = prefs.getBool("fmil", true);
    if (!f.civil && !f.cargo && !f.military) {
        f.civil = f.cargo = f.military = true;
    }
    f.airlines = prefs.isKey("fairl") ? prefs.getString("fairl") : String("");
    f.types = prefs.isKey("ftype") ? prefs.getString("ftype") : String("");
    f.hideTypes = prefs.isKey("fhide") ? prefs.getString("fhide") : String("");
    prefs.end();
    return f;
}

void saveTrafficFilter(const TrafficFilterSet &f) {
    Preferences prefs;
    prefs.begin(NAMESPACE, false);
    prefs.putUShort("fminft", f.minFt / 100);
    prefs.putUShort("fmaxft", f.maxFt / 100);
    prefs.putUShort("fminkt", f.minKt);
    prefs.putUShort("fmaxkt", f.maxKt);
    prefs.putBool("fciv", f.civil);
    prefs.putBool("fcargo", f.cargo);
    prefs.putBool("fmil", f.military);
    prefs.putString("fairl", f.airlines);
    prefs.putString("ftype", f.types);
    prefs.putString("fhide", f.hideTypes);
    prefs.end();
}
