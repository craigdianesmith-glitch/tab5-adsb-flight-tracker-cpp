#pragma once

#include <Arduino.h>

#include "config.h"

// Civil and military are an either/or choice, not two independent filters -
// which is also what sets the range ceiling, since the two have different ones.
enum class TrafficFilter : uint8_t { CIVIL, MILITARY };

struct AppSettings {
    double lat;
    double lon;
    String label;

    TrafficFilter traffic;
    int radiusNm;
    bool showRefresh;   // shade cells that changed on the last poll
    bool navaids;       // mark the VORs, DMEs and TACANs in range on the radar
    int pollIntervalS;  // how often the sky is refetched
    bool muted;         // speaker silenced from the header
    AdsbSource source;  // which ADS-B provider to poll, or AUTO to fail over
    RadarCentre radarCentre;  // toggled from the radar screen itself
    bool radarAirports;       // the other airports around the centred one - a long press there
    int radarRangeNm;         // how far the radar shows, from ZOOM IN and OUT; 0 for the whole radius

    uint8_t alertMask;  // which AlertReasons raise an alert
    String watchlist;   // as typed: entries separated by spaces or commas
    bool autoRecord;    // record the radar while an alerted aircraft is about
    bool autoFollow;    // follow an alerted aircraft on the radar as its alert fires

    // Empty ssid means "use the credentials compiled in from secrets.h", so a
    // device that's never had WiFi set on-screen behaves exactly as before.
    String wifiSsid;
    String wifiPass;
};

AppSettings loadSettings();
void saveLocation(double lat, double lon, const String &label);
void saveFilters(TrafficFilter traffic, int radiusNm, bool showRefresh, int pollIntervalS,
                 AdsbSource source);
void saveMuted(bool muted);
void saveNavaids(bool navaids);
void saveRadarCentre(RadarCentre centre);
void saveRadarAirports(bool airports);
void saveRadarRange(int rangeNm);
void saveWifi(const String &ssid, const String &pass);
void saveAlerts(uint8_t alertMask, const String &watchlist, bool autoRecord, bool autoFollow);
