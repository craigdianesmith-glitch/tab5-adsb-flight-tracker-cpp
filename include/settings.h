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
    int pollIntervalS;  // how often the sky is refetched
    bool muted;         // speaker silenced from the header
    AdsbSource source;  // which ADS-B provider to poll, or AUTO to fail over
    RadarCentre radarCentre;  // toggled from the radar screen itself

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
void saveRadarCentre(RadarCentre centre);
void saveWifi(const String &ssid, const String &pass);
