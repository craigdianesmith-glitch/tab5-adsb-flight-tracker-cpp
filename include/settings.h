#pragma once

#include <Arduino.h>

// Civil and military are an either/or choice, not two independent filters -
// which is also what sets the range ceiling, since the two have different ones.
enum class TrafficFilter : uint8_t { CIVIL, MILITARY };

struct AppSettings {
    double lat;
    double lon;
    String label;

    TrafficFilter traffic;
    int radiusNm;
    bool showRefresh;  // shade cells that changed on the last poll

    // Empty ssid means "use the credentials compiled in from secrets.h", so a
    // device that's never had WiFi set on-screen behaves exactly as before.
    String wifiSsid;
    String wifiPass;
};

AppSettings loadSettings();
void saveLocation(double lat, double lon, const String &label);
void saveFilters(TrafficFilter traffic, int radiusNm, bool showRefresh);
void saveWifi(const String &ssid, const String &pass);
