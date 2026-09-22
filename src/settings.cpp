#include "settings.h"

#include <Preferences.h>

#include "config.h"

LocationSettings loadSettings() {
    Preferences prefs;
    prefs.begin("overhead", true);
    LocationSettings s;
    s.lat = prefs.getDouble("lat", DEFAULT_LAT);
    s.lon = prefs.getDouble("lon", DEFAULT_LON);
    s.label = prefs.getString("label", DEFAULT_LABEL);
    prefs.end();
    return s;
}

void saveSettings(double lat, double lon, const String &label) {
    Preferences prefs;
    prefs.begin("overhead", false);
    prefs.putDouble("lat", lat);
    prefs.putDouble("lon", lon);
    prefs.putString("label", label);
    prefs.end();
}
