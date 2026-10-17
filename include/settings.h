#pragma once

#include <Arduino.h>

#include "config.h"
#include "clock_zone.h"
#include "filter.h"

// Civil and military are an either/or choice, not two independent filters -
// which is also what sets the range ceiling, since the two have different ones.

constexpr uint8_t CLOCK_ZONE_AUTO = 0xFF;

struct AppSettings {
    double lat;
    double lon;
    String label;

    int radiusNm;
    bool showRefresh;   // shade cells that changed on the last poll
    bool navaids;       // mark the VORs, DMEs and TACANs in range on the radar
    bool coast;         // the map under the radar: coasts, lakes and borders
    bool rings;         // the radar's range rings, inside its outer one
    int pollIntervalS;  // how often the sky is refetched
    bool muted;         // speaker silenced from the header
    AdsbSource source;  // which ADS-B provider to poll, or AUTO to fail over
    RadarCentre radarCentre;  // toggled from the radar screen itself
    bool radarAirports;       // the other airports around the centred one - a long press there
    int radarRangeNm;         // how far the radar shows, from ZOOM IN and OUT; 0 for the whole radius
    // The airport zoom the radar was left on, to come back to after a reboot
    // as it was: its airport, empty for none, and the range ZOOM IN and OUT
    // had it at, 0 for its framing.
    String zoomCode;
    float zoomViewNm;

    uint8_t alertMask;  // which AlertReasons raise an alert
    String watchlist;   // as typed: entries separated by spaces or commas
    bool autoRecord;    // record the radar while an alerted aircraft is about
    bool autoFollow;    // follow an alerted aircraft on the radar as its alert fires
    bool followIn3d;    // ... in the 3D view, rather than on the radar
    bool readAloud;          // say an aircraft's details, and a navaid's or an airport's card, as they open
    Accent elsewhereAccent;  // the voice for everywhere without one of its own - see sound.h
    uint8_t brightnessPct;   // the screen's, 10 to 100 - see display_page.h
    uint8_t dimMinutes;      // dimmed after this long untouched; 0 for never
    // The clock on the radar and the zoom: off, or the zone it keeps - or
    // CLOCK_ZONE_AUTO, never picked, for the zone home is in.
    uint8_t clockZone;
    bool autoZoom;           // following, the radar's range sized to the sky round the aircraft
    bool replayTracks;       // each aircraft's last ten minutes behind it in a replay
    uint8_t tableSortCol;    // the table's order - see displayHitHeader()
    bool tableSortDesc;

    // Empty ssid means "use the credentials compiled in from secrets.h", so a
    // device that's never had WiFi set on-screen behaves exactly as before.
    String wifiSsid;
    String wifiPass;
};

AppSettings loadSettings();
void saveLocation(double lat, double lon, const String &label);
void saveFilters(int radiusNm, bool showRefresh, int pollIntervalS, AdsbSource source);
void saveMuted(bool muted);
void saveNavaids(bool navaids);
void saveCoast(bool coast);
void saveRings(bool rings);
void saveRadarCentre(RadarCentre centre);
void saveRadarAirports(bool airports);
void saveRadarRange(int rangeNm);
void saveRadarZoom(const String &code, float viewNm);
void saveWifi(const String &ssid, const String &pass);
void saveAlerts(uint8_t alertMask, const String &watchlist, bool autoRecord, bool autoFollow, bool followIn3d);
void saveVoice(bool readAloud, Accent elsewhere);
void saveTableSort(int col, bool descending);
void saveDisplay(uint8_t brightnessPct, uint8_t dimMinutes, uint8_t clockZone, bool replayTracks);
void saveAutoZoom(bool autoZoom);
// The filter screen's, apart from the rest as it is put in force apart.
TrafficFilterSet loadTrafficFilter();
void saveTrafficFilter(const TrafficFilterSet &f);
