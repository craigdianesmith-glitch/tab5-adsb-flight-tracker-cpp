#include "geocode.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ctype.h>

namespace {

String urlEncode(const String &s) {
    String out;
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') {
            out += c;
        } else if (c == ' ') {
            out += "%20";
        } else {
            char buf[4];
            snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c);
            out += buf;
        }
    }
    return out;
}

}  // namespace

bool geocodeSearch(const String &query, std::vector<GeoResult> &out) {
    String url = "https://geocoding-api.open-meteo.com/v1/search?name=" + urlEncode(query) + "&count=8&language=en&format=json";

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, url)) {
        return false;
    }
    http.setUserAgent(
        "OverheadFlightTracker/1.0 (+https://github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-micropython)");
    int code = http.GET();
    String payload = http.getString();
    http.end();
    if (code != 200) {
        return false;
    }

    JsonDocument doc;
    if (deserializeJson(doc, payload)) {
        return false;
    }

    std::vector<GeoResult> result;
    JsonVariantConst resultsV = doc["results"];
    if (!resultsV.isNull()) {
        for (JsonObjectConst r : resultsV.as<JsonArrayConst>()) {
            GeoResult g;
            String name = (const char *)(r["name"] | "");
            String label = name;
            if (!r["admin1"].isNull()) {
                label += ", " + String((const char *)r["admin1"]);
            }
            if (!r["country"].isNull()) {
                label += ", " + String((const char *)r["country"]);
            }
            g.label = label;
            g.lat = r["latitude"] | 0.0;
            g.lon = r["longitude"] | 0.0;
            result.push_back(g);
        }
    }
    out = std::move(result);
    return true;
}
