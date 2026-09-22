#include "adsb_client.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <algorithm>
#include <math.h>

#include "config.h"

static float haversineNm(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R_KM = 6371.0;
    double p1 = radians(lat1), p2 = radians(lat2);
    double dphi = radians(lat2 - lat1);
    double dlambda = radians(lon2 - lon1);
    double a = sin(dphi / 2) * sin(dphi / 2) + cos(p1) * cos(p2) * sin(dlambda / 2) * sin(dlambda / 2);
    double km = 2 * R_KM * asin(sqrt(a));
    return (float)(km * 0.539957);
}

bool fetchAircraft(double lat, double lon, int radiusNm, std::vector<Aircraft> &out) {
    char url[160];
    snprintf(url, sizeof(url), ADSB_API_URL_FMT, lat, lon, radiusNm);

    Serial.printf("[adsb] GET %s\n", url);
    WiFiClientSecure client;
    client.setInsecure();  // no CA bundle managed for this hobby project
    HTTPClient http;
    if (!http.begin(client, url)) {
        Serial.println("[adsb] http.begin() failed");
        return false;
    }
    // HTTPClient always sends its own internal _userAgent field regardless of
    // addHeader("User-Agent", ...) (see HTTPClient.cpp ~line 1226), so that
    // call was silently overridden by the default "ESP32HTTPClient" - which
    // is exactly the kind of generic string adsb.lol rejects. setUserAgent()
    // is the actual override point.
    http.setUserAgent(
        "OverheadFlightTracker/1.0 (+https://github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-micropython)");
    int code = http.GET();
    String payload = http.getString();
    http.end();
    Serial.printf("[adsb] status=%d body=%s\n", code, payload.c_str());
    if (code != 200) {
        return false;
    }
    Serial.printf("[adsb] payload length=%d\n", payload.length());

    JsonDocument doc;
    DeserializationError jerr = deserializeJson(doc, payload);
    if (jerr) {
        Serial.printf("[adsb] json parse error: %s\n", jerr.c_str());
        return false;
    }

    std::vector<Aircraft> result;
    for (JsonObjectConst ac : doc["ac"].as<JsonArrayConst>()) {
        const char *hex = ac["hex"] | "";
        if (!hex[0]) {
            continue;
        }

        JsonVariantConst altV = ac["alt_baro"];
        bool onGround = altV.is<const char *>();
        float altNum = 0;
        if (!onGround && !altV.isNull()) {
            altNum = altV.as<float>();
            if (altNum < 0) {
                continue;  // bogus negative-altitude reports occasionally show up
            }
        }

        Aircraft a;
        a.hex = hex;
        String cs = String((const char *)(ac["flight"] | ""));
        cs.trim();
        a.callsign = cs.length() ? cs : a.hex;
        a.type = (const char *)(ac["t"] | "----");

        if (onGround) {
            a.altStr = "GND";
        } else if (altV.isNull()) {
            a.altStr = "?";
        } else {
            a.altStr = String((long)altNum);
        }

        a.speedStr = ac["gs"].isNull() ? "?" : String((long)lround(ac["gs"].as<float>()));

        if (!ac["lat"].isNull() && !ac["lon"].isNull()) {
            a.hasDist = true;
            a.distNm = haversineNm(lat, lon, ac["lat"].as<double>(), ac["lon"].as<double>());
        } else {
            a.hasDist = false;
            a.distNm = 0;
        }

        if (onGround) {
            float gs = ac["gs"] | -1.0f;
            a.status = (gs > 2) ? "TAXI" : "GROUND";
        } else {
            float rate;
            if (!ac["baro_rate"].isNull()) {
                rate = ac["baro_rate"].as<float>();
            } else if (!ac["geom_rate"].isNull()) {
                rate = ac["geom_rate"].as<float>();
            } else {
                rate = 0.0f;
            }
            if (rate >= CLIMB_THRESHOLD_FPM) {
                a.status = "CLIMB";
            } else if (rate <= DESCEND_THRESHOLD_FPM) {
                a.status = "DESCEND";
            } else {
                a.status = "LEVEL";
            }
        }

        result.push_back(a);
    }

    std::sort(result.begin(), result.end(), [](const Aircraft &a, const Aircraft &b) {
        float da = a.hasDist ? a.distNm : 1e9f;
        float db = b.hasDist ? b.distNm : 1e9f;
        return da < db;
    });

    out = std::move(result);
    return true;
}
