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
    if (code != 200) {
        Serial.printf("[adsb] status=%d body=%s\n", code, payload.c_str());
        return false;
    }

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
            a.hasPos = true;
            a.lat = ac["lat"].as<float>();
            a.lon = ac["lon"].as<float>();
        } else {
            a.hasDist = false;
            a.distNm = 0;
            a.hasPos = false;
        }

        a.reg = ac["r"].isNull() ? "" : String((const char *)ac["r"]);
        a.squawk = ac["squawk"].isNull() ? "" : String((const char *)ac["squawk"]);
        a.category = ac["category"].isNull() ? "" : String((const char *)ac["category"]);

        if (!ac["track"].isNull()) {
            a.hasTrack = true;
            a.track = ac["track"].as<float>();
        } else {
            a.hasTrack = false;
        }

        if (!ac["alt_geom"].isNull()) {
            a.hasAltGeom = true;
            a.altGeom = ac["alt_geom"].as<int>();
        } else {
            a.hasAltGeom = false;
        }

        a.hasVertRate = !ac["baro_rate"].isNull() || !ac["geom_rate"].isNull();
        a.vertRate = ac["baro_rate"].isNull() ? ac["geom_rate"] | 0.0f : ac["baro_rate"].as<float>();

        if (onGround) {
            float gs = ac["gs"] | -1.0f;
            a.status = (gs > 2) ? "TAXI" : "GROUND";
        } else {
            float rate = a.vertRate;
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
