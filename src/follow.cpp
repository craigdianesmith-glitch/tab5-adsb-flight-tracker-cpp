#include "follow.h"

#include <algorithm>
#include <math.h>

#include "config.h"
#include "runways.h"

namespace {

// Off its nose by more than this, an airport is one it is passing, not one
// it is heading for.
constexpr float AHEAD_DEG = 60.0f;
// Nearer than this, it is that airport's whichever way it is pointing: on a
// base leg, or turning onto final.
constexpr float ALWAYS_NM = 4.0f;

float bearingDeg(float lat1, float lon1, float lat2, float lon2) {
    float dNorth = (lat2 - lat1) * 60.0f;
    float dEast = (lon2 - lon1) * 60.0f * cosf(lat1 * (float)M_PI / 180.0f);
    return atan2f(dEast, dNorth) * 180.0f / (float)M_PI;
}

}  // namespace

bool followDestination(const Aircraft &ac, Airport &out, float &distNm) {
    if (!ac.hasPos || ac.status == "CLIMB") {
        return false;
    }
    bool ground = (ac.status == "GROUND" || ac.status == "TAXI");
    if (!ground && (ac.altStr == "?" || ac.altStr.toInt() > FOLLOW_APPROACH_MAX_FT)) {
        return false;
    }
    // Level, it is only on its way down once it is low: high and level, an
    // airport ahead is one it is passing over.
    if (!ground && ac.status != "DESCEND" && ac.altStr.toInt() > FOLLOW_APPROACH_LEVEL_FT) {
        return false;
    }
    for (const Airport &a : airportsWithin(ac.lat, ac.lon, FOLLOW_RANGE_NM)) {  // nearest first
        float d = haversineNm(ac.lat, ac.lon, a.lat, a.lon);
        if (!ground && d > ALWAYS_NM && ac.hasTrack) {
            float off = fabsf(fmodf(bearingDeg(ac.lat, ac.lon, a.lat, a.lon) - ac.track + 540.0f, 360.0f) - 180.0f);
            if (off > AHEAD_DEG) {
                continue;
            }
        }
        out = a;
        distNm = d;
        return true;
    }
    return false;
}

int followRangeFor(const Aircraft &ac) {
    Airport a;
    float d;
    if (!followDestination(ac, a, d)) {
        return FOLLOW_RANGE_NM;
    }
    // With a quarter again around it, so the airport sits inside the outer
    // ring rather than on it.
    for (int r : FOLLOW_RANGE_STEPS_NM) {
        if (d * 1.25f <= r) {
            return r;
        }
    }
    return FOLLOW_RANGE_NM;
}

namespace {

// The aiming point is about 300m in from the threshold, where a landing
// aircraft is taken to touch down if it hasn't already.
constexpr float TOUCHDOWN_NM = 0.17f;
constexpr float GLIDE_FT_PER_NM = 318.0f;  // three degrees
// Braking on the rollout, to the speed it would turn off at - after which
// where it goes is anyone's guess, so it is left there.
constexpr float ROLLOUT_DECEL_KT_PER_S = 2.2f;  // as an airliner landing at Glasgow was seen to
constexpr float TURN_OFF_KT = 20.0f;
constexpr float STOP_SHORT_NM = 0.1f;  // of the far end
// Lined up: pointing along the runway, and near its centreline - out on
// final, or on it.
constexpr float LINED_UP_DEG = 20.0f;
constexpr float LINED_UP_OFFSET_NM = 0.3f;
// A departure gathering speed can still be turning onto the runway - one at
// Glasgow was lost at 41kt pointing well off it - so for one on the ground
// the runway itself is enough: on its pavement, turned at least half way
// round to it, and not past its far end.
constexpr float ON_RUNWAY_DEG = 60.0f;
constexpr float ON_RUNWAY_OFFSET_NM = 0.05f;
constexpr float FINAL_MAX_NM = 10.0f;
// Below this on the ground it is taxiing, and could turn any way. A departure
// is taken to be on its roll a little sooner: lined up and moving, it has
// only one way to go.
constexpr float ROLLING_KT = 40.0f;
constexpr float TAKEOFF_ROLLING_KT = 30.0f;
// The take-off: accelerating down the runway to rotation, then climbing out
// on its heading, gathering speed to the climb's.
constexpr float TAKEOFF_ACCEL_KT_PER_S = 3.5f;
constexpr float ROTATE_KT = 145.0f;
constexpr float CLIMB_ACCEL_KT_PER_S = 1.0f;
constexpr float CLIMB_OUT_KT = 180.0f;
constexpr int CLIMB_OUT_FPM = 2000;
// A report in the air this long before one on the ground makes it a landing
// rather than a departure.
constexpr uint32_t LANDED_WITHIN_MS = 180000;
// Or a report on the ground this soon before it, this much faster.
constexpr uint32_t SLOWING_WITHIN_MS = 30000;
constexpr int SLOWING_KT = 5;

struct Lineup {
    float thrX, thrY;  // the threshold it is landing over, in nm from its last position
    float ux, uy;      // along the runway, the way it is landing
    float len;
    float along;       // where it was, from the threshold
    int32_t fieldFt;
};

bool scanLineup(const FollowSample &last, bool onGround, Lineup &out);

// The answers for the last few reports asked about. An estimate is asked for
// again and again from the same report - each frame between polls, for each
// low contact - and the scan is of every airport near it.
struct LineupCache {
    float lat = NAN, lon = NAN, track = NAN;
    bool ground = false;
    bool found = false;
    Lineup l;
};
LineupCache g_lineups[16];
int g_lineupNext = 0;

// The runway end `last` is lined up on, if any, of the airports near it.
bool findLineup(const FollowSample &last, Lineup &out) {
    if (!last.hasTrack) {
        return false;
    }
    for (const LineupCache &c : g_lineups) {
        if (c.lat == last.lat && c.lon == last.lon && c.track == last.track && c.ground == last.ground) {
            out = c.l;
            return c.found;
        }
    }
    LineupCache &c = g_lineups[g_lineupNext];
    g_lineupNext = (g_lineupNext + 1) % 16;
    c.lat = last.lat;
    c.lon = last.lon;
    c.track = last.track;
    c.ground = last.ground;
    c.found = scanLineup(last, last.ground, c.l);
    out = c.l;
    return c.found;
}

bool scanLineup(const FollowSample &last, bool onGround, Lineup &out) {
    if (!last.hasTrack) {
        return false;
    }
    float nmPerDegLon = 60.0f * cosf(last.lat * (float)M_PI / 180.0f);
    float bestOffset = LINED_UP_OFFSET_NM;
    bool found = false;
    for (const Airport &a : airportsWithin(last.lat, last.lon, FINAL_MAX_NM)) {
        int count = 0;
        const Runway *runways = runwaysAt(a.code, count);
        for (int r = 0; r < count; r++) {
            const Runway &rw = runways[r];
            if (rw.closed) {
                continue;
            }
            for (int k = 0; k < 2; k++) {
                const RunwayEnd &from = rw.end[k], &to = rw.end[1 - k];
                float fx = (from.lon - last.lon) * nmPerDegLon, fy = (from.lat - last.lat) * 60.0f;
                float tx = (to.lon - last.lon) * nmPerDegLon, ty = (to.lat - last.lat) * 60.0f;
                float len = hypotf(tx - fx, ty - fy);
                if (len <= 0) {
                    continue;
                }
                float ux = (tx - fx) / len, uy = (ty - fy) / len;
                float heading = atan2f(ux, uy) * 180.0f / (float)M_PI;
                float off = fabsf(fmodf(last.track - heading + 540.0f, 360.0f) - 180.0f);
                // Its position relative to the threshold: along, and across.
                float rx = -fx, ry = -fy;
                float along = rx * ux + ry * uy;
                float offset = fabsf(rx * uy - ry * ux);
                bool aligned = off <= LINED_UP_DEG && along >= -FINAL_MAX_NM;
                bool onIt = onGround && off <= ON_RUNWAY_DEG && offset <= ON_RUNWAY_OFFSET_NM && along >= -ON_RUNWAY_OFFSET_NM;
                if (!(aligned || onIt) || along > len || offset > bestOffset) {
                    continue;
                }
                bestOffset = offset;
                out = {fx, fy, ux, uy, len, along, rw.elevationFt};
                found = true;
            }
        }
    }
    return found;
}

}  // namespace

bool followDeparting(const std::vector<FollowSample> &history, const FollowSample &last) {
    if (!last.ground) {
        return false;
    }
    for (const FollowSample &s : history) {
        if (s.ms >= last.ms || last.ms - s.ms > LANDED_WITHIN_MS) {
            continue;
        }
        if (!s.ground) {
            return false;
        }
        // Slowing down the runway since the report before - picked on its
        // rollout, before it was ever seen in the air.
        if (last.ms - s.ms <= SLOWING_WITHIN_MS && s.hasGs && last.hasGs && s.gsKt > last.gsKt + SLOWING_KT &&
            s.gsKt >= TAKEOFF_ROLLING_KT) {
            return false;
        }
    }
    return true;
}

namespace {

// Which way it was going, from where it was before, for a report that
// doesn't say: the bearing from the latest earlier position far enough off
// to give one, from the last minute.
bool trackFromHistory(const std::vector<FollowSample> &history, const FollowSample &last, float &track) {
    for (auto it = history.rbegin(); it != history.rend(); ++it) {
        const FollowSample &s = *it;
        if (s.ms >= last.ms || !s.hasPos || s.posStale) {
            continue;
        }
        if (last.ms - s.ms > 60000) {
            break;
        }
        float dNorth = (last.lat - s.lat) * 60.0f;
        float dEast = (last.lon - s.lon) * 60.0f * cosf(last.lat * (float)M_PI / 180.0f);
        if (hypotf(dNorth, dEast) >= 0.02f) {
            track = atan2f(dEast, dNorth) * 180.0f / (float)M_PI;
            if (track < 0) {
                track += 360.0f;
            }
            return true;
        }
    }
    return false;
}

// For a departure gathering speed that no runway end lines up with by its
// track - none given, or still turning on - the runway whose pavement it is
// on, taken in the direction it was moving along it, or failing that towards
// the end with the more runway ahead.
bool runwayUnder(const FollowSample &last, const std::vector<FollowSample> &history, Lineup &out) {
    float nmPerDegLon = 60.0f * cosf(last.lat * (float)M_PI / 180.0f);
    for (const Airport &a : airportsWithin(last.lat, last.lon, 5.0f)) {
        int count = 0;
        const Runway *runways = runwaysAt(a.code, count);
        for (int r = 0; r < count; r++) {
            const Runway &rw = runways[r];
            if (rw.closed) {
                continue;
            }
            float ax = (rw.end[0].lon - last.lon) * nmPerDegLon, ay = (rw.end[0].lat - last.lat) * 60.0f;
            float bx = (rw.end[1].lon - last.lon) * nmPerDegLon, by = (rw.end[1].lat - last.lat) * 60.0f;
            float len = hypotf(bx - ax, by - ay);
            if (len <= 0) {
                continue;
            }
            float ux = (bx - ax) / len, uy = (by - ay) / len;  // end 0 to end 1
            float along = -ax * ux - ay * uy;                  // from end 0
            float offset = fabsf(-ax * uy + ay * ux);
            if (offset > ON_RUNWAY_OFFSET_NM || along < 0 || along > len) {
                continue;
            }
            // Towards end 1, unless it was moving towards end 0 - or, not
            // known to be moving either way, end 0 has the more ahead.
            bool towards1 = along < len / 2;
            float t;
            if (trackFromHistory(history, last, t)) {
                float a = t * (float)M_PI / 180.0f;
                towards1 = sinf(a) * ux + cosf(a) * uy >= 0;
            }
            if (towards1) {
                out = {ax, ay, ux, uy, len, along, rw.elevationFt};
            } else {
                out = {bx, by, -ux, -uy, len, len - along, rw.elevationFt};
            }
            return true;
        }
    }
    return false;
}

}  // namespace

FollowEstimate estimateFollowed(const FollowSample &given, float ageS, const std::vector<FollowSample> &history) {
    FollowSample last = given;
    if (!last.hasTrack) {
        last.hasTrack = trackFromHistory(history, last, last.track);
    }
    bool departing = followDeparting(history, last);
    FollowEstimate e = {last.lat, last.lon, last.track, last.hasAlt ? last.altFt : 0, last.ground,
                        last.hasGs ? last.gsKt : 0, last.hasVs ? last.vsFpm : 0, "held"};
    float gs = last.hasGs ? (float)last.gsKt : 0.0f;
    float nmPerDegLon = 60.0f * cosf(last.lat * (float)M_PI / 180.0f);
    auto place = [&](float x, float y) {
        e.lat = last.lat + y / 60.0f;
        e.lon = last.lon + x / nmPerDegLon;
    };

    // Only low, or rolling, is a runway worth looking for - and the look is a
    // scan of the airports around it, so it is not made for the rest.
    Lineup l;
    bool rolling = !last.ground || gs >= (departing ? TAKEOFF_ROLLING_KT : ROLLING_KT);
    bool lowEnough = last.ground || !last.hasAlt || last.altFt < ESTIMATE_MAX_FT;
    bool lined = rolling && lowEnough && findLineup(last, l);
    if (!lined && rolling && last.ground && departing) {
        lined = runwayUnder(last, history, l);
    }
    if (!lined) {
        if (last.ground) {
            e.how = rolling ? "held: rolling, but on no runway" : "held: taxiing";
            return e;  // taxiing, or holding: left where it was
        }
        if (!last.hasTrack) {
            e.how = "held: no track";
            return e;
        }
        e.how = "straight on";
        float d = gs * ageS / 3600.0f;
        float a = last.track * (float)M_PI / 180.0f;
        place(sinf(a) * d, cosf(a) * d);
        if (last.hasAlt && last.hasVs) {
            e.altFt = std::max((int32_t)0, last.altFt + (int32_t)(last.vsFpm * ageS / 60.0f));
        }
        return e;
    }

    e.track = atan2f(l.ux, l.uy) * 180.0f / (float)M_PI;
    if (e.track < 0) {
        e.track += 360.0f;
    }
    float along = l.along;
    if (last.ground && departing) {
        // The take-off roll, to rotation - or the far end, whichever is first.
        float toRotateS = gs < ROTATE_KT ? (ROTATE_KT - gs) / TAKEOFF_ACCEL_KT_PER_S : 0;
        float liftoff = std::min(along + (gs * toRotateS + TAKEOFF_ACCEL_KT_PER_S * toRotateS * toRotateS / 2.0f) / 3600.0f,
                                 l.len - STOP_SHORT_NM);
        if (ageS < toRotateS) {
            along += (gs * ageS + TAKEOFF_ACCEL_KT_PER_S * ageS * ageS / 2.0f) / 3600.0f;
            along = std::min(along, liftoff);
            place(l.thrX + l.ux * along, l.thrY + l.uy * along);
            e.altFt = l.fieldFt;
            e.ground = true;
            e.gsKt = (int)lroundf(gs + TAKEOFF_ACCEL_KT_PER_S * ageS);
            e.vsFpm = 0;
            e.how = "take-off roll";
            return e;
        }
        // Off, and climbing out along the runway's line.
        float airS = ageS - toRotateS;
        float v0 = std::max(gs, ROTATE_KT);
        float accelS = std::min(airS, std::max(0.0f, (CLIMB_OUT_KT - v0) / CLIMB_ACCEL_KT_PER_S));
        float d = (v0 * accelS + CLIMB_ACCEL_KT_PER_S * accelS * accelS / 2.0f +
                   (v0 + CLIMB_ACCEL_KT_PER_S * accelS) * (airS - accelS)) / 3600.0f;
        along = liftoff + d;
        place(l.thrX + l.ux * along, l.thrY + l.uy * along);
        e.altFt = l.fieldFt + (int32_t)(CLIMB_OUT_FPM * airS / 60.0f);
        e.ground = false;
        e.gsKt = (int)lroundf(v0 + CLIMB_ACCEL_KT_PER_S * accelS);
        e.vsFpm = CLIMB_OUT_FPM;
        e.how = "climbing out";
        return e;
    }
    float rollS = ageS;
    if (!last.ground) {
        // In to the touchdown point - or, past it already, down now.
        float touchdown = std::max(along, TOUCHDOWN_NM);
        float toTouchdownS = gs > 0 ? (touchdown - along) / gs * 3600.0f : 0;
        if (ageS < toTouchdownS) {
            along += gs * ageS / 3600.0f;
            place(l.thrX + l.ux * along, l.thrY + l.uy * along);
            e.altFt = l.fieldFt + (int32_t)((touchdown - along) * GLIDE_FT_PER_NM);
            e.ground = false;
            e.vsFpm = -(int)(gs * GLIDE_FT_PER_NM / 60.0f);
            e.how = "on final";
            return e;
        }
        along = touchdown;
        rollS = ageS - toTouchdownS;
    }
    // The rollout: braking steadily to the turn-off speed, then left there.
    float brakeS = gs > TURN_OFF_KT ? (gs - TURN_OFF_KT) / ROLLOUT_DECEL_KT_PER_S : 0;
    float t = std::min(rollS, brakeS);
    along += (gs * t - ROLLOUT_DECEL_KT_PER_S * t * t / 2.0f) / 3600.0f;
    along = std::min(along, l.len - STOP_SHORT_NM);
    place(l.thrX + l.ux * along, l.thrY + l.uy * along);
    e.altFt = l.fieldFt;
    e.ground = true;
    e.gsKt = (int)lroundf(gs - ROLLOUT_DECEL_KT_PER_S * t);
    e.vsFpm = 0;
    e.how = "rollout";
    return e;
}
