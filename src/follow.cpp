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
// Braking on the rollout, to the speed it would turn off at, then more gently
// to a stop: where it turns off is anyone's guess, so it is left there.
constexpr float ROLLOUT_DECEL_KT_PER_S = 2.2f;  // as an airliner landing at Glasgow was seen to
constexpr float TURN_OFF_KT = 20.0f;
constexpr float TAXI_DECEL_KT_PER_S = 1.0f;  // from there to a stop
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
// Climbing faster than this, in the air, it is not landing - lifted off, or
// going around - however it is lined up.
constexpr int CLIMBING_FPM = 300;
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
// A departure stopped on the runway itself - lined up, waiting - and lost
// there is taken to have begun its roll once this long has gone by: it was
// lined up to go, and the feed losing it is as likely as not its going.
// Tighter about the centreline than a roll down it, since a holding point is
// only some 90m from it, and pointing along it.
constexpr float LINED_UP_WAIT_S = 10.0f;
constexpr float LINED_UP_CENTRE_NM = 0.025f;
constexpr float LINED_UP_HEADING_DEG = 30.0f;
// A report in the air this long before one on the ground makes it a landing
// rather than a departure.
constexpr uint32_t LANDED_WITHIN_MS = 180000;
// Or a report on the ground this soon before it, this much faster.
constexpr uint32_t SLOWING_WITHIN_MS = 30000;
constexpr int SLOWING_KT = 5;
constexpr float FT_PER_NM = 6076.0f;

struct Lineup {
    float thrX, thrY;  // the threshold it is landing over, in nm from its last position
    float ux, uy;      // along the runway, the way it is landing
    float len;
    float along;       // where it was, from the threshold
    int32_t fieldFt;
    float halfWidthNm;
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
                out = {fx, fy, ux, uy, len, along, rw.elevationFt, rw.widthFt / 2.0f / FT_PER_NM};
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
//
// With `linedUp`, for one stopped there, it has to be on the centreline and
// pointing along the runway, which gives the direction - no movement to go by.
bool runwayUnder(const FollowSample &last, const std::vector<FollowSample> &history, Lineup &out,
                 bool linedUp = false) {
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
            if (offset > (linedUp ? LINED_UP_CENTRE_NM : ON_RUNWAY_OFFSET_NM) || along < 0 || along > len) {
                continue;
            }
            bool towards1 = along < len / 2;
            if (linedUp) {
                // Which way it points, and that it points along it at all.
                float heading = atan2f(ux, uy) * 180.0f / (float)M_PI;
                float off = fabsf(fmodf(last.track - heading + 540.0f, 360.0f) - 180.0f);
                if (off > LINED_UP_HEADING_DEG && off < 180.0f - LINED_UP_HEADING_DEG) {
                    continue;
                }
                towards1 = off <= LINED_UP_HEADING_DEG;
            } else {
                // Towards end 1, unless it was moving towards end 0 - or, not
                // known to be moving either way, end 0 has the more ahead.
                float t;
                if (trackFromHistory(history, last, t)) {
                    float a = t * (float)M_PI / 180.0f;
                    towards1 = sinf(a) * ux + cosf(a) * uy >= 0;
                }
            }
            if (towards1) {
                out = {ax, ay, ux, uy, len, along, rw.elevationFt, rw.widthFt / 2.0f / FT_PER_NM};
            } else {
                out = {bx, by, -ux, -uy, len, len - along, rw.elevationFt, rw.widthFt / 2.0f / FT_PER_NM};
            }
            return true;
        }
    }
    return false;
}

}  // namespace

namespace {

float nmPerDegLonAt(float lat) { return 60.0f * cosf(lat * (float)M_PI / 180.0f); }

float angleOff(float a, float b) { return fabsf(fmodf(a - b + 540.0f, 360.0f) - 180.0f); }

float headingOf(const RunwayLine &r) {
    float h = atan2f(r.ux, r.uy) * 180.0f / (float)M_PI;
    return h < 0 ? h + 360.0f : h;
}

// Where (lat, lon) is from runway `r`'s threshold: along it, and how far off
// its centreline.
void relate(const RunwayLine &r, float lat, float lon, float &along, float &offset) {
    float x = (lon - r.thrLon) * nmPerDegLonAt(r.thrLat), y = (lat - r.thrLat) * 60.0f;
    along = x * r.ux + y * r.uy;
    offset = fabsf(x * r.uy - y * r.ux);
}

// The runway a lineup found from `s` is relative to, fixed where it is.
RunwayLine lineOf(const FollowSample &s, const Lineup &l) {
    return {s.lat + l.thrY / 60.0f, s.lon + l.thrX / nmPerDegLonAt(s.lat), l.ux, l.uy, l.len, l.halfWidthNm,
            l.fieldFt};
}

// On runway `r`'s centreline, or its extension, `along` from its threshold,
// pointing down it.
FollowEstimate onLine(const RunwayLine &r, float along, int32_t altFt, bool ground, float gs, int vsFpm,
                      const char *how) {
    FollowEstimate e;
    e.lat = r.thrLat + r.uy * along / 60.0f;
    e.lon = r.thrLon + r.ux * along / nmPerDegLonAt(r.thrLat);
    e.track = headingOf(r);
    e.altFt = altFt;
    e.ground = ground;
    e.gsKt = (int)lroundf(std::max(gs, 0.0f));
    e.vsFpm = vsFpm;
    e.how = how;
    return e;
}

// The take-off from `along` down runway `r`, `t` seconds on. On the ground at
// `gs`: accelerating to rotation - by the far end at the latest - then
// climbing out on the runway's heading, gathering speed to the climb's. Or,
// `airborne` at `altFt` already, climbing on out at `vsFpm`.
FollowEstimate takeoffFrom(const RunwayLine &r, float along, float gs, bool airborne, int32_t altFt, int vsFpm,
                           float t) {
    float toRotateS = 0, liftoff = along;
    int32_t offFt = r.fieldFt;
    int fpm = CLIMB_OUT_FPM;
    if (!airborne) {
        toRotateS = gs < ROTATE_KT ? (ROTATE_KT - gs) / TAKEOFF_ACCEL_KT_PER_S : 0;
        float roll = (gs * toRotateS + TAKEOFF_ACCEL_KT_PER_S * toRotateS * toRotateS / 2.0f) / 3600.0f;
        liftoff = std::max(along, std::min(along + roll, r.len - STOP_SHORT_NM));
        if (t < toRotateS) {
            float a = along + (gs * t + TAKEOFF_ACCEL_KT_PER_S * t * t / 2.0f) / 3600.0f;
            return onLine(r, std::min(a, liftoff), r.fieldFt, true, gs + TAKEOFF_ACCEL_KT_PER_S * t, 0,
                          "take-off roll");
        }
    } else {
        offFt = altFt;
        if (vsFpm > 0) {
            fpm = vsFpm;
        }
    }
    float airS = t - toRotateS;
    float v0 = std::max(gs, ROTATE_KT);
    float accelS = std::min(airS, std::max(0.0f, (CLIMB_OUT_KT - v0) / CLIMB_ACCEL_KT_PER_S));
    float d = (v0 * accelS + CLIMB_ACCEL_KT_PER_S * accelS * accelS / 2.0f +
               (v0 + CLIMB_ACCEL_KT_PER_S * accelS) * (airS - accelS)) / 3600.0f;
    return onLine(r, liftoff + d, offFt + (int32_t)(fpm * airS / 60.0f), false, v0 + CLIMB_ACCEL_KT_PER_S * accelS,
                  fpm, "climbing out");
}

// The landing on runway `r` from `along`, `t` seconds on. In the air: in at
// `gs` to the touchdown point - or, past it already, down now - coming down
// from the height it gave to the field's by then, which lines its barometric
// height up with the ground however far out the day's pressure has it; or on
// a three-degree glide, where it gave none. Then the rollout: braking to the
// speed it would turn off at, and on more gently to a stop, no further than
// the far end. Where it turns off is left to the feed - the runway table has
// no taxiways - and it waits there until the feed says.
FollowEstimate landingFrom(const RunwayLine &r, float along, float gs, bool airborne, bool hasAlt, int32_t altFt,
                           float t) {
    float rollS = t;
    if (airborne) {
        float touchdown = std::max(along, TOUCHDOWN_NM);
        float toTouchdownS = gs > 0 ? (touchdown - along) / gs * 3600.0f : 0;
        if (t < toTouchdownS) {
            float a = along + gs * t / 3600.0f;
            float agl = hasAlt ? (float)(altFt - r.fieldFt) : -1.0f;
            if (agl > 0) {
                return onLine(r, a, r.fieldFt + (int32_t)(agl * (touchdown - a) / (touchdown - along)), false, gs,
                              -(int)(agl / toTouchdownS * 60.0f), "on final");
            }
            return onLine(r, a, r.fieldFt + (int32_t)((touchdown - a) * GLIDE_FT_PER_NM), false, gs,
                          -(int)(gs * GLIDE_FT_PER_NM / 60.0f), "on final");
        }
        along = touchdown;
        rollS = t - toTouchdownS;
    }
    float start = along;
    float brakeS = std::min(rollS, gs > TURN_OFF_KT ? (gs - TURN_OFF_KT) / ROLLOUT_DECEL_KT_PER_S : 0.0f);
    along += (gs * brakeS - ROLLOUT_DECEL_KT_PER_S * brakeS * brakeS / 2.0f) / 3600.0f;
    float v = gs - ROLLOUT_DECEL_KT_PER_S * brakeS;
    float slowS = std::min(rollS - brakeS, std::max(v, 0.0f) / TAXI_DECEL_KT_PER_S);
    along += (v * slowS - TAXI_DECEL_KT_PER_S * slowS * slowS / 2.0f) / 3600.0f;
    v -= TAXI_DECEL_KT_PER_S * slowS;
    if (along > r.len - STOP_SHORT_NM) {
        along = std::max(start, r.len - STOP_SHORT_NM);
        v = 0;
    }
    return onLine(r, along, r.fieldFt, true, v, 0, v >= 1.0f ? "rollout" : "rollout: stopped");
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
    bool climbing = !last.ground && last.hasVs && last.vsFpm >= CLIMBING_FPM;
    bool lined = rolling && lowEnough && !climbing && findLineup(last, l);
    if (!lined && rolling && last.ground && departing) {
        lined = runwayUnder(last, history, l);
    }
    // Stopped on the runway, lined up: its roll is taken to begin once it has
    // been lost for LINED_UP_WAIT_S, from a standstill.
    if (!lined && !rolling && last.ground && departing && last.hasTrack && runwayUnder(last, history, l, true)) {
        if (ageS <= LINED_UP_WAIT_S) {
            e.how = "held: lined up";
            return e;
        }
        lined = true;
        ageS -= LINED_UP_WAIT_S;
        gs = 0;
        e.gsKt = 0;
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

    RunwayLine r = lineOf(last, l);
    if (last.ground && departing) {
        return takeoffFrom(r, l.along, gs, false, 0, 0, ageS);
    }
    return landingFrom(r, l.along, gs, !last.ground, last.hasAlt, last.altFt, ageS);
}

namespace {

// The reports kept, to tell a take-off from a landing.
constexpr uint32_t SIM_HISTORY_MS = 180000;
constexpr size_t SIM_HISTORY_MAX = 12;  // there can be a hundred contacts, in the internal RAM WiFi and TLS need
// A landing begins this low over the field, this close in, lined up and not
// climbing (see CLIMBING_FPM); it is a go-around climbing faster than this.
constexpr float FINAL_START_AGL_FT = 2500.0f;
constexpr float FINAL_START_NM = 8.0f;
constexpr int GO_AROUND_FPM = 500;
// Its thrust picking up: on a runway, this much faster than at its report
// before, and at least this fast.
constexpr int THRUST_GAIN_KT = 8;
constexpr int THRUST_MIN_KT = 12;
// Slower than this on the ground, it is stopped - and its position, the same
// report after report, is not the feed repeating itself.
constexpr int STOPPED_KT = 5;
// A take-off is the feed's again once it is this high over the field, this
// far past the runway's end, or off its line by this much either way; and
// one slowing this much on the runway has been rejected.
constexpr float HANDOVER_AGL_FT = 1500.0f;
constexpr float HANDOVER_PAST_END_NM = 3.0f;
constexpr float OFF_LINE_NM = 0.5f;
constexpr float OFF_LINE_DEG = 30.0f;
constexpr int REJECTED_DROP_KT = 15;
// How far past its edges a report on the ground still counts as on the
// runway: ground positions are good to a few tens of metres.
constexpr float RUNWAY_EDGE_NM = 0.03f;
// A correction is eased in at no more than half its speed again, over
// between these.
constexpr float EASE_MIN_S = 3.0f, EASE_MAX_S = 20.0f;
constexpr float EASE_MIN_KT = 20.0f;
// A turn off the runway is taxied at about this, over between these.
constexpr float TURN_TAXI_KT = 20.0f;
constexpr float TURN_MIN_S = 3.0f, TURN_MAX_S = 10.0f;
// Taxiing, it is carried on along its track for up to this long - about a
// poll of the zoom's - and then left where it was, since it could turn any
// way; and each report is eased into, so it neither stops nor jumps between
// them.
constexpr float TAXI_CARRY_S = 12.0f;

bool lowReport(const FollowSample &s) { return s.ground || (s.hasAlt && s.altFt < ESTIMATE_MAX_FT); }

bool taxiing(const FollowSample &s) { return s.ground && s.hasGs && s.gsKt >= STOPPED_KT && s.hasTrack; }

float smoothstep(float u) { return u * u * (3.0f - 2.0f * u); }

}  // namespace

const char *RunwaySim::phase() const {
    switch (phase_) {
    case LINED_UP: return "lined up";
    case TAKEOFF: return "take-off";
    case LANDING: return "landing";
    default: return "none";
    }
}

uint32_t RunwaySim::keepMs() const { return phase_ != NONE ? RUNWAY_SIM_LOST_MS : ESTIMATE_LOST_MS; }

bool RunwaySim::simulating(uint32_t ms) const {
    return haveFresh_ && (phase_ != NONE || (int32_t)(ms - easeEndMs_) < 0);
}

// How long one stopped lined up is left before its roll is taken to have
// begun: until the poll after next would have had it again, by the gap since
// the report before - the zoom's ten seconds, or the radar's thirty.
uint32_t RunwaySim::waitFor(const FollowSample &prev, bool hadPrev) const {
    uint32_t gap = hadPrev ? fresh_.ms - prev.ms : 10000;
    return std::min(std::max(gap, (uint32_t)10000), (uint32_t)40000) + 3000;
}

void RunwaySim::report(const FollowSample &s) {
    if (haveFresh_ && (int32_t)(s.ms - fresh_.ms) < 0) {
        return;  // older than what it has
    }
    if (lowReport(s)) {
        history_.push_back(s);
        while (!history_.empty() &&
               (s.ms - history_.front().ms > SIM_HISTORY_MS || history_.size() > SIM_HISTORY_MAX)) {
            history_.erase(history_.begin());
        }
    } else if (!history_.empty()) {
        std::vector<FollowSample>().swap(history_);  // climbed away: its room back
    }
    if (!s.ground) {
        airMs_ = s.ms;
        seenAir_ = true;
    }
    if (!s.hasPos || s.posStale) {
        return;
    }
    // Moving, yet where it was: the same report again, in the other list.
    if (haveFresh_ && s.lat == fresh_.lat && s.lon == fresh_.lon && s.hasGs && s.gsKt >= STOPPED_KT) {
        return;
    }
    bool hadPrev = haveFresh_ && s.ms - fresh_.ms <= keepMs();
    bool wasUnderWay = hadPrev && phase_ != NONE;
    bool wasSim = hadPrev && simulating(s.ms);
    FollowEstimate shown = {};
    if (hadPrev) {
        shown = at(s.ms);
    }
    FollowSample prev = fresh_;
    fresh_ = s;
    haveFresh_ = true;
    if (!hadPrev) {
        phase_ = NONE;
    }
    bool turnOff = advance(prev, hadPrev);
    easeStartMs_ = easeEndMs_ = s.ms;
    turning_ = false;
    if (!hadPrev || (!wasSim && phase_ == NONE && !taxiing(s))) {
        return;  // the feed's, as it always was
    }
    // From where it was shown, to where this report puts it.
    FollowEstimate now = model(s.ms, 1e9f);
    float k = nmPerDegLonAt(s.lat);
    float d = hypotf((shown.lon - now.lon) * k, (shown.lat - now.lat) * 60.0f);
    float easeS;
    if (turnOff) {
        turning_ = true;
        fromLat_ = shown.lat;
        fromLon_ = shown.lon;
        fromTrack_ = shown.track;
        easeS = std::min(std::max(d / TURN_TAXI_KT * 3600.0f, TURN_MIN_S), TURN_MAX_S);
    } else {
        if (d < 0.001f && shown.altFt == now.altFt) {
            return;  // nothing to ease
        }
        dLat_ = shown.lat - now.lat;
        dLon_ = shown.lon - now.lon;
        dAlt_ = (float)(shown.altFt - now.altFt);
        float halfKt = std::max((float)now.gsKt, EASE_MIN_KT) / 2.0f;
        easeS = std::min(std::max(d / halfKt * 3600.0f, EASE_MIN_S), EASE_MAX_S);
        // Already handed back, and still easing: the feed's from here.
        if (!wasUnderWay && phase_ == NONE) {
            easeS = EASE_MIN_S;
        }
    }
    easeEndMs_ = s.ms + (uint32_t)(easeS * 1000.0f);
}

// fresh_ just in: whether the take-off or landing under way goes on, ends,
// or - not under way - begins. True where a landing ends by turning off.
bool RunwaySim::advance(const FollowSample &prev, bool hadPrev) {
    const FollowSample &s = fresh_;
    float gs = s.hasGs ? (float)s.gsKt : 0.0f;
    float along = 0, offset = 0, off = 0;
    if (phase_ != NONE) {
        relate(rw_, s.lat, s.lon, along, offset);
        off = s.hasTrack ? angleOff(s.track, headingOf(rw_)) : 0.0f;
    }
    bool onPavement = offset <= rw_.halfWidthNm + RUNWAY_EDGE_NM && along >= -RUNWAY_EDGE_NM &&
                      along <= rw_.len + RUNWAY_EDGE_NM;
    float aglFt = s.hasAlt ? (float)(s.altFt - rw_.fieldFt) : 0.0f;
    switch (phase_) {
    case LINED_UP:
        if (s.ground && onPavement && (gs < STOPPED_KT || off <= LINED_UP_HEADING_DEG)) {
            if (gs < STOPPED_KT) {
                rollMs_ = s.ms + waitFor(prev, hadPrev);  // still waiting
            } else {
                phase_ = TAKEOFF;  // and off
            }
            return false;
        }
        if (s.ground) {
            phase_ = NONE;  // moved off it: taxiing on, perhaps to another
            break;
        }
        phase_ = TAKEOFF;  // off already, unseen on its roll
        [[fallthrough]];
    case TAKEOFF:
        if (s.ground) {
            bool rejected = prev.ground && prev.hasGs && gs < prev.gsKt - REJECTED_DROP_KT;
            if (onPavement && off <= OFF_LINE_DEG && !rejected) {
                return false;
            }
        } else if (aglFt <= HANDOVER_AGL_FT && along <= rw_.len + HANDOVER_PAST_END_NM && offset <= OFF_LINE_NM &&
                   off <= OFF_LINE_DEG) {
            return false;
        }
        phase_ = NONE;  // up and away, turning, or stopped: the feed's again
        return false;
    case LANDING:
        if (!s.ground) {
            bool goingAround = s.hasVs && s.vsFpm >= GO_AROUND_FPM;
            if (!goingAround && along <= rw_.len && offset <= OFF_LINE_NM && off <= OFF_LINE_DEG) {
                return false;
            }
            phase_ = NONE;
            return false;
        }
        // Stopped, its track says little.
        if (onPavement && (gs < STOPPED_KT || off <= OFF_LINE_DEG)) {
            return false;
        }
        phase_ = NONE;
        return true;  // off the runway, or turning off it
    case NONE:
        break;
    }
    start(prev, hadPrev);
    return false;
}

// Whether fresh_, nothing under way, begins a take-off or a landing.
void RunwaySim::start(const FollowSample &prev, bool hadPrev) {
    FollowSample s = fresh_;
    if (!lowReport(s)) {
        return;
    }
    if (!s.hasTrack) {
        s.hasTrack = trackFromHistory(history_, s, s.track);
    }
    float gs = s.hasGs ? (float)s.gsKt : 0.0f;
    Lineup l;
    if (!s.ground) {
        if (s.hasVs && s.vsFpm >= CLIMBING_FPM) {
            return;
        }
        if (findLineup(s, l) && l.along >= -FINAL_START_NM &&
            !(s.hasAlt && s.altFt - l.fieldFt > FINAL_START_AGL_FT)) {
            phase_ = LANDING;
            rw_ = lineOf(s, l);
        }
        return;
    }
    // Landed, a few minutes ago - longer than its history goes back.
    bool landed = seenAir_ && s.ms - airMs_ <= LANDED_WITHIN_MS;
    if (landed || !followDeparting(history_, s)) {
        // Picked up on its rollout, never seen in the air.
        if (gs >= ROLLING_KT && findLineup(s, l)) {
            phase_ = LANDING;
            rw_ = lineOf(s, l);
        }
        return;
    }
    bool thrust = hadPrev && prev.ground && prev.hasGs && gs >= THRUST_MIN_KT && gs >= prev.gsKt + THRUST_GAIN_KT;
    if (gs >= TAKEOFF_ROLLING_KT || thrust) {
        bool found = (gs >= TAKEOFF_ROLLING_KT && findLineup(s, l)) || runwayUnder(s, history_, l);
        if (!found) {
            return;
        }
        RunwayLine r = lineOf(s, l);
        // Crossing it rather than going down it.
        float off = angleOff(s.track, headingOf(r));
        if (s.hasTrack && off > ON_RUNWAY_DEG && off < 180.0f - ON_RUNWAY_DEG) {
            return;
        }
        phase_ = TAKEOFF;
        rw_ = r;
        return;
    }
    if (gs < STOPPED_KT && s.hasTrack && runwayUnder(s, history_, l, true)) {
        phase_ = LINED_UP;
        rw_ = lineOf(s, l);
        rollMs_ = s.ms + waitFor(prev, hadPrev);
    }
}

FollowEstimate RunwaySim::model(uint32_t ms, float maxAgeS) const {
    const FollowSample &s = fresh_;
    float t = (int32_t)(ms - s.ms) > 0 ? std::min((ms - s.ms) / 1000.0f, maxAgeS) : 0.0f;
    if (phase_ == NONE) {
        FollowEstimate e = estimateFollowed(s, t, history_);
        if (taxiing(s) && e.lat == s.lat && e.lon == s.lon) {
            float d = s.gsKt * std::min(t, TAXI_CARRY_S) / 3600.0f;
            float a = s.track * (float)M_PI / 180.0f;
            e.lat += cosf(a) * d / 60.0f;
            e.lon += sinf(a) * d / nmPerDegLonAt(s.lat);
            e.how = "taxiing";
        }
        return e;
    }
    float along, offset;
    relate(rw_, s.lat, s.lon, along, offset);
    float gs = s.hasGs ? (float)s.gsKt : 0.0f;
    switch (phase_) {
    case LINED_UP: {
        uint32_t when = s.ms + (uint32_t)(t * 1000.0f);
        if ((int32_t)(when - rollMs_) < 0) {
            return onLine(rw_, along, rw_.fieldFt, true, 0, 0, "held: lined up");
        }
        return takeoffFrom(rw_, along, 0, false, 0, 0, (when - rollMs_) / 1000.0f);
    }
    case TAKEOFF:
        return takeoffFrom(rw_, along, gs, !s.ground, s.hasAlt ? s.altFt : rw_.fieldFt, s.hasVs ? s.vsFpm : 0, t);
    default:
        return landingFrom(rw_, along, gs, !s.ground, s.hasAlt, s.altFt, t);
    }
}

FollowEstimate RunwaySim::at(uint32_t ms, float maxAgeS) const {
    FollowEstimate e = model(ms, maxAgeS);
    int32_t span = (int32_t)(easeEndMs_ - easeStartMs_), since = (int32_t)(ms - easeStartMs_);
    if (span <= 0 || since >= span) {
        return e;
    }
    float u = smoothstep(since <= 0 ? 0.0f : (float)since / span);
    if (!turning_) {
        e.lat += dLat_ * (1.0f - u);
        e.lon += dLon_ * (1.0f - u);
        e.altFt += (int32_t)(dAlt_ * (1.0f - u));
        return e;
    }
    // Off the runway, along a curve leaving it the way it was going and
    // arriving the way the report has it going - in nm from the turn's start.
    float k = nmPerDegLonAt(fromLat_);
    float x3 = (e.lon - fromLon_) * k, y3 = (e.lat - fromLat_) * 60.0f;
    float len = hypotf(x3, y3);
    float h0 = fromTrack_ * (float)M_PI / 180.0f, h3 = e.track * (float)M_PI / 180.0f;
    float x1 = sinf(h0) * len / 3.0f, y1 = cosf(h0) * len / 3.0f;
    float x2 = x3 - sinf(h3) * len / 3.0f, y2 = y3 - cosf(h3) * len / 3.0f;
    float v = 1.0f - u;
    float x = 3 * v * v * u * x1 + 3 * v * u * u * x2 + u * u * u * x3;
    float y = 3 * v * v * u * y1 + 3 * v * u * u * y2 + u * u * u * y3;
    float dx = 3 * v * v * x1 + 6 * v * u * (x2 - x1) + 3 * u * u * (x3 - x2);
    float dy = 3 * v * v * y1 + 6 * v * u * (y2 - y1) + 3 * u * u * (y3 - y2);
    e.lat = fromLat_ + y / 60.0f;
    e.lon = fromLon_ + x / k;
    if (hypotf(dx, dy) > 1e-6f) {
        e.track = atan2f(dx, dy) * 180.0f / (float)M_PI;
        if (e.track < 0) {
            e.track += 360.0f;
        }
    }
    e.ground = true;
    e.vsFpm = 0;
    e.gsKt = (int)lroundf(len / (span / 1000.0f) * 3600.0f);
    e.how = "turning off";
    return e;
}
