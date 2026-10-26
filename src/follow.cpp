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

int followRangeFor(const Aircraft &ac, bool fine) {
    Airport a;
    float d;
    if (!followDestination(ac, a, d)) {
        return FOLLOW_RANGE_NM;
    }
    // With a quarter again around it, so the airport sits inside the outer
    // ring rather than on it.
    if (fine) {
        for (int r : FOLLOW_FINE_STEPS_NM) {
            if (d * 1.25f <= r) {
                return r;
            }
        }
        return FOLLOW_RANGE_NM;
    }
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
// On a runway too short for those, both harder - by up to this, 8.8kt/s
// at most, about as hard as an airliner's brakes go.
constexpr float ROLLOUT_HARDEST = 4.0f;
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
// Nor faster than this, however low: a Typhoon running in along the runway at
// 1,000ft and 400kt, to break into the circuit as they do at Lossiemouth,
// would be shown diving onto it and rolling down it at 400kt.
constexpr float LANDING_MAX_KT = 250.0f;
// Nor - see couldLand() - higher over the field than this many feet a mile
// from the touchdown point, seven and a half degrees, and this much more:
// more where its height is the feed's pressure altitude, out by the day's
// pressure. Nor level, or not coming down faster than this, more than this
// high over the field and this close to the touchdown point.
constexpr float STEEPEST_FT_PER_NM = 800.0f;
constexpr float HEIGHT_SPARE_FT = 300.0f, HEIGHT_SPARE_PRESSURE_FT = 800.0f;
constexpr int LEVEL_FPM = 300;
constexpr float LEVEL_ABOVE_FT = 600.0f, LEVEL_WITHIN_NM = 3.0f;
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

// Whether `s`, in the air lined up on a runway `along` from its threshold and
// `aglFt` over the field, could be landing on it: not too high to get down
// to the touchdown point by any approach flown, nor flying level well above
// the field close in - as one running in along the runway to break into the
// circuit, joining overhead or passing low over the field is, slower than a
// fast jet's run in and lined up as a landing is.
bool couldLand(const FollowSample &s, float along, float aglFt) {
    float d = std::max(TOUCHDOWN_NM - along, 0.0f);
    if (aglFt > d * STEEPEST_FT_PER_NM + (s.pressureFt != 0 ? HEIGHT_SPARE_FT : HEIGHT_SPARE_PRESSURE_FT)) {
        return false;
    }
    return !(s.hasVs && s.vsFpm > -LEVEL_FPM && aglFt > LEVEL_ABOVE_FT && d < LEVEL_WITHIN_NM);
}

struct Lineup {
    float thrX, thrY;  // the threshold it is landing over, in nm from its last position
    float ux, uy;      // along the runway, the way it is landing
    float len;
    float along;       // where it was, from the threshold
    int32_t fieldFt;
    float halfWidthNm;
    const char *ident;    // the end it is lined up on
    const char *airport;  // the airport's IATA code
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
                out = {fx, fy, ux, uy, len, along, rw.elevationFt, rw.widthFt / 2.0f / FT_PER_NM, from.ident, rw.code};
                found = true;
            }
        }
    }
    return found;
}

}  // namespace

bool followDeparting(const SampleList &history, const FollowSample &last) {
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
// In the air, carried on between reports round the turn it is in - its
// track's change from a report a few seconds before - rather than straight
// off it at a tangent: a fighter in a 40-degree bank was a hundred metres off
// its circle in five seconds, and put back with a swerve at every report. A
// turn slower than the least of these is taken for none, one faster for a
// misreading; and one is followed round for no longer than the most, then
// straight on.
constexpr float TURN_MIN_DEG_S = 0.4f, TURN_MAX_DEG_S = 8.0f, TURN_CARRY_S = 30.0f;
constexpr uint32_t TURN_FROM_MS = 2500, TURN_WITHIN_MS = 30000;

// Its rate of turn, degrees a second, right positive: from its track now and
// in a report at least a few seconds before, so a jitter in the track isn't
// read as a turn. False if it isn't turning, or nothing says.
bool turnRateFromHistory(const SampleList &history, const FollowSample &last, float &rate) {
    if (!last.hasTrack) {
        return false;
    }
    for (auto it = history.rbegin(); it != history.rend(); ++it) {
        const FollowSample &s = *it;
        if (s.ms >= last.ms || !s.hasTrack || s.ground) {
            continue;
        }
        uint32_t span = last.ms - s.ms;
        if (span < TURN_FROM_MS) {
            continue;
        }
        if (span > TURN_WITHIN_MS) {
            return false;
        }
        float turned = fmodf(last.track - s.track + 540.0f, 360.0f) - 180.0f;
        rate = turned / (span / 1000.0f);
        return fabsf(rate) >= TURN_MIN_DEG_S && fabsf(rate) <= TURN_MAX_DEG_S;
    }
    return false;
}

bool trackFromHistory(const SampleList &history, const FollowSample &last, float &track) {
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

// Its ground speed, knots, from how far it went since its last report a
// little way off - for a report the feed gave no speed with. Over at least
// 10 seconds and no more than a minute: closer, a position's few metres
// of error are most of the distance.
bool speedFromHistory(const SampleList &history, const FollowSample &last, float &knots) {
    for (auto it = history.rbegin(); it != history.rend(); ++it) {
        const FollowSample &s = *it;
        if (s.ms >= last.ms || !s.hasPos || s.posStale) {
            continue;
        }
        uint32_t ms = last.ms - s.ms;
        if (ms > 60000) {
            break;
        }
        if (ms < 10000) {
            continue;
        }
        float dNorth = (last.lat - s.lat) * 60.0f;
        float dEast = (last.lon - s.lon) * 60.0f * cosf(last.lat * (float)M_PI / 180.0f);
        knots = hypotf(dNorth, dEast) / (ms / 3600000.0f);
        return true;
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
bool runwayUnder(const FollowSample &last, const SampleList &history, Lineup &out,
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
                out = {ax, ay, ux, uy, len, along, rw.elevationFt, rw.widthFt / 2.0f / FT_PER_NM, rw.end[0].ident,
                       rw.code};
            } else {
                out = {bx, by, -ux, -uy, len, len - along, rw.elevationFt, rw.widthFt / 2.0f / FT_PER_NM,
                       rw.end[1].ident, rw.code};
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
            l.fieldFt, l.ident, l.airport};
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
// from the height it gave to the field's by then, which lines it up with the
// ground whatever the day's pressure leaves it out by; or on a three-degree
// glide, where it gave none. Then the rollout: braking to the
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
    // Braked harder where the runway left is too short to stop on as it
    // usually would - down at 140kt at Glasgow's aiming point, it needs
    // 2.35km and has 2.15 - so it slows to a stop short of the far end,
    // not cut off there at 37kt, or at 82 on Amsterdam's 04; but no harder
    // than an airliner can.
    float brakeA = ROLLOUT_DECEL_KT_PER_S, taxiA = TAXI_DECEL_KT_PER_S;
    float hi = std::max(gs, TURN_OFF_KT), lo = std::min(gs, TURN_OFF_KT);
    float need = ((hi * hi - TURN_OFF_KT * TURN_OFF_KT) / (2 * brakeA) + lo * lo / (2 * taxiA)) / 3600.0f;
    float room = r.len - STOP_SHORT_NM - start;
    if (room > 0.01f && need > room) {
        float harder = std::min(need / room, ROLLOUT_HARDEST);
        brakeA *= harder;
        taxiA *= harder;
    }
    float brakeS = std::min(rollS, gs > TURN_OFF_KT ? (gs - TURN_OFF_KT) / brakeA : 0.0f);
    along += (gs * brakeS - brakeA * brakeS * brakeS / 2.0f) / 3600.0f;
    float v = gs - brakeA * brakeS;
    float slowS = std::min(rollS - brakeS, std::max(v, 0.0f) / taxiA);
    along += (v * slowS - taxiA * slowS * slowS / 2.0f) / 3600.0f;
    v -= taxiA * slowS;
    if (along > r.len - STOP_SHORT_NM) {
        along = std::max(start, r.len - STOP_SHORT_NM);
        v = 0;
    }
    return onLine(r, along, r.fieldFt, true, v, 0, v >= 1.0f ? "rollout" : "rollout: stopped");
}

}  // namespace

// How fast `last` was speeding up, knots a second - slowing, less than
// nought - from its report before it, two seconds before it at least: no
// more than four either way, a helicopter's flare to a hover, and nought if
// it was nearly steady. False, with no report before it to tell.
bool speedingOf(const SampleList &history, const FollowSample &last, float &rate) {
    for (auto it = history.rbegin(); it != history.rend(); ++it) {
        if (it->ms + 2000 > last.ms || !it->hasGs || !last.hasGs) {
            continue;
        }
        rate = (last.gsKt - it->gsKt) / ((last.ms - it->ms) / 1000.0f);
        rate = fabsf(rate) < 0.3f ? 0.0f : std::max(-4.0f, std::min(4.0f, rate));
        return true;
    }
    rate = 0;
    return false;
}

// A rotorcraft coming down this fast, this low - above the sea, there being
// no telling the ground's height - and going between these speeds, with no
// report before to say how its speed is going: taken to be slowing this
// much, knots a second, for its landing.
constexpr int ROTOR_APPROACH_FPM = -200, ROTOR_APPROACH_BELOW_FT = 4000;
constexpr float ROTOR_APPROACH_MIN_KT = 10, ROTOR_APPROACH_MAX_KT = 120, ROTOR_APPROACH_SLOWING = 0.8f;

FollowEstimate estimateFollowed(const FollowSample &given, float ageS, const SampleList &history) {
    FollowSample last = given;
    if (!last.hasTrack) {
        last.hasTrack = trackFromHistory(history, last, last.track);
    }
    bool departing = followDeparting(history, last);
    FollowEstimate e = {last.lat, last.lon, last.track, last.hasAlt ? aboveSeaFt(last) : 0, last.ground,
                        last.hasGs ? last.gsKt : 0, last.hasVs ? last.vsFpm : 0, "held", last.pressureFt};
    float gs = last.hasGs ? (float)last.gsKt : 0.0f;
    // In the air with no speed given - some feeds' military contacts - how
    // fast it went between reports: carried on at that, where it was held
    // where it was, standing still in the air, as only what hovers does.
    if (!last.hasGs && !last.ground && !last.rotorcraft && speedFromHistory(history, last, gs)) {
        e.gsKt = (int)lroundf(gs);
    }
    float nmPerDegLon = 60.0f * cosf(last.lat * (float)M_PI / 180.0f);
    auto place = [&](float x, float y) {
        e.lat = last.lat + y / 60.0f;
        e.lon = last.lon + x / nmPerDegLon;
    };

    // Only low, or rolling, is a runway worth looking for - and the look is a
    // scan of the airports around it, so it is not made for the rest. Nor
    // for a rotorcraft, which needs none: one coming in to a spot beside a
    // runway, lined up with it, was put on its centreline and landed on it
    // from 250ft.
    Lineup l;
    bool runways = !last.rotorcraft;
    bool rolling = !last.ground || gs >= (departing ? TAKEOFF_ROLLING_KT : ROLLING_KT);
    bool lowEnough = last.ground || !last.hasAlt || last.altFt < ESTIMATE_MAX_FT;
    bool climbing = !last.ground && last.hasVs && last.vsFpm >= CLIMBING_FPM;
    bool fast = !last.ground && gs > LANDING_MAX_KT;
    // In the air with no speed, given or worked out - its first report, none
    // before to go by - nothing to fly it in by: landed at nought, it was put
    // down stopped at the touchdown point, and set back in the air 8km out
    // at the next report.
    bool still = !last.ground && gs <= 0;
    bool lined = runways && rolling && lowEnough && !climbing && !fast && !still && findLineup(last, l);
    if (lined && !last.ground && last.hasAlt && !couldLand(last, l.along, (float)(aboveSeaFt(last) - l.fieldFt))) {
        lined = false;
    }
    if (!lined && runways && rolling && last.ground && departing) {
        lined = runwayUnder(last, history, l);
    }
    // Stopped on the runway, lined up: its roll is taken to begin once it has
    // been lost for LINED_UP_WAIT_S, from a standstill.
    if (!lined && runways && !rolling && last.ground && departing && last.hasTrack &&
        runwayUnder(last, history, l, true)) {
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
        float rate = 0;
        float travelled = -1;  // straight on, slowing: how far, nm, for its height along its way down
        if (turnRateFromHistory(history, last, rate)) {
            // Round the arc for as long as it is likely to keep turning
            // unseen, then straight on the way it was heading by then.
            e.how = "round its turn";
            float t = std::min(ageS, TURN_CARRY_S), w = rate * (float)M_PI / 180.0f, v = gs / 3600.0f;
            float a0 = last.track * (float)M_PI / 180.0f, a1 = a0 + w * t, rest = v * (ageS - t);
            place(v / w * (cosf(a0) - cosf(a1)) + sinf(a1) * rest, v / w * (sinf(a1) - sinf(a0)) + cosf(a1) * rest);
            e.track = fmodf(last.track + rate * t + 720.0f, 360.0f);
        } else {
            e.how = "straight on";
            float d = gs * ageS / 3600.0f;
            // A rotorcraft slowing - to a hover, to set down - keeps slowing
            // as it was between its last two reports, to a stop: carried on
            // at its last speed, it sailed past where it stopped and came
            // back at the next report. Speeding up, off from a hover, it
            // keeps on doing so for as long again as it was seen to, and no
            // more than 40 knots on.
            float rate = 0;
            bool known = last.rotorcraft && speedingOf(history, last, rate);
            // One coming down low with nothing yet to say how its speed is
            // going - its first report - taken to be slowing for its
            // landing, as one coming down low most often is: carried on at
            // its speed for the 40 seconds to the next report a busy feed
            // managed, one slowing to a hover by a runway overshot it by
            // 330m and went back.
            if (last.rotorcraft && !known && last.hasVs && last.vsFpm <= ROTOR_APPROACH_FPM && last.hasAlt &&
                last.altFt < ROTOR_APPROACH_BELOW_FT && gs >= ROTOR_APPROACH_MIN_KT && gs <= ROTOR_APPROACH_MAX_KT) {
                rate = -ROTOR_APPROACH_SLOWING;
            }
            if (rate < 0) {
                e.how = "slowing";
                float t = std::min(ageS, gs / -rate);
                d = (gs * t + rate * t * t / 2) / 3600.0f;
                e.gsKt = (int)lroundf(gs + rate * t);
                travelled = d;
            } else if (rate > 0) {
                e.how = "speeding up";
                float t = std::min(ageS, 40 / rate);
                d = (gs * t + rate * t * t / 2 + (gs + rate * t) * (ageS - t)) / 3600.0f;
                e.gsKt = (int)lroundf(gs + rate * t);
            }
            float a = last.track * (float)M_PI / 180.0f;
            place(sinf(a) * d, cosf(a) * d);
        }
        if (last.hasAlt && last.hasVs && travelled >= 0 && gs > 0) {
            // Slowing, it comes down along the line it was coming down -
            // the less steeply in feet a minute as it slows, to its hover at
            // the end of it. Its rate held, one slowing to a hover came down
            // through the ground short of it, and was shown skimming along
            // the grass at 30 knots.
            e.altFt = std::max((int32_t)0, aboveSeaFt(last) + (int32_t)lroundf(60.0f * last.vsFpm / gs * travelled));
            e.vsFpm = (int)lroundf((float)last.vsFpm * e.gsKt / gs);
        } else if (last.hasAlt && last.hasVs && gs > 0) {
            // Climbing or coming down, the rate fading over half a minute or
            // so - it levels off somewhere, unseen - so no more than half a
            // minute's worth of it in all: held, a descent lost five miles
            // out came down for minutes, through the ground, and was shown
            // standing in a field.
            constexpr float FADE_S = 30.0f;
            float done = FADE_S * (1 - expf(-ageS / FADE_S));
            e.altFt = std::max((int32_t)0, aboveSeaFt(last) + (int32_t)lroundf(last.vsFpm * done / 60.0f));
            e.vsFpm = (int)lroundf(last.vsFpm * expf(-ageS / FADE_S));
        }
        // With no speed to carry it on by, held at its last height too.
        return e;
    }

    RunwayLine r = lineOf(last, l);
    e = last.ground && departing ? takeoffFrom(r, l.along, gs, false, 0, 0, ageS)
                                 : landingFrom(r, l.along, gs, !last.ground, last.hasAlt, aboveSeaFt(last), ageS);
    e.pressureFt = last.pressureFt;
    return e;
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
// On the ground this recently, it is not out on final, whatever the feed
// says of it now - see start().
constexpr uint32_t OFF_GROUND_WITHIN_MS = 60000;
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
// between these - and one to its height at no more than this.
constexpr float EASE_MIN_S = 3.0f, EASE_MAX_S = 20.0f;
constexpr float EASE_MIN_KT = 20.0f;
constexpr float EASE_FPM = 600.0f;
// A turn off the runway is taxied at about this, over between these.
constexpr float TURN_TAXI_KT = 20.0f;
constexpr float TURN_MIN_S = 3.0f, TURN_MAX_S = 10.0f;
// Taxiing, it is carried on along its track for up to this long - about a
// poll of the zoom's - and then left where it was, since it could turn any
// way; and each report is eased into, so it neither stops nor jumps between
// them.
constexpr float TAXI_CARRY_S = 12.0f;
// A landing put back down the runway is eased by its roll, not the clock:
// held this long, its stop included, unless a report comes first.
constexpr uint32_t ROLL_EASE_MS = 600000;

bool lowReport(const FollowSample &s) { return s.ground || (s.hasAlt && s.altFt < ESTIMATE_MAX_FT); }

// Taxiing, `track` the way: its report's, or, where it gave none, the way
// its reports before have it going.
bool taxiingAlong(const FollowSample &s, const SampleList &history, float &track) {
    track = s.track;
    return s.ground && s.hasGs && s.gsKt >= STOPPED_KT && (s.hasTrack || trackFromHistory(history, s, track));
}

// Its latest report on the ground in `history` from before `ms`, no more
// than `withinMs` before it; null if there is none.
const FollowSample *groundBefore(const SampleList &history, uint32_t ms, uint32_t withinMs) {
    for (auto it = history.rbegin(); it != history.rend(); ++it) {
        if (it->ms >= ms) {
            continue;
        }
        if (ms - it->ms > withinMs) {
            break;
        }
        if (it->ground) {
            return &*it;
        }
    }
    return nullptr;
}

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

RunwaySim::Movement RunwaySim::movement() const {
    switch (phase_) {
    case LINED_UP:
    case TAKEOFF: return Movement::TAKEOFF;
    case LANDING: return Movement::LANDING;
    default: return Movement::NONE;
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
        SampleList().swap(history_);  // climbed away: its room back
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
    // Landing, and this report on final with no speed - one in a while, from
    // some feeds - kept landing at the speed it was shown: ended there, it
    // was carried on at a speed worked out from positions of different ages,
    // 215 knots for one at 149, and slid back as much as a kilometre at the
    // next report.
    if (hadPrev && phase_ == LANDING && !s.ground && !s.hasGs && shown.gsKt > 0) {
        fresh_.hasGs = true;
        fresh_.gsKt = (int16_t)shown.gsKt;
    }
    bool turnOff = advance(prev, hadPrev);
    easeStartMs_ = easeEndMs_ = altEndMs_ = s.ms;
    turning_ = false;
    rollOverNm_ = 0;
    // In the air still by this report, though shown down already - put down
    // a little early, the feed having lost it fast on final - and over the
    // runway, not climbing away: kept down, not shown lifting off again to
    // touch down twice.
    heldDown_ = false;
    if (hadPrev && phase_ == LANDING && !s.ground && shown.ground && !(s.hasVs && s.vsFpm >= CLIMBING_FPM)) {
        float along, offset;
        relate(rw_, s.lat, s.lon, along, offset);
        heldDown_ = along >= 0;
    }
    float alongTrack;
    if (!hadPrev || (!wasSim && phase_ == NONE && !taxiingAlong(s, history_, alongTrack))) {
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
        // Off the runway behind where it was shown - found again on the exit
        // it took, having braked harder than the simulation does: left
        // facing the report, and taxied back to it - not swung round from
        // the way it was going and driven back up the runway at 80 knots.
        float toReport = atan2f((now.lon - shown.lon) * k, (now.lat - shown.lat) * 60.0f) * 180.0f / (float)M_PI;
        if (angleOff(toReport, shown.track) > 90.0f) {
            fromTrack_ = toReport;
            easeS = std::max(d / TURN_TAXI_KT * 3600.0f, TURN_MIN_S);
        }
    } else {
        if (d < 0.001f && shown.altFt == now.altFt) {
            return;  // nothing to ease
        }
        dLat_ = shown.lat - now.lat;
        dLon_ = shown.lon - now.lon;
        dAlt_ = (float)(shown.altFt - now.altFt);
        float halfKt = std::max((float)now.gsKt, EASE_MIN_KT) / 2.0f;
        easeS = std::min(std::max(d / halfKt * 3600.0f, EASE_MIN_S), EASE_MAX_S);
        // Its height put right - gone around, or a report higher or lower
        // than it was shown - to be seen to settle rather than jump; but by
        // the touchdown. Its height alone: its place too, put right in the
        // three seconds left - a report a few seconds old, behind where it
        // was shown - went back faster than it flew.
        float altS = std::max(easeS, std::min(fabsf(dAlt_) / EASE_FPM * 60.0f, EASE_MAX_S));
        if (phase_ == LANDING && !s.ground && !heldDown_) {
            float along, offset;
            relate(rw_, s.lat, s.lon, along, offset);
            float toTouchdownS = (TOUCHDOWN_NM - along) / std::max((float)now.gsKt, EASE_MIN_KT) * 3600.0f;
            altS = std::min(altS, std::max(toTouchdownS - 1.0f, EASE_MIN_S));
        }
        // Landing, and put back down the runway - found again behind where
        // the simulation had it, which brakes more gently than most: taken
        // in as it goes on, not over so many seconds, so it is seen to slow
        // and never to go back. Over eight times the distance where its roll
        // leaves room - slowed by a fifth at most, so a few metres put right
        // don't dip it under the 40 knots the taxiway follower takes it at -
        // and one and a half at least, which keeps it going forwards;
        // stopping first, it is left that little on from the report rather
        // than slid back to it.
        if (phase_ == LANDING) {
            float nowAlong, shownAlong, stopAlong, offset;
            relate(rw_, now.lat, now.lon, nowAlong, offset);
            relate(rw_, shown.lat, shown.lon, shownAlong, offset);
            FollowEstimate stop = model(s.ms + ROLL_EASE_MS, 1e9f);
            relate(rw_, stop.lat, stop.lon, stopAlong, offset);
            float roll = stopAlong - nowAlong;
            if (shownAlong > nowAlong && roll > 0) {
                rollFromNm_ = nowAlong;
                rollOverNm_ = std::max(1.5f * d, std::min(8.0f * d, roll));
            }
        }
        // Already handed back, and still easing: the feed's from here. Not
        // taxiing, though: carried on along its track since its last report,
        // one that has it slowing - turned off and down to taxiing speed -
        // was eased back in three seconds whatever the distance.
        if (!wasUnderWay && phase_ == NONE && !taxiingAlong(s, history_, alongTrack)) {
            easeS = altS = EASE_MIN_S;
        }
        altEndMs_ = s.ms + (uint32_t)(altS * 1000.0f);
        easeS = rollOverNm_ > 0 ? ROLL_EASE_MS / 1000.0f : std::max(easeS, altS);
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
    float aglFt = s.hasAlt ? (float)(aboveSeaFt(s) - rw_.fieldFt) : 0.0f;
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
            if (!goingAround && s.hasGs && gs <= LANDING_MAX_KT && (!s.hasAlt || couldLand(s, along, aglFt)) &&
                along <= rw_.len && offset <= OFF_LINE_NM && off <= OFF_LINE_DEG) {
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
    // A rotorcraft takes off and lands where it likes, not down a runway:
    // one coming in lined up with one, slowing to a hover over a helipad by
    // it, was shown touching down and rolling out like an airliner. Its
    // reports, as they are.
    if (!lowReport(s) || s.rotorcraft) {
        return;
    }
    if (!s.hasTrack) {
        s.hasTrack = trackFromHistory(history_, s, s.track);
    }
    float gs = s.hasGs ? (float)s.gsKt : 0.0f;
    Lineup l;
    if (!s.ground) {
        // No speed given - a position worked out from when its signals
        // reached the receivers, or some feeds' military contacts - nothing
        // to fly it in by: a jet lined up 5nm out was put down stopped at
        // the touchdown point and slid there. Its reports, as they are.
        if ((s.hasVs && s.vsFpm >= CLIMBING_FPM) || gs > LANDING_MAX_KT || !s.hasGs) {
            return;
        }
        // On the ground a moment ago, it is not out on final, however it is
        // lined up. Some transponders say they are in the air from early in
        // the take-off roll - an E190 at Glasgow did at 48kt, and was cleared
        // to land - and some flick to it on a rollout. Departing, and down
        // the runway at a roll, it is its take-off.
        if (const FollowSample *g = groundBefore(history_, s.ms, OFF_GROUND_WITHIN_MS)) {
            if (followDeparting(history_, *g) && gs >= TAKEOFF_ROLLING_KT && findLineup(s, l) &&
                l.along >= -ON_RUNWAY_OFFSET_NM) {
                phase_ = TAKEOFF;
                rw_ = lineOf(s, l);
            }
            return;
        }
        if (findLineup(s, l) && l.along >= -FINAL_START_NM &&
            !(s.hasAlt && aboveSeaFt(s) - l.fieldFt > FINAL_START_AGL_FT) &&
            (!s.hasAlt || couldLand(s, l.along, (float)(aboveSeaFt(s) - l.fieldFt)))) {
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
        float track;
        if (taxiingAlong(s, history_, track) && e.lat == s.lat && e.lon == s.lon) {
            float d = s.gsKt * std::min(t, TAXI_CARRY_S) / 3600.0f;
            float a = track * (float)M_PI / 180.0f;
            e.lat += cosf(a) * d / 60.0f;
            e.lon += sinf(a) * d / nmPerDegLonAt(s.lat);
            e.how = "taxiing";
        }
        return e;
    }
    float along, offset;
    relate(rw_, s.lat, s.lon, along, offset);
    float gs = s.hasGs ? (float)s.gsKt : 0.0f;
    FollowEstimate e;
    switch (phase_) {
    case LINED_UP: {
        uint32_t when = s.ms + (uint32_t)(t * 1000.0f);
        e = (int32_t)(when - rollMs_) < 0 ? onLine(rw_, along, rw_.fieldFt, true, 0, 0, "held: lined up")
                                          : takeoffFrom(rw_, along, 0, false, 0, 0, (when - rollMs_) / 1000.0f);
        break;
    }
    case TAKEOFF: {
        // In the air by the feed's say, but slower than it lifts off at, not
        // climbing, and over the runway still: on its roll - see start().
        bool airborne = !s.ground && (gs >= ROTATE_KT || (s.hasVs && s.vsFpm >= CLIMBING_FPM) || along > rw_.len);
        e = takeoffFrom(rw_, along, gs, airborne, s.hasAlt ? aboveSeaFt(s) : rw_.fieldFt, s.hasVs ? s.vsFpm : 0, t);
        break;
    }
    default:
        e = landingFrom(rw_, along, gs, !s.ground && !heldDown_, s.hasAlt, aboveSeaFt(s), t);
        break;
    }
    e.pressureFt = s.pressureFt;
    return e;
}

FollowEstimate RunwaySim::at(uint32_t ms, float maxAgeS) const {
    FollowEstimate e = model(ms, maxAgeS);
    int32_t span = (int32_t)(easeEndMs_ - easeStartMs_), since = (int32_t)(ms - easeStartMs_);
    if (span <= 0 || since >= span) {
        return e;
    }
    float u = smoothstep(since <= 0 ? 0.0f : (float)since / span);
    if (!turning_) {
        // Its height over its own span, by the touchdown; its place by the
        // clock, or by how far it has rolled on - see report().
        int32_t altSpan = (int32_t)(altEndMs_ - easeStartMs_);
        float ua = altSpan <= 0 || since >= altSpan ? 1.0f : smoothstep(since <= 0 ? 0.0f : (float)since / altSpan);
        if (rollOverNm_ > 0) {
            float along, offset;
            relate(rw_, e.lat, e.lon, along, offset);
            float r = std::min(std::max((along - rollFromNm_) / rollOverNm_, 0.0f), 1.0f);
            u = smoothstep(r);
            // And its speed the speed it is shown going, slower than the
            // roll's while the offset is taken in: carried on at the roll's,
            // the view ran on ahead of it, and back.
            float d = hypotf(dLat_ * 60.0f, dLon_ * nmPerDegLonAt(e.lat));
            e.gsKt = (int)lroundf(e.gsKt * std::max(0.0f, 1.0f - d * 6.0f * r * (1.0f - r) / rollOverNm_));
        }
        e.lat += dLat_ * (1.0f - u);
        e.lon += dLon_ * (1.0f - u);
        e.altFt += (int32_t)(dAlt_ * (1.0f - ua));
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
