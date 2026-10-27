#include "follow.h"

#include <algorithm>
#include <cmath>
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
// The take-off: its kind's, as its report has it - see takeoff.h.
const TakeoffProfile &profileOf(const FollowSample &s) { return takeoffProfile(s.takeoffKind); }
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

// The runway `last` - in the air - is climbing away from: lined up with it
// the way it is going, over it or no further than `pastNm` past its far
// end, near its centreline; the nearest to it of those near.
bool findDeparture(const FollowSample &last, float pastNm, Lineup &out) {
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
                float off = fabsf(fmodf(last.track - atan2f(ux, uy) * 180.0f / (float)M_PI + 540.0f, 360.0f) - 180.0f);
                float along = -fx * ux - fy * uy, offset = fabsf(-fx * uy + fy * ux);
                if (off > LINED_UP_DEG || along < -ON_RUNWAY_OFFSET_NM || along > len + pastNm || offset > bestOffset) {
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

// --- A take-off, as takeoff.h has its kind ---------------------------------------

using Rolled = RunwaySim::Rolled;
using TakeoffAt = RunwaySim::TakeoffAt;

// Its nose coming up, it gathers speed at this share of the roll's rate,
// the wing taking the weight off the wheels: run on at the roll's, the
// lift-off came 4.6 kt faster than the departures recorded (takeoff.h).
constexpr float ROTATE_SHARE = 0.65f;
// The speed it gathers climbing away comes over about this many seconds;
// and above its kind's easeFt its climb eases to this share of itself over
// this long, the thrust cut back to accelerate.
constexpr float GAIN_S = 3.0f;
constexpr float CLIMB_EASED = 0.45f, CLIMB_EASE_S = 6.0f;
// To leave the runway STOP_SHORT_NM before its far end - a report further
// down it than its speed would have it - its roll quickened, by up to this
// many times.
constexpr float ROLL_HARDEST = 3.0f;

float clamp01(float u) { return u < 0 ? 0 : u > 1 ? 1 : u; }
// The later of two times on a clock that wraps.
uint32_t laterOf(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0 ? a : b; }

// Rolling, `t` seconds on from `v0` knots, its acceleration `k` times its
// kind's: its speed, and how far it has gone in nm - in closed form, as
// takeoff.h has it; and how long it takes to reach `v1`.
float rollKt(const TakeoffProfile &p, float v0, float k, float t) {
    if (p.dragPerS <= 0) {
        return v0 + k * p.accelKtS * t;
    }
    float top = p.accelKtS / p.dragPerS;
    return top - (top - v0) * expf(-k * p.dragPerS * t);
}
float rollNm(const TakeoffProfile &p, float v0, float k, float t) {
    if (p.dragPerS <= 0) {
        return (v0 * t + k * p.accelKtS * t * t / 2) / 3600.0f;
    }
    float top = p.accelKtS / p.dragPerS, kb = k * p.dragPerS;
    return (top * t - (top - v0) * (1 - expf(-kb * t)) / kb) / 3600.0f;
}
float rollSecs(const TakeoffProfile &p, float v0, float v1, float k) {
    if (v1 <= v0) {
        return 0;
    }
    if (p.dragPerS <= 0) {
        return (v1 - v0) / (k * p.accelKtS);
    }
    float top = p.accelKtS / p.dragPerS;
    v1 = std::min(v1, top - 0.5f);
    return v1 <= v0 ? 0 : logf((top - v0) / (top - v1)) / (k * p.dragPerS);
}
// The acceleration that takes it from `v0` to `v1` in `t` seconds, as a
// share of its kind's.
float rollShare(const TakeoffProfile &p, float v0, float v1, float t) {
    if (p.dragPerS <= 0) {
        return (v1 - v0) / (p.accelKtS * t);
    }
    float top = p.accelKtS / p.dragPerS;
    v1 = std::min(v1, top - 0.5f);
    return logf((top - v0) / (top - v1)) / (p.dragPerS * t);
}
// The speed it starts rotating at, to leave the runway at `liftoffKt` once
// its nose is up; and how fast it gathers speed meanwhile.
float rotateKtOf(const TakeoffProfile &p, float liftoffKt, float k) {
    float c = ROTATE_SHARE * k * p.rotateS;
    return (liftoffKt - c * p.accelKtS) / (1 - c * p.dragPerS);
}
float rotateAccelOf(const TakeoffProfile &p, float rotateKt, float k) {
    return ROTATE_SHARE * k * (p.accelKtS - p.dragPerS * rotateKt);
}
// Climbing away, `tau` seconds after leaving the runway at `liftoffKt`: its
// speed, and how far it has gone in nm.
float climbOutKt(const TakeoffProfile &p, float liftoffKt, float tau) {
    return tau <= 0 ? liftoffKt : liftoffKt + p.gainKt * (1 - expf(-tau / GAIN_S));
}
float climbOutNm(const TakeoffProfile &p, float liftoffKt, float tau) {
    if (tau <= 0) {
        return liftoffKt * tau / 3600.0f;
    }
    return (liftoffKt * tau + p.gainKt * (tau - GAIN_S * (1 - expf(-tau / GAIN_S)))) / 3600.0f;
}
// Its height over the field `tau` seconds after leaving the runway, its
// climb built up over its kind's buildS to `fpm` and eased above easeFt;
// how fast it is climbing then; and the seconds it takes to reach `ft`.
float climbFt(const TakeoffProfile &p, float fpm, float tau) {
    float tb = p.buildS;
    if (tau <= 0) {
        return 0;
    }
    if (tau < tb) {
        return fpm / 60 * tau * tau / (2 * tb);
    }
    float ft = fpm / 60 * (tau - tb / 2);
    if (ft <= p.easeFt) {
        return ft;
    }
    float easeAt = p.easeFt * 60 / fpm + tb / 2, u = (tau - easeAt) / CLIMB_EASE_S, cut = 1 - CLIMB_EASED;
    if (u >= 1) {
        return p.easeFt + fpm / 60 * CLIMB_EASE_S * (1 - cut / 2) +
               CLIMB_EASED * fpm / 60 * (tau - easeAt - CLIMB_EASE_S);
    }
    return p.easeFt + fpm / 60 * CLIMB_EASE_S * (u - cut * (u * u * u - u * u * u * u / 2));
}
float climbFpmAt(const TakeoffProfile &p, float fpm, float tau) {
    float tb = p.buildS;
    if (tau <= 0) {
        return 0;
    }
    if (tau < tb) {
        return fpm * tau / tb;
    }
    float easeAt = p.easeFt * 60 / fpm + tb / 2, u = clamp01((tau - easeAt) / CLIMB_EASE_S);
    return tau <= easeAt ? fpm : fpm * (1 - (1 - CLIMB_EASED) * u * u * (3 - 2 * u));
}
float climbSecsAt(const TakeoffProfile &p, float fpm, float ft) {
    float tb = p.buildS;
    if (ft <= 0) {
        return 0;
    }
    if (ft < fpm * tb / 120) {
        return sqrtf(2 * tb * 60 * ft / fpm);
    }
    if (ft <= p.easeFt) {
        return ft * 60 / fpm + tb / 2;
    }
    float easeAt = p.easeFt * 60 / fpm + tb / 2, eased = climbFt(p, fpm, easeAt + CLIMB_EASE_S);
    if (ft >= eased) {
        return easeAt + CLIMB_EASE_S + (ft - eased) * 60 / (CLIMB_EASED * fpm);
    }
    float lo = easeAt, hi = easeAt + CLIMB_EASE_S;
    for (int i = 0; i < 12; i++) {
        float mid = (lo + hi) / 2;
        (climbFt(p, fpm, mid) < ft ? lo : hi) = mid;
    }
    return (lo + hi) / 2;
}

// The roll from `along` at `kt` at `ms` - or from a standstill there, with
// `stand` - its acceleration `k` times its kind's, to lift off at
// `liftoffKt`: rotating at the speed for that, and off by the runway's end
// (`lenNm` its length) at the latest, its roll quickened if it must be.
// Faster than that already, a report's is rotating, off in what is left of
// it; `shown`, the one shown, its rotation begins there and takes as long
// as its kind's - its nose seen to come up.
Rolled rolledFrom(const TakeoffProfile &p, uint32_t ms, float along, float kt, float liftoffKt, float lenNm,
                  bool stand, float k = 1, bool shown = false) {
    Rolled r = {};
    r.ms = ms;
    r.along = along;
    r.kt = stand ? 0.0f : kt;
    r.stand = stand;
    r.k = k;
    float last = lenNm - STOP_SHORT_NM;
    for (int pass = 0; pass < 5; pass++) {
        float t0 = 0, a0 = along, v0 = r.kt;
        k = r.k;
        if (stand) {
            // Its thrust coming up: the acceleration rising from nought over
            // twice its kind's standS.
            t0 = 2 * p.standS;
            v0 = k * p.accelKtS * p.standS;
            a0 = along + 2 * k * p.accelKtS * p.standS * p.standS / 3 / 3600.0f;
        }
        float vrot = rotateKtOf(p, liftoffKt, k), rotS = t0, lift;
        if (v0 >= vrot) {
            // Rotating already: off in what is left of it.
            r.rotAlong = a0;
            r.rotKt = v0;
            r.rotAccel = rotateAccelOf(p, v0, k);
            lift = shown ? p.rotateS : std::max(0.5f, (liftoffKt - v0) / std::max(0.5f, r.rotAccel));
        } else {
            float rollS = rollSecs(p, v0, vrot, k);
            rotS += rollS;
            r.rotAlong = a0 + rollNm(p, v0, k, rollS);
            r.rotKt = vrot;
            r.rotAccel = rotateAccelOf(p, vrot, k);
            lift = p.rotateS;
        }
        r.rotMs = ms + (uint32_t)lroundf(rotS * 1000);
        r.offMs = r.rotMs + (uint32_t)lroundf(lift * 1000);
        r.offKt = r.rotKt + r.rotAccel * lift;
        r.offAlong = r.rotAlong + (r.rotKt * lift + r.rotAccel * lift * lift / 2) / 3600.0f;
        if (r.offAlong <= last || k >= ROLL_HARDEST) {
            break;
        }
        r.k = std::min(ROLL_HARDEST, k * (r.offAlong - along) / std::max(0.01f, last - along) * 1.05f);
    }
    return r;
}

// Where roll `r` has it at `ms`, before it leaves the runway: rolling, or
// rotating.
void rolledAt(const TakeoffProfile &p, const Rolled &r, uint32_t ms, TakeoffAt &a) {
    a.ground = true;
    a.ft = a.fpm = a.climbS = 0;
    a.rate = 1;
    a.offInS = (int32_t)(r.offMs - ms) / 1000.0f;
    if (r.air) {
        // Off already: rolled at its lift-off speed to there.
        a.rotating = true;
        a.along = r.offAlong + r.offKt * (int32_t)(ms - r.offMs) / 1000.0f / 3600.0f;
        a.kt = r.offKt;
        return;
    }
    if ((int32_t)(ms - r.rotMs) >= 0) {
        float x = (int32_t)(ms - r.rotMs) / 1000.0f;
        a.rotating = true;
        a.along = r.rotAlong + (r.rotKt * x + r.rotAccel * x * x / 2) / 3600.0f;
        a.kt = r.rotKt + r.rotAccel * x;
        return;
    }
    a.rotating = false;
    float t = (int32_t)(ms - r.ms) / 1000.0f;
    if (t <= 0) {
        a.along = r.along + r.kt * t / 3600.0f;
        a.kt = r.kt;
        return;
    }
    if (r.stand) {
        float rampS = 2 * p.standS, ka = r.k * p.accelKtS;
        if (t < rampS) {
            a.kt = ka * t * t / (4 * p.standS);
            a.along = r.along + ka * t * t * t / (12 * p.standS) / 3600.0f;
            return;
        }
        float v1 = ka * p.standS;
        a.along = r.along + 2 * ka * p.standS * p.standS / 3 / 3600.0f + rollNm(p, v1, r.k, t - rampS);
        a.kt = rollKt(p, v1, r.k, t - rampS);
        return;
    }
    a.along = r.along + rollNm(p, r.kt, r.k, t);
    a.kt = rollKt(p, r.kt, r.k, t);
}

// And climbing away after it, at `fpm` once its climb has built up.
TakeoffAt rolledOn(const TakeoffProfile &p, const Rolled &r, float fpm, uint32_t ms) {
    TakeoffAt a = {};
    if ((int32_t)(ms - r.offMs) < 0) {
        rolledAt(p, r, ms, a);
        return a;
    }
    float tau = (int32_t)(ms - r.offMs) / 1000.0f;
    a.along = r.offAlong + climbOutNm(p, r.offKt, tau);
    a.kt = climbOutKt(p, r.offKt, tau);
    a.ft = climbFt(p, fpm, tau);
    a.fpm = climbFpmAt(p, fpm, tau);
    a.climbS = tau;
    a.rate = 1;
    a.offInS = -tau;
    return a;
}

// A take-off at a moment, on runway `r`, as an estimate.
FollowEstimate takeoffOn(const RunwayLine &r, const TakeoffAt &a) {
    FollowEstimate e = onLine(r, a.along, r.fieldFt + (int32_t)lroundf(a.ft), a.ground, a.kt, (int)lroundf(a.fpm),
                              !a.ground ? "climbing out" : a.rotating ? "rotating" : "take-off roll");
    e.liftoffInS = a.offInS;
    return e;
}

// Easing between two places, heights or clocks: from `p0` going at `m0` a
// second to `p1`, still, over `lenS` seconds - `s` the share of them gone -
// a cubic Hermite's curve; how fast it goes; and the least and most of that
// along the way, and its greatest rate of change.
float hermite(float s, float p0, float m0, float p1, float lenS) {
    s = clamp01(s);
    float s2 = s * s, s3 = s2 * s;
    return (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * lenS * m0 + (3 * s2 - 2 * s3) * p1;
}
float hermiteRate(float s, float p0, float m0, float p1, float lenS) {
    s = clamp01(s);
    return 6 * s * (1 - s) * (p1 - p0) / lenS + (3 * s * s - 4 * s + 1) * m0;
}
void hermiteBounds(float p0, float m0, float p1, float lenS, float &least, float &most, float &accel) {
    float d = (p1 - p0) / lenS;
    // Its rate a s^2 + b s + c: m0 at the start, nought at the end, and
    // its turning point between.
    float a = 3 * m0 - 6 * d, b = 6 * d - 4 * m0;
    least = std::min(m0, 0.0f);
    most = std::max(m0, 0.0f);
    if (fabsf(a) > 1e-9f) {
        float s = -b / (2 * a);
        if (s > 0 && s < 1) {
            float r = (a * s + b) * s + m0;
            least = std::min(least, r);
            most = std::max(most, r);
        }
    }
    accel = std::max(fabsf(6 * d - 4 * m0), fabsf(2 * m0 - 6 * d)) / lenS;
}
// The shortest ease from `p0` at `m0` to `p1`, `lenS` no less than `fromS`,
// whose rate stays between `least` and `most` - or no worse than it starts -
// and changes by no more than `accel` a second; `toS` at most.
float easeSecs(float p0, float m0, float p1, float least, float most, float accel, float fromS, float toS) {
    least = std::min(least, m0);
    most = std::max(most, m0);
    auto fits = [&](float lenS) {
        float lo, hi, acc;
        hermiteBounds(p0, m0, p1, lenS, lo, hi, acc);
        return lo >= least - 1e-6f && hi <= most + 1e-6f && acc <= accel + 1e-6f;
    };
    if (fits(fromS)) {
        return fromS;
    }
    if (!fits(toS)) {
        return toS;
    }
    float lo = fromS, hi = toS;
    for (int i = 0; i < 18; i++) {
        float mid = (lo + hi) / 2;
        (fits(mid) ? hi : lo) = mid;
    }
    return hi;
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

// Climbing or coming down, an estimate's rate fades over about this long: see
// estimateFollowed().
constexpr float CLIMB_FADE_S = 30.0f;

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
            float done = CLIMB_FADE_S * (1 - expf(-ageS / CLIMB_FADE_S));
            e.altFt = std::max((int32_t)0, aboveSeaFt(last) + (int32_t)lroundf(last.vsFpm * done / 60.0f));
            e.vsFpm = (int)lroundf(last.vsFpm * expf(-ageS / CLIMB_FADE_S));
        }
        // With no speed to carry it on by, held at its last height too.
        return e;
    }

    RunwayLine r = lineOf(last, l);
    if (last.ground && departing) {
        // Its take-off as its kind's, from its speed - or from a standstill,
        // lined up and lost.
        const TakeoffProfile &p = profileOf(last);
        Rolled roll = rolledFrom(p, 0, l.along, gs, p.liftoffKt, r.len, gs < 1);
        e = takeoffOn(r, rolledOn(p, roll, p.climbFpm, (uint32_t)lroundf(ageS * 1000)));
    } else {
        e = landingFrom(r, l.along, gs, !last.ground, last.hasAlt, aboveSeaFt(last), ageS);
    }
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
// In the air by the feed over the runway, but slower than this share of the
// speed it lifts off at: rolling still - see start(). Or faster, and further
// down the runway than this, past where a landing touches down and slows.
constexpr float ROLLING_SHARE = 0.85f;
constexpr float ROLLING_ON_NM = 0.4f;
// Its thrust picking up: on a runway, this much faster than at its report
// before, and at least this fast - quicker than this a second, too: from 2
// to 12 kt in 35 s, lining up at Heathrow's 27R, an A330 was taken to be
// off down the runway the other way, 40m from its end. And a take-off
// begun on the ground has this much runway ahead of it at least.
constexpr int THRUST_GAIN_KT = 8;
constexpr int THRUST_MIN_KT = 12;
constexpr float THRUST_KT_S = 0.5f;
constexpr float TAKEOFF_AHEAD_NM = 0.3f;
// Further down a runway than its report before by this, it is going down it.
constexpr float DOWN_IT_NM = 0.005f;
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
constexpr int REJECTED_DROP_KT = 10;
// How far past its edges a report on the ground still counts as on the
// runway: ground positions are good to a few tens of metres.
constexpr float RUNWAY_EDGE_NM = 0.03f;
// A correction is eased in at no more than half its speed again, over
// between these - and one to its height at no more than this.
constexpr float EASE_MIN_S = 3.0f, EASE_MAX_S = 20.0f;
// Put back along its way by more than this - see plotAt() - the radar takes
// it at once. Less, eased as at() eases it: too little to see on the plot,
// and no jitter.
constexpr float PLOT_BACK_M = 5.0f;
constexpr float EASE_MIN_KT = 20.0f;
constexpr float EASE_FPM = 600.0f;
// A take-off put right is taken in no faster than this share of its speed
// again, nor its height faster than that share of its climb on top of it -
// a Cessna's 700ft a minute, an airliner's 2500 - settling over about the
// last few seconds of it: see report().
constexpr float CATCH_UP_SHARE = 0.6f;
constexpr float CATCH_UP_MIN_FPM = 300.0f;
constexpr float CATCH_UP_TAU_S = 4.0f;
// A take-off under way - see report() - is two: the real one as its
// reports have it, and the one shown, planned afresh at each report from
// where it was shown as that report came in. Of the real one:
//
// Its height on the runway, as the feed has it put right by its GNSS
// height or the day's pressure: this much over the field (-8 to +42 ft,
// tenth to ninetieth percentile, off Heathrow's 27R on 27 Oct 2026) - or
// its own, once it has some: the average of the last few of its reports
// "airborne" at a roll, level and slower than ROLLING_SHARE of the speed it
// lifts off at, within this much of the field's.
constexpr float ROLLING_FT = 17.0f, ROLLING_NEAR_FT = 120.0f;
constexpr int ROLLING_KEPT = 5;
// Off the ground only by what shows it - this much over that height; or
// climbing at CLIMBING_FPM, at nine tenths of the speed it lifts off at,
// and no lower than the feed's height dips as its nose comes up; or past
// the runway's end by this much - never by the feed's say: each of the 43
// departures recorded was "airborne" from 67-133 kt, rolling still.
constexpr float OFF_FT = 100.0f, OFF_SHARE = 0.9f, DIP_FT = -80.0f, PAST_END_NM = 0.05f;
// A rate coming down faster than this, on the roll or climbing away, is
// nonsense: Comm-B's -960 and -1856 ft a minute.
constexpr int NONSENSE_FPM = -300;
// Rolling, a speed the same as at its report before - this long before at
// least, and this fast - is the feed's old one: 139 kt twice, five seconds
// apart, from an A350 gathering speed at 4 kt a second.
constexpr float STALE_GS_S = 3.0f;
constexpr int STALE_GS_KT = 30;
// Sure it is rolling - on the ground, or level this low - it lifts off at
// this much over its speed at least, and no more than this over its kind's.
constexpr float SURE_ROLLING_FT = 30.0f, LIFTS_ABOVE_KT = 3.0f, LIFTS_MOST_OVER_KT = 25.0f;
// In the air, its seconds since it lifted off: from its climb's rate - which
// is building, not lagging: its rate says where on the build it is - if
// that is no more than this share of its climb's; from its height, this high
// or higher; the two together where they agree to within this, the rate
// counting twice; this long where neither says; and no longer than since
// its last report rolling, and this.
constexpr float RATE_SAYS_SHARE = 0.9f, HEIGHT_SAYS_FT = 60.0f, SAYS_AGREE_S = 4.0f, SAYS_NOTHING_S = 1.0f,
                SINCE_ROLLING_S = 2.0f;
// Its climb learnt from a report's rate, its build a second behind it and
// below where its climb eases, by this - the rate within these shares of
// its kind's - half and half with what it had.
constexpr float LEARN_AFTER_S = 1.0f, LEARN_BELOW_FT = 200.0f, LEARN_LEAST = 0.75f, LEARN_MOST = 1.3f;
// A report on the roll whose position was older than this, below this, is
// left out: its speed is newer than its place by none can say how much.
constexpr uint32_t OLD_ROLL_MS = 5000;
constexpr float OLD_ROLL_BELOW_FT = 300.0f;
//
// Of the one shown: rolling, its roll sped up or slowed - no less, no more
// than these shares of its kind's acceleration - to leave the runway as the
// real one is reckoned to; the real one off already, or leaving within its
// rotation and this, caught up with: its roll at the quickest, off as soon
// as it can at no less than this share of the speed the real one lifts off
// at, its nose coming up for that. Its rotation, once begun, finished as
// planned - or sooner, as the real one leaves sooner, but not before this
// share of it.
constexpr float ROLL_SLOWEST = 0.65f, ROLL_QUICKEST = 1.4f, CATCH_UP_LEAD_S = 0.5f, CATCH_UP_LIFTS = 0.92f,
                ROTATION_LEAST = 0.6f;
// Climbing away, its climb's clock played no slower than this and no faster
// than these - building and built - its climb changed by it no more than
// this many feet a minute a second, a sixth of a g, over this long at least
// and this at most; a lead given back once its climb has built.
constexpr float PACE_SLOWEST = 0.6f, PACE_QUICKEST_BUILDING = 1.25f, PACE_QUICKEST = 1.3f, PACE_CHANGE_FPM_S = 290.0f,
                PACE_MIN_S = 4.0f, PACE_MAX_S = 200.0f;
// And its place the real one's, what is between them eased out over this
// long at least - at most this - no faster than this share of its speed.
constexpr float PLACE_MIN_S = 10.0f, PLACE_MAX_S = 300.0f, PLACE_SHARE = 0.10f;
// Its roll begun where it was shown before - on the runway's line, no more
// than this behind where the real one is nor this much slower - or else
// where the real one is.
constexpr float START_BEHIND_NM = 0.05f, START_SLOWER_KT = 10.0f;
// Found in the air, a take-off: climbing away, below HANDOVER_AGL_FT, over
// the runway or HANDOVER_PAST_END_NM past it, lined up with it, flying - at
// this share of the speed it lifts off at - and never seen higher or coming
// down before.
constexpr float FOUND_FLYING_SHARE = 0.7f;
// Handed back to the feed in the air: a height above the feed's taken down
// no faster than this share of its climb - so never a descent - or,
// level, this; one below it made up no faster than this share of the
// take-off's climb - over this long at least, this at most.
constexpr float HANDBACK_DOWN_SHARE = 0.5f, HANDBACK_LEVEL_FPM = 300.0f, HANDBACK_UP_SHARE = 0.35f,
                HANDBACK_MIN_S = 3.0f, HANDBACK_MAX_S = 120.0f;
// And climbing on out - a report climbing, low - each report after it eased
// in so too, and its climb from each faded over this long rather than
// CLIMB_FADE_S: it is still climbing at the next.
constexpr float CLIMB_OUT_FADE_S = 120.0f;
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

// Lined up and stopped, its roll taken to have begun if no report says
// otherwise this long after one heard `gapMs` after the one before: see
// RunwaySim::waitFor().
uint32_t lineUpWait(uint32_t gapMs) {
    return (uint32_t)std::min(std::max(gapMs, (uint32_t)10000) * 1.5f, 45000.0f) + 3000;
}

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

// Whether any of its reports in `history` from before `ms` has it in the air.
bool airBefore(const SampleList &history, uint32_t ms) {
    for (const FollowSample &h : history) {
        if (h.ms < ms && !h.ground) {
            return true;
        }
    }
    return false;
}

// Whether any of its reports in `history` before `s` had it in the air as
// high as `s` or higher, or coming down: not just off a runway.
bool airAboveBefore(const SampleList &history, const FollowSample &s) {
    for (const FollowSample &h : history) {
        if (h.ms < s.ms && !h.ground &&
            ((h.hasAlt && aboveSeaFt(h) >= aboveSeaFt(s)) || (h.hasVs && h.vsFpm <= -CLIMBING_FPM))) {
            return true;
        }
    }
    return false;
}

float smoothstep(float u) { return u * u * (3.0f - 2.0f * u); }

// The share of a take-off's correction still to take in, `t` seconds on: at
// a steady rate, all of it in `lenS` seconds, then more and more gently over
// the last CATCH_UP_TAU_S's worth, its speed and climb settling back to its
// own rather than stopping short at the end.
float catchUpLeft(float lenS, float t) {
    if (lenS > CATCH_UP_TAU_S) {
        float steadyS = lenS - CATCH_UP_TAU_S;
        if (t <= steadyS) {
            return 1 - t / lenS;
        }
        return CATCH_UP_TAU_S / lenS * expf(-(t - steadyS) / CATCH_UP_TAU_S);
    }
    return expf(-t / CATCH_UP_TAU_S);
}

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
// begun: until the next poll would have had it again, by the gap since the
// report before - the zoom's ten seconds, or the radar's thirty - and half
// as long again, as the polls come unevenly: one 35 s after the last, the
// gap before it 25 s, found it stopped still where it was shown rolling at
// 21 kt, and put it back. Both from when the reports were heard, not their
// positions' times: one heard with its position 12 s old was taken to be
// rolling 12 s early.
uint32_t RunwaySim::waitFor(const FollowSample &prev, bool hadPrev) const {
    uint32_t gap = hadPrev ? (fresh_.ms + fresh_.ageCs * 10u) - (prev.ms + prev.ageCs * 10u) : 10000;
    return lineUpWait((int32_t)gap < 0 ? 10000 : gap);
}

void RunwaySim::report(const FollowSample &s) {
    if (haveFresh_ && (int32_t)(s.ms - fresh_.ms) < 0) {
        // Older than what it has. But stopped, lined up, its position is old
        // because it has not moved: heard later, it is waiting still - not
        // taken to have begun its roll unseen, at 45 kt by its next report,
        // which had it stopped there and put it back 110m.
        if (phase_ == LINED_UP && s.ground && s.hasGs && s.gsKt < STOPPED_KT && s.hasPos && !s.posStale) {
            uint32_t heard = s.ms + s.ageCs * 10u, gap = heard - (fresh_.ms + fresh_.ageCs * 10u);
            if ((int32_t)gap > 0) {
                rollMs_ = laterOf(rollMs_, heard + lineUpWait(gap));
            }
        }
        return;
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
    // When it was heard - its position's time, and how old that was then. A
    // correction is eased in from where it was shown then: from where it
    // would have been shown at its position's time, which the clock had
    // passed, it jumped by all it had made up since - 200m down a runway, a
    // ground report half a minute old.
    uint32_t heardMs = s.ms + s.ageCs * 10u;
    bool wasUnderWay = hadPrev && phase_ != NONE;
    bool wasSim = hadPrev && simulating(heardMs);
    // A take-off under way, or its climb-out still being caught up with.
    bool wasTakeoff = hadPrev && (phase_ == LINED_UP || phase_ == TAKEOFF ||
                                  (climbEase_ && (int32_t)(heardMs - easeEndMs_) < 0));
    FollowEstimate shown = {};
    if (hadPrev) {
        shown = at(heardMs);
    }
    // A take-off as it was shown, exactly - how fast it rolled, its climb's
    // clock - for the one shown to be planned on from: see takeoffReport().
    Phase before = hadPrev ? phase_ : NONE;
    TakeoffAt was = {};
    if (before == TAKEOFF) {
        was = shownAt(heardMs);
    } else if (before == LINED_UP) {
        was = linedUpAt(heardMs);
        float offset;
        relate(rw_, shown.lat, shown.lon, was.along, offset);
    }
    // Its height being handed back to the feed after a take-off, or climbing
    // on out after one.
    bool climbSafeWas = hadPrev && climbEase_ && climbSafe_ && (int32_t)(heardMs - easeEndMs_) < 0;
    bool climbingOutWas = hadPrev && climbingOut_;
    // And its speed, eased with its place after a take-off ended on the runway.
    bool speedWas = hadPrev && dKt_ != 0 && (int32_t)(heardMs - easeEndMs_) < 0;
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
    easeStartMs_ = easeEndMs_ = altEndMs_ = heardMs;
    turning_ = false;
    climbEase_ = false;
    climbSafe_ = false;
    climbingOut_ = false;
    dKt_ = 0;
    rollOverNm_ = 0;
    plotCut_ = false;
    if (phase_ == TAKEOFF) {
        takeoffReport(heardMs, before, before == TAKEOFF || before == LINED_UP ? &was : nullptr,
                      hadPrev ? &shown : nullptr);
        return;
    }
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
    if (!hadPrev || (!wasSim && !climbingOutWas && phase_ == NONE && !taxiingAlong(s, history_, alongTrack))) {
        return;  // the feed's, as it always was
    }
    // From where it was shown, to where this report puts it.
    FollowEstimate now = model(heardMs, 1e9f);
    // A take-off handed back to the feed in the air, or climbing on out
    // after one - see below.
    bool handBack = (before == TAKEOFF || before == LINED_UP || climbSafeWas || climbingOutWas) && phase_ == NONE &&
                    !now.ground && !shown.ground;
    climbingOut_ = handBack && lowReport(s) && s.hasVs && s.vsFpm >= CLIMBING_FPM && s.hasGs && s.gsKt > 0 &&
                   s.hasTrack;
    if (climbingOut_) {
        climbOutOf(heardMs, 1e9f, now);
    }
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
        if (wasTakeoff || phase_ == LINED_UP || handBack) {
            // A take-off found further on than it was shown - lined up and
            // lost on its roll, or picked up from a report some seconds old -
            // or behind: taken in as though it were shown that much earlier
            // in its take-off, its place and its height together, so it goes
            // on along its way, lifting off on the runway and climbing a
            // little more steeply than it is, a little faster - by no more
            // than CATCH_UP_SHARE of its speed and of its climb - then
            // settling. Taken in by the clock in no more than twenty
            // seconds, one picked up as it climbed away ran off the runway's
            // end and climbed like a rocket.
            float mostKt = std::max(EASE_MIN_KT, CATCH_UP_SHARE * now.gsKt);
            float t = now.track * (float)M_PI / 180.0f;
            plotCut_ = (dLat_ * 60.0f * cosf(t) + dLon_ * k * sinf(t)) * 1852.0f > PLOT_BACK_M;
            climbEase_ = true;
            if (handBack) {
                // A take-off handed back to the feed in the air - up and
                // away, or turning off its runway's line: its place as
                // above, but its height eased out keeping its climb's rate
                // as it was shown, a height above the feed's taken down no
                // faster than half the feed's climb - so it never comes
                // down - and one below made up no faster than a third of
                // its own on top: the offset shrinking faster than it
                // climbed, the catch-up sank 1,300ft a minute (QFA10). And
                // so at each report while it climbs on out, low, its climb
                // from each carried on to the next rather than faded as
                // for one lost: faded, it was shown climbing at half its
                // rate between reports and catching up at each at up to
                // 50,000ft a minute.
                climbLenS_ = d / mostKt * 3600.0f;
                climbSafe_ = true;
                altRate_ = (shown.vsFpm - now.vsFpm) / 60.0f;
                float up = HANDBACK_UP_SHARE * std::max(tkClimbFpm_, (float)CATCH_UP_MIN_FPM) / 60.0f;
                altLenS_ = HANDBACK_MIN_S;
                for (int pass = 0; pass < 4; pass++) {
                    float fading = now.vsFpm * expf(-altLenS_ / (climbingOut_ ? CLIMB_OUT_FADE_S : CLIMB_FADE_S));
                    float down = fading > 0 ? std::max(HANDBACK_DOWN_SHARE * fading, HANDBACK_LEVEL_FPM - fading)
                                            : HANDBACK_LEVEL_FPM;
                    altLenS_ = std::max(altLenS_, easeSecs(dAlt_, altRate_, 0, -down / 60.0f, up,
                                                           PACE_CHANGE_FPM_S / 60.0f, HANDBACK_MIN_S, HANDBACK_MAX_S));
                }
                float spanS = std::max(std::max(climbLenS_ - CATCH_UP_TAU_S, 0.0f) + 4 * CATCH_UP_TAU_S, altLenS_);
                altEndMs_ = easeEndMs_ = heardMs + (uint32_t)(spanS * 1000.0f);
                return;
            }
            float climbFpm = std::max(profileOf(s).climbFpm, fabsf((float)now.vsFpm));
            float mostFpm = std::max(CATCH_UP_MIN_FPM, CATCH_UP_SHARE * climbFpm);
            climbLenS_ = std::max(d / mostKt * 3600.0f, fabsf(dAlt_) / mostFpm * 60.0f);
            // Rejected on its roll, its speed eased down with its place, not
            // dropped from the roll's 114 kt to the report's 82 at once.
            if ((before == TAKEOFF && now.ground && shown.ground) || speedWas) {
                dKt_ = (float)(shown.gsKt - now.gsKt);
            }
            float spanS = std::max(climbLenS_ - CATCH_UP_TAU_S, 0.0f) + 4 * CATCH_UP_TAU_S;
            altEndMs_ = easeEndMs_ = heardMs + (uint32_t)(spanS * 1000.0f);
            return;
        }
        float halfKt = std::max((float)now.gsKt, EASE_MIN_KT) / 2.0f;
        easeS = std::min(std::max(d / halfKt * 3600.0f, EASE_MIN_S), EASE_MAX_S);
        // A rejected take-off's speed still being eased down: on with its
        // place - into the landing's roll-out, say, that it is taken for.
        if (speedWas) {
            dKt_ = (float)(shown.gsKt - now.gsKt);
        }
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
            FollowEstimate stop = model(heardMs + ROLL_EASE_MS, 1e9f);
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
        altEndMs_ = heardMs + (uint32_t)(altS * 1000.0f);
        easeS = rollOverNm_ > 0 ? ROLL_EASE_MS / 1000.0f : std::max(easeS, altS);
        // Eased back along its way by the clock - not slowed for as it rolls
        // on, nor turned to face the report - which on the radar is a blip
        // sliding backwards: there, taken at once. See plotAt().
        if (rollOverNm_ == 0) {
            float t = now.track * (float)M_PI / 180.0f;
            plotCut_ = (dLat_ * 60.0f * cosf(t) + dLon_ * k * sinf(t)) * 1852.0f > PLOT_BACK_M;
        }
    }
    easeEndMs_ = heardMs + (uint32_t)(easeS * 1000.0f);
}

// fresh_ just in, a take-off under way: begun by it, or `before` it. `was`
// is the one shown as it came in - its roll, or its climb's clock - and
// `shown` where it was shown then, before a take-off began.
//
// Two take-offs are kept, each a closed form in time (takeoff.h): the real
// one as its reports have it, rebuilt from each; and the one shown, planned
// afresh at each from where it was shown as the report came in, so it never
// jumps. On the runway its own roll, sped up or slowed to leave the runway as
// the real one is reckoned to - its speed only ever gathering, the change a
// little more or less acceleration - its nose coming up over its rotation's
// last seconds; off, never on the runway again, its climb built up as its
// kind's is, its climb's clock played a little faster or slower to meet the
// real one's - never so slowly that it stops climbing - and its place the
// real one's with what was between them eased out. It hung at a report's rate
// just off the runway (BAW131 at Heathrow) - the rate lagging, it was taken,
// but it is the climb building - was held down while its nose came up and let
// go 200ft below the real one to catch up at 10,000ft a minute (BAW594),
// lifted off twice, put back on the runway by a report at 147 kt that was
// "airborne" but rolling still (BAW2PD), and dropped its nose on the runway
// for a speed the feed had repeated (BAW143).
void RunwaySim::takeoffReport(uint32_t heardMs, Phase before, const TakeoffAt *was, const FollowEstimate *shown) {
    const FollowSample &s = fresh_;
    const TakeoffProfile &p = profileOf(s);
    float along, offset;
    relate(rw_, s.lat, s.lon, along, offset);
    if (before != TAKEOFF) {
        // A new take-off: its kind's, until its reports say otherwise.
        tkLiftoffKt_ = p.liftoffKt;
        tkClimbFpm_ = p.climbFpm;
        tkLevels_ = 0;
        tkRolled_ = false;
        planAir_ = false;
    } else if (s.ageCs * 10u > OLD_ROLL_MS && s.hasGs && s.gsKt >= TAKEOFF_ROLLING_KT && !s.ground &&
               (!s.hasAlt || aboveSeaFt(s) - rw_.fieldFt < OLD_ROLL_BELOW_FT)) {
        return;  // an old position on the roll, its speed newer: see OLD_ROLL_MS
    }
    takeoffEstimate(along);
    if (before == TAKEOFF) {
        takeoffPlan(heardMs, was, true);
        return;
    }
    // Begun: from where it was shown - lined up, or rolling down the
    // runway's line - if that is no further behind the real one than
    // START_BEHIND_NM nor slower by START_SLOWER_KT, or climbing away
    // already; or else from where the real one is. Held lined up while the
    // real one was off at 40 kt 116m on, it was begun from a standstill
    // there and left the runway 600m short of it.
    TakeoffAt real = realAt(heardMs), from = {};
    bool fromShown = false;
    if (was != nullptr) {
        from = *was;
        fromShown = true;
    } else if (shown != nullptr && shown->ground) {
        float off;
        relate(rw_, shown->lat, shown->lon, from.along, off);
        from.ground = true;
        from.kt = (float)shown->gsKt;
        from.rate = 1;
        fromShown = off <= rw_.halfWidthNm + RUNWAY_EDGE_NM;
    }
    if (fromShown && (!from.ground || (real.ground && real.along - from.along <= START_BEHIND_NM &&
                                       real.kt - from.kt <= START_SLOWER_KT))) {
        takeoffPlan(heardMs, &from, false);
        return;
    }
    takeoffPlan(heardMs, nullptr, false);
}

// The real take-off, as fresh_ - `along` the runway - has it.
void RunwaySim::takeoffEstimate(float along) {
    const FollowSample &s = fresh_;
    const TakeoffProfile &p = profileOf(s);
    // Its height over the runway: over its own height read rolling, or - its
    // height put right by its GNSS height or the day's pressure - over the
    // field's. With neither, the feed's pressure altitude read 117ft high on
    // the roll: no say in whether it is off.
    bool level = tkLevels_ > 0 || s.pressureFt != 0;
    bool hasFt = !s.ground && s.hasAlt && level;
    float ft = hasFt ? aboveSeaFt(s) - (tkLevels_ > 0 ? tkLevelFt_ : rw_.fieldFt + ROLLING_FT) : 0.0f;
    bool hasVs = !s.ground && s.hasVs && s.vsFpm > NONSENSE_FPM;
    float vs = hasVs ? (float)s.vsFpm : 0.0f, gs = s.hasGs ? (float)s.gsKt : 0.0f;
    bool air = !s.ground && ((hasFt && ft >= OFF_FT) ||
                             (hasVs && vs >= CLIMBING_FPM && s.hasGs && gs >= OFF_SHARE * tkLiftoffKt_ &&
                              (!hasFt || ft >= DIP_FT)) ||
                             along > rw_.len + PAST_END_NM);
    if (!air) {
        // Rolling: at its speed - never slower than at its report rolling
        // before - its roll on to its lift-off as its kind's.
        float v = s.hasGs ? gs : tkRolled_ ? tkRollKt_ : 0.0f;
        if (tkRolled_) {
            float since = (int32_t)(s.ms - tkRollMs_) / 1000.0f;
            if (s.hasGs && s.gsKt == tkRollGs_ && since >= STALE_GS_S && s.gsKt >= STALE_GS_KT && !est_.air) {
                v = realAt(s.ms).kt;  // the feed's old speed again: as it was reckoned to be going
            }
            v = std::max(v, tkRollKt_);
        }
        // Sure it is rolling still, it leaves faster than that: an
        // "airborne" 777 at 147 kt, level, was not off (BAW2PD).
        if (s.ground || ((!hasFt || ft < SURE_ROLLING_FT) && (!hasVs || vs < CLIMBING_FPM))) {
            tkLiftoffKt_ = std::min(std::max(tkLiftoffKt_, v + LIFTS_ABOVE_KT), p.liftoffKt + LIFTS_MOST_OVER_KT);
        }
        if (!s.ground && s.hasAlt && (!hasVs || vs < CLIMBING_FPM) && s.hasGs && gs < ROLLING_SHARE * tkLiftoffKt_ &&
            (s.pressureFt == 0 || fabsf(aboveSeaFt(s) - (rw_.fieldFt + ROLLING_FT)) < ROLLING_NEAR_FT)) {
            tkLevels_ = (uint8_t)std::min(tkLevels_ + 1, ROLLING_KEPT);
            tkLevelFt_ += (aboveSeaFt(s) - tkLevelFt_) / tkLevels_;
        }
        tkRolled_ = true;
        tkRollMs_ = s.ms;
        tkRollAlong_ = along;
        tkRollKt_ = v;
        tkRollGs_ = s.hasGs ? s.gsKt : (int16_t)-1;
        est_ = rolledFrom(p, s.ms, along, v, tkLiftoffKt_, rw_.len, false);
        return;
    }
    // Off: its lift-off reckoned back by how far its climb has come - by its
    // rate, while that is building, and by its height.
    float fpm = tkClimbFpm_;
    float byRate = hasVs && vs >= CLIMBING_FPM && vs <= RATE_SAYS_SHARE * fpm ? vs * p.buildS / fpm : NAN;
    float atFt = hasFt ? ft : !s.ground && s.hasAlt ? aboveSeaFt(s) - rw_.fieldFt : NAN;
    float byHeight = atFt >= HEIGHT_SAYS_FT ? climbSecsAt(p, fpm, atFt) : NAN;
    float tau = !std::isnan(byRate) && !std::isnan(byHeight)
                    ? (fabsf(byRate - byHeight) < SAYS_AGREE_S ? (2 * byRate + byHeight) / 3 : byHeight)
                : !std::isnan(byHeight) ? byHeight
                : !std::isnan(byRate)   ? byRate
                                        : SAYS_NOTHING_S;
    if (tkRolled_) {
        tau = std::min(tau, (int32_t)(s.ms - tkRollMs_) / 1000.0f + SINCE_ROLLING_S);
    }
    tau = std::max(tau, 0.0f);
    float liftKt = s.hasGs ? gs - p.gainKt * (1 - expf(-tau / GAIN_S)) : tkLiftoffKt_;
    float offAlong = along - climbOutNm(p, liftKt, tau);
    if (tkRolled_) {
        offAlong = std::max(offAlong, tkRollAlong_);
    }
    est_ = {};
    est_.air = true;
    est_.k = 1;
    est_.ms = est_.rotMs = est_.offMs = s.ms - (uint32_t)lroundf(tau * 1000);
    est_.along = est_.rotAlong = est_.offAlong = std::min(std::max(offAlong, 0.0f), rw_.len - STOP_SHORT_NM);
    est_.kt = est_.rotKt = est_.offKt = liftKt;
    // Its climb, from its rate once that is steady - the build behind it.
    if (tau >= p.buildS + LEARN_AFTER_S && hasFt && ft < p.easeFt - LEARN_BELOW_FT && hasVs &&
        vs >= LEARN_LEAST * p.climbFpm && vs <= LEARN_MOST * p.climbFpm) {
        tkClimbFpm_ = (tkClimbFpm_ + vs) / 2;
    }
    tkLiftoffKt_ = liftKt;
}

// The one shown, planned on from `from` - where it was shown at `t0` - or,
// with none, from where the real one is; `replanning` a plan under way.
void RunwaySim::takeoffPlan(uint32_t t0, const TakeoffAt *from, bool replanning) {
    const TakeoffProfile &p = profileOf(fresh_);
    TakeoffAt st;
    float wasFpm = replanning ? planFpm_ : p.climbFpm;
    if (from == nullptr) {
        st = realAt(t0);
        if (!st.ground) {
            // Found off already: shown where it is, climbing as it is.
            plan_ = est_;
            planAir_ = true;
            takeoffClimb(t0, st.climbS, 1, tkClimbFpm_, st.along, st.kt, t0);
            return;
        }
        st.rotating = false;
    } else {
        st = *from;
    }
    if (!st.ground) {
        // Off: never on the runway again. Its climb's clock played on from
        // where it is - its climb learnt afresh, taken up at the height and
        // rate it is shown at.
        float tau0 = st.climbS, rate0 = st.rate;
        if (fabsf(tkClimbFpm_ - wasFpm) > 1) {
            tau0 = st.ft > 0 ? climbSecsAt(p, tkClimbFpm_, st.ft) : tau0;
            float fpm = climbFpmAt(p, tkClimbFpm_, tau0);
            if (fpm > CLIMBING_FPM) {
                rate0 = std::min(std::max(st.fpm / fpm, 0.3f), 2.0f);
            }
        }
        planAir_ = true;
        takeoffClimb(t0, tau0, rate0, tkClimbFpm_, st.along, st.kt, t0);
        return;
    }
    if (st.rotating && replanning && !planAir_) {
        // Its nose coming up: its rotation finished as planned - or sooner,
        // the real one leaving sooner, but not before most of it is done.
        uint32_t earliest = plan_.rotMs + (uint32_t)lroundf(ROTATION_LEAST * p.rotateS * 1000);
        if ((int32_t)(est_.offMs - plan_.offMs) < 0) {
            uint32_t off = laterOf(laterOf(earliest, est_.offMs), t0);
            if ((int32_t)(off - plan_.offMs) < 0) {
                float x = (int32_t)(off - plan_.rotMs) / 1000.0f;
                plan_.offMs = off;
                plan_.offAlong = plan_.rotAlong + (plan_.rotKt * x + plan_.rotAccel * x * x / 2) / 3600.0f;
                plan_.offKt = plan_.rotKt + plan_.rotAccel * x;
            }
        }
        takeoffClimb(plan_.offMs, 0, 1, tkClimbFpm_, plan_.offAlong, plan_.offKt, plan_.offMs);
        return;
    }
    // Rolling, from where it was shown: its own roll sped up or slowed to
    // leave the runway when the real one is reckoned to - or, that one off
    // or all but, caught up with - and by the runway's end.
    float liftKt = tkLiftoffKt_, k = 1;
    float toOffS = (int32_t)(est_.offMs - t0) / 1000.0f;
    if (est_.air || toOffS < p.rotateS + CATCH_UP_LEAD_S) {
        k = ROLL_QUICKEST;
        float reach = st.kt + rotateAccelOf(p, st.kt, k) * p.rotateS;
        liftKt = std::max(CATCH_UP_LIFTS * tkLiftoffKt_, std::min(tkLiftoffKt_, reach));
    } else {
        float rollS = std::max(toOffS - p.rotateS, 0.5f);
        for (int pass = 0; pass < 3; pass++) {
            float vrot = rotateKtOf(p, liftKt, k);
            if (vrot <= st.kt) {
                break;
            }
            k = std::min(std::max(rollShare(p, st.kt, vrot, rollS), ROLL_SLOWEST), ROLL_QUICKEST);
        }
    }
    plan_ = rolledFrom(p, t0, st.along, st.kt, liftKt, rw_.len, false, k, true);
    planAir_ = false;
    takeoffClimb(plan_.offMs, 0, 1, tkClimbFpm_, plan_.offAlong, plan_.offKt, plan_.offMs);
}

// The one shown climbing away from `ta`, its climb's clock at `tau0` going
// at `rate0` its own pace and played on to meet the real one's - no faster
// than PACE_QUICKEST, a lead given back once its climb has built, at no
// slower than PACE_SLOWEST - climbing at `fpm` once built; its place,
// `along` at `kt` at `tR`, the real one's, what is between eased out.
void RunwaySim::takeoffClimb(uint32_t ta, float tau0, float rate0, float fpm, float along, float kt, uint32_t tR) {
    const TakeoffProfile &p = profileOf(fresh_);
    climbMs_ = ta;
    climbS0_ = tau0;
    rate0_ = rate0;
    planFpm_ = fpm;
    climbGapS_ = (int32_t)(ta - est_.offMs) / 1000.0f - tau0;
    holdS_ = climbGapS_ < 0 ? std::max(0.0f, p.buildS - tau0) : 0.0f;
    float quickest = tau0 < p.buildS ? PACE_QUICKEST_BUILDING : PACE_QUICKEST;
    warpS_ = easeSecs((rate0 - 1) * holdS_, rate0 - 1, climbGapS_, PACE_SLOWEST - 1, quickest - 1,
                      PACE_CHANGE_FPM_S / std::max(fpm, 500.0f), PACE_MIN_S, PACE_MAX_S);
    float realAlong, realKt;
    flightAt(tR, realAlong, realKt);
    restMs_ = tR;
    rest0Nm_ = along - realAlong;
    rest0NmS_ = (kt - realKt) / 3600.0f;
    float most = std::max(PLACE_SHARE * std::max(realKt, 60.0f) / 3600.0f, fabsf(rest0NmS_));
    restS_ = easeSecs(rest0Nm_, rest0NmS_, 0, -most, most, 1e9f, PLACE_MIN_S, PLACE_MAX_S);
}

// Climbing on out after a take-off, estimate `e` - the feed's, from fresh_ -
// with its climb faded over CLIMB_OUT_FADE_S rather than CLIMB_FADE_S.
void RunwaySim::climbOutOf(uint32_t ms, float maxAgeS, FollowEstimate &e) const {
    float t = (int32_t)(ms - fresh_.ms) > 0 ? std::min((ms - fresh_.ms) / 1000.0f, maxAgeS) : 0.0f, vs = fresh_.vsFpm;
    float more = CLIMB_OUT_FADE_S * (1 - expf(-t / CLIMB_OUT_FADE_S)) - CLIMB_FADE_S * (1 - expf(-t / CLIMB_FADE_S));
    e.altFt += (int32_t)lroundf(vs * more / 60.0f);
    e.vsFpm += (int)lroundf(vs * (expf(-t / CLIMB_OUT_FADE_S) - expf(-t / CLIMB_FADE_S)));
}

RunwaySim::TakeoffAt RunwaySim::realAt(uint32_t ms) const {
    return rolledOn(profileOf(fresh_), est_, tkClimbFpm_, ms);
}

// Where the real one is reckoned to be flying at `ms`, along the runway's
// line, and how fast - for the one shown in the air to keep to: away, as
// realAt(); before its lift-off, where it would have been flying at its
// lift-off speed - shown off before the real one, as it can be, it is
// slowed no more than a little, not taken back down to the roll's speed
// (35 kt, for one found taxiing on as it was shown climbing away).
void RunwaySim::flightAt(uint32_t ms, float &along, float &kt) const {
    float tau = (int32_t)(ms - est_.offMs) / 1000.0f;
    const TakeoffProfile &p = profileOf(fresh_);
    along = est_.offAlong + climbOutNm(p, est_.offKt, tau);
    kt = climbOutKt(p, est_.offKt, tau);
}

RunwaySim::TakeoffAt RunwaySim::shownAt(uint32_t ms) const {
    const TakeoffProfile &p = profileOf(fresh_);
    TakeoffAt a = {};
    if (!planAir_ && (int32_t)(ms - plan_.offMs) < 0) {
        rolledAt(p, plan_, ms, a);
        return a;
    }
    // Its climb's clock: held at its pace while its climb builds, if it is
    // ahead, then eased to meet the real one's.
    float x = (int32_t)(ms - climbMs_) / 1000.0f, d, m;
    if (x <= holdS_) {
        d = (rate0_ - 1) * x;
        m = rate0_ - 1;
    } else {
        float d0 = (rate0_ - 1) * holdS_, u = (x - holdS_) / warpS_;
        d = hermite(u, d0, rate0_ - 1, climbGapS_, warpS_);
        m = hermiteRate(u, d0, rate0_ - 1, climbGapS_, warpS_);
    }
    a.climbS = climbS0_ + x + d;
    a.rate = 1 + m;
    a.ft = climbFt(p, planFpm_, a.climbS);
    a.fpm = climbFpmAt(p, planFpm_, a.climbS) * a.rate;
    // Its place: the real one's, and what was between them eased out.
    float realAlong, realKt;
    flightAt(ms, realAlong, realKt);
    float u = (int32_t)(ms - restMs_) / 1000.0f / restS_;
    a.along = realAlong + hermite(u, rest0Nm_, rest0NmS_, 0, restS_);
    a.kt = realKt + hermiteRate(u, rest0Nm_, rest0NmS_, 0, restS_) * 3600.0f;
    a.ground = a.rotating = false;
    a.offInS = (int32_t)(plan_.offMs - ms) / 1000.0f;
    return a;
}

RunwaySim::TakeoffAt RunwaySim::linedUpAt(uint32_t ms) const {
    const TakeoffProfile &p = profileOf(fresh_);
    float along, offset;
    relate(rw_, fresh_.lat, fresh_.lon, along, offset);
    return rolledOn(p, rolledFrom(p, rollMs_, along, 0, p.liftoffKt, rw_.len, true), p.climbFpm, ms);
}

FollowEstimate RunwaySim::takeoffEstimateOf(const TakeoffAt &a) const {
    FollowEstimate e = takeoffOn(rw_, a);
    e.pressureFt = fresh_.pressureFt;
    return e;
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
    // On the ground, going down the runway - further along it than its
    // report before - it is along it, whatever its track: the feed's on
    // the ground can be the taxiway's still, 327 degrees for a 787
    // rolling down Heathrow's 27R at 67 kt, which ended its take-off.
    bool downIt = false;
    if ((phase_ == LINED_UP || phase_ == TAKEOFF) && s.ground && hadPrev && prev.hasPos) {
        float prevAlong, prevOffset;
        relate(rw_, prev.lat, prev.lon, prevAlong, prevOffset);
        downIt = along > prevAlong + DOWN_IT_NM;
    }
    switch (phase_) {
    case LINED_UP:
        if (s.ground && onPavement && (gs < STOPPED_KT || off <= LINED_UP_HEADING_DEG || downIt)) {
            if (gs < STOPPED_KT) {
                rollMs_ = s.ms + s.ageCs * 10u + waitFor(prev, hadPrev);  // still waiting
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
            if (onPavement && (off <= OFF_LINE_DEG || downIt) && !rejected) {
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
        // Climbing away low from a runway it is lined up with - over it, or a
        // few miles past its far end - flying, and never seen as high before
        // nor coming down: a take-off, found in the air - the feed picking it
        // up only as it climbed into view, or the zoom late to it - shown
        // climbing on from where it is as one does, not held at its report's
        // rate. Not one just handed back from its take-off, climbing on out.
        if (!climbingOut_ && !climbSafe_ && s.hasGs && s.hasAlt && !(s.hasVs && s.vsFpm <= -CLIMBING_FPM) &&
            !airAboveBefore(history_, s) && findDeparture(s, HANDOVER_PAST_END_NM, l)) {
            float aglFt = (float)(aboveSeaFt(s) - l.fieldFt);
            if (((s.hasVs && s.vsFpm >= CLIMBING_FPM) || aglFt >= OFF_FT) && aglFt < HANDOVER_AGL_FT &&
                gs >= FOUND_FLYING_SHARE * profileOf(s).liftoffKt) {
                phase_ = TAKEOFF;
                rw_ = lineOf(s, l);
                return;
            }
        }
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
        // Over the runway itself, level, slower than it could fly and never
        // seen in the air before: its take-off, picked up rolling - the feed
        // having it in the air from part way down, as all thirteen departures
        // from Heathrow's 27R on 27 Oct 2026 did, from 70 to 110 knots - not
        // a landing, shown rolling out and slowing as it climbed away.
        // So too faster than that, up to the speed it might leave at, not
        // coming down and far enough down the runway that a landing would
        // have touched down and slowed by then: first heard at 130 kt half
        // way down its roll, one was taken for a landing.
        const TakeoffProfile &p = profileOf(s);
        bool rollingOn = gs >= ROLLING_SHARE * p.liftoffKt && gs <= p.liftoffKt + LIFTS_MOST_OVER_KT && s.hasVs &&
                         s.vsFpm >= 0 && s.vsFpm < CLIMBING_FPM;
        if (gs >= TAKEOFF_ROLLING_KT && (gs < ROLLING_SHARE * p.liftoffKt || rollingOn) &&
            !(s.hasVs && s.vsFpm <= -CLIMBING_FPM) && !airBefore(history_, s.ms) && findLineup(s, l) &&
            l.along >= (gs < ROLLING_SHARE * p.liftoffKt ? 0.0f : ROLLING_ON_NM) && l.along <= l.len) {
            phase_ = TAKEOFF;
            rw_ = lineOf(s, l);
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
    bool thrust = hadPrev && prev.ground && prev.hasGs && gs >= THRUST_MIN_KT && gs >= prev.gsKt + THRUST_GAIN_KT &&
                  (gs - prev.gsKt) * 1000.0f >= THRUST_KT_S * (float)(s.ms - prev.ms);
    if (gs >= TAKEOFF_ROLLING_KT || thrust) {
        bool found = (gs >= TAKEOFF_ROLLING_KT && findLineup(s, l)) || runwayUnder(s, history_, l);
        if (!found || l.len - l.along < TAKEOFF_AHEAD_NM) {
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
        rollMs_ = s.ms + s.ageCs * 10u + waitFor(prev, hadPrev);
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
    uint32_t when = s.ms + (uint32_t)(t * 1000.0f);
    switch (phase_) {
    case LINED_UP:
        // Its roll taken to have begun at rollMs_, from a standstill.
        e = (int32_t)(when - rollMs_) < 0 ? onLine(rw_, along, rw_.fieldFt, true, 0, 0, "held: lined up")
                                          : takeoffOn(rw_, linedUpAt(when));
        break;
    case TAKEOFF:
        // The real one as its reports have it: at() shows the one planned
        // to meet it - see takeoffReport().
        e = takeoffOn(rw_, realAt(when));
        break;
    default:
        e = landingFrom(rw_, along, gs, !s.ground && !heldDown_, s.hasAlt, aboveSeaFt(s), t);
        break;
    }
    e.pressureFt = s.pressureFt;
    return e;
}

// Where the radar, the zoom and their replays put it: at()'s, but a report
// behind where it was shown - the simulation having taken it on further
// than it went, a landing that braked harder stopped short of it, a
// position some seconds old - that at() eases back by the clock is taken
// at once: the blip put where the report has it. Eased back so for the 3D
// view, where the taxiway follower takes it on the ground, a blip slid back
// along its way for seconds, up to 450m, while showing it stopped: on a
// plot, going backwards. at()'s easing as it rolls on - slowed, never
// backwards - and its turn to face a report behind are the radar's too.
FollowEstimate RunwaySim::plotAt(uint32_t ms, float maxAgeS) const {
    int32_t span = (int32_t)(easeEndMs_ - easeStartMs_), since = (int32_t)(ms - easeStartMs_);
    if (plotCut_ && phase_ != TAKEOFF && span > 0 && since < span) {
        FollowEstimate e = model(ms, maxAgeS);
        if (climbingOut_ && phase_ == NONE) {
            climbOutOf(ms, maxAgeS, e);
        }
        return e;
    }
    return at(ms, maxAgeS);  // a take-off shown never goes back: see takeoffReport()
}

FollowEstimate RunwaySim::at(uint32_t ms, float maxAgeS) const {
    if (phase_ == TAKEOFF) {
        // The take-off planned at the last report: its own, never jumping -
        // see takeoffReport().
        if ((int32_t)(ms - fresh_.ms) > 0 && (ms - fresh_.ms) / 1000.0f > maxAgeS) {
            ms = fresh_.ms + (uint32_t)(maxAgeS * 1000.0f);
        }
        return takeoffEstimateOf(shownAt(ms));
    }
    FollowEstimate e = model(ms, maxAgeS);
    if (climbingOut_ && phase_ == NONE) {
        climbOutOf(ms, maxAgeS, e);
    }
    int32_t span = (int32_t)(easeEndMs_ - easeStartMs_), since = (int32_t)(ms - easeStartMs_);
    if (span <= 0 || since >= span) {
        return e;
    }
    if (climbEase_) {
        float t = since <= 0 ? 0.0f : since / 1000.0f, left = catchUpLeft(climbLenS_, t);
        e.lat += dLat_ * left;
        e.lon += dLon_ * left;
        if (climbSafe_) {
            // A take-off's height handed back: see report().
            float u = t / altLenS_;
            e.altFt += (int32_t)lroundf(hermite(u, dAlt_, altRate_, 0, altLenS_));
            e.vsFpm += (int)lroundf(hermiteRate(u, dAlt_, altRate_, 0, altLenS_) * 60.0f);
        } else {
            e.altFt += (int32_t)(dAlt_ * left);
        }
        if (dKt_ != 0) {
            e.gsKt = std::max(0, e.gsKt + (int)lroundf(dKt_ * left));
        }
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
        if (dKt_ != 0) {
            e.gsKt = std::max(0, e.gsKt + (int)lroundf(dKt_ * (1.0f - u)));
        }
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
