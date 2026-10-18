#pragma once

#include <string.h>

#include <algorithm>

#include "adsb_client.h"
#include "airports.h"
#include "follow.h"

// What the 3D view shows. It shows no taxiing - an aircraft's reports on the
// ground are too few and far between to show it truly - only landings and
// take-offs, and aircraft stopped on their stands. So the one followed, on
// the ground off the runways, isn't what it shows. Seen landing in the view,
// following moves on - once it has slowed on the runway - to the airport's
// next runway movement: a take-off, or a landing coming in; or, none due,
// the aircraft in the air nearest. Departing - or found on the ground - the
// view shows those meanwhile, its guest, the one followed left followed,
// until that is lined up on its runway; one just landed, though, it shows
// parked once it is on its stand.
//
// Header only, over any table of tracks - each with its RunwaySim `sim`,
// its last report `last` and when that came `freshMs` - for main's and a PC
// test's alike.
namespace subject3d {

constexpr uint32_t FRESH_MS = 45000;  // a track this recent is current
constexpr float DUE_S = 240;          // a landing touching down this soon is due
constexpr int ROLLED_OUT_KT = 45;     // slowed on the runway, landed: its landing over for the view
constexpr int ROLLOUT_SHOWN_KT = 80;  // on its rollout this fast, a landing still worth showing
constexpr uint32_t LOOK_MS = 15000;   // a guest in the air: a runway movement looked for this often
constexpr uint32_t LANDED_WITHIN_MS = 15 * 60 * 1000;
constexpr uint32_t PARKED_AFTER_MS = 30000;  // landed, stopped this long: parked, shown
constexpr int STOPPED_KT = 6;

enum class Show : uint8_t {
    FOLLOWED,  // the one followed
    GUEST,     // `guest`, while the one followed is on the ground
    NEXT,      // following moves on to the hex given
};

// How soon `t`'s runway movement at the airport `code` is, seconds: a
// take-off at once; a landing when it touches down, at its speed - or, on
// its rollout, while it is still fast. -1 for none there.
template <typename Track>
float movementDue(const Track &t, const char *code, uint32_t now) {
    if (now - t.freshMs > FRESH_MS || !t.sim.haveFresh()) {
        return -1;
    }
    RunwaySim::Movement m = t.sim.movement();
    const RunwayLine &rw = t.sim.runway();
    if (m == RunwaySim::Movement::NONE || rw.airport == nullptr || strcmp(rw.airport, code) != 0) {
        return -1;
    }
    FollowEstimate e = t.sim.at(now);
    if (m == RunwaySim::Movement::TAKEOFF) {
        return 0;
    }
    if (e.ground) {
        return e.gsKt >= ROLLOUT_SHOWN_KT ? 30 : -1;
    }
    return haversineNm(e.lat, e.lon, rw.thrLat, rw.thrLon) / std::max(e.gsKt, 100) * 3600.0f;
}

// The next runway movement at `code` - the soonest due within DUE_S - or,
// none, the aircraft in the air nearest (lat, lon): its hex, empty if there
// is neither. Not `skip`, nor `skip2`.
template <typename Tracks>
String nextMovement(const Tracks &tracks, const String &code, double lat, double lon, const String &skip,
                    const String &skip2, uint32_t now, bool &inFlight) {
    String best;
    float soonest = DUE_S;
    for (const auto &kv : tracks) {
        if (kv.first == skip || kv.first == skip2) {
            continue;
        }
        float due = code.length() ? movementDue(kv.second, code.c_str(), now) : -1;
        if (due >= 0 && due < soonest) {
            soonest = due;
            best = kv.first;
        }
    }
    inFlight = best.length() == 0;
    float nearest = 1e30f;
    for (const auto &kv : tracks) {
        const auto &t = kv.second;
        if (!inFlight || kv.first == skip || kv.first == skip2 || now - t.freshMs > FRESH_MS ||
            !t.sim.haveFresh() || t.sim.fresh().ground) {
            continue;
        }
        float d = haversineNm(lat, lon, t.sim.fresh().lat, t.sim.fresh().lon);
        if (d < nearest) {
            nearest = d;
            best = kv.first;
        }
    }
    return best;
}

// Whether guest `t` has had its turn: landed and slowed on the runway, or
// down and off it; or lost. One in the air with its take-off or landing over
// carries on - `inFlight`, now one only for want of another.
template <typename Track>
bool guestOver(const Track &t, uint32_t now, bool &inFlight) {
    if (now - t.freshMs > 2 * FRESH_MS || !t.sim.haveFresh()) {
        return true;
    }
    FollowEstimate e = t.sim.at(now);
    RunwaySim::Movement m = t.sim.movement();
    if (m == RunwaySim::Movement::LANDING) {
        return e.ground && e.gsKt < ROLLED_OUT_KT;
    }
    if (m == RunwaySim::Movement::TAKEOFF) {
        return false;
    }
    if (e.ground) {
        return true;
    }
    inFlight = true;
    return false;
}

class Picker {
  public:
    // The view opened afresh: nothing shown yet. What it saw land it keeps -
    // by its hex - closed a moment for an aircraft's details over its rollout.
    void reset() {
        guest_ = String();
        guestInFlight_ = false;
        stoppedMs_ = 0;
        stoppedHex_ = String();
        atStand_ = -1;
    }
    // The guest given up - following having moved on, say.
    void clearGuest() {
        guest_ = String();
        guestInFlight_ = false;
    }
    const String &guest() const { return guest_; }

    // Each pass, the view open: what it shows, the one followed - `hex`, its
    // runway simulation `sim` - being where it is. `airport` is where the
    // view is ("" high up) and `atStand(lat, lon)` whether a place there is
    // at one of its stands. NEXT says who following moves on to, in `next`.
    template <typename Tracks, typename AtStand>
    Show pick(const String &hex, const RunwaySim &sim, const Tracks &tracks, const char *airport, AtStand atStand,
              uint32_t now, String &next) {
        if (!sim.haveFresh()) {
            clearGuest();
            return Show::FOLLOWED;
        }
        FollowEstimate e = sim.at(now);
        RunwaySim::Movement m = sim.movement();
        const FollowSample &s = sim.fresh();
        // In the air, or landing and still rolling out: itself - seen landing.
        if (!e.ground || (m == RunwaySim::Movement::LANDING && e.gsKt >= ROLLED_OUT_KT)) {
            sawLanding_ = hex;
            clearGuest();
            return Show::FOLLOWED;
        }
        // Lined up, or rolling for take-off: itself.
        if (m == RunwaySim::Movement::TAKEOFF) {
            clearGuest();
            return Show::FOLLOWED;
        }
        bool landed = sim.airborneWithin(now, LANDED_WITHIN_MS);
        bool stopped = !s.hasGs || s.gsKt < STOPPED_KT;
        if (!stopped || stoppedMs_ == 0 || stoppedHex_ != hex) {
            stoppedMs_ = stopped ? now : 0;
            atStand_ = -1;
            stoppedHex_ = hex;
        }
        Airport near;
        String code = nearestAirport(s.lat, s.lon, near, 5.0f) ? near.code : String();
        if (landed && sawLanding_ == hex) {
            // Seen landing: on to the next.
            bool inFlight;
            next = nextMovement(tracks, code, s.lat, s.lon, hex, String(), now, inFlight);
            if (next.length()) {
                clearGuest();
                return Show::NEXT;
            }
        } else if (landed && stopped && now - stoppedMs_ >= PARKED_AFTER_MS) {
            // Landed a while ago, and stopped a while: on its stand, shown
            // parked there - looked at once the view is at its airport.
            if (strcmp(airport, code.c_str()) == 0) {
                atStand_ = atStand(s.lat, s.lon) ? 1 : 0;
            }
            if (atStand_ != 0) {
                clearGuest();
                return Show::FOLLOWED;
            }
        }
        // Taxiing - or waiting to: another meanwhile, its turn had first.
        if (guest_.length()) {
            auto it = tracks.find(guest_);
            if (it != tracks.end() && !guestOver(it->second, now, guestInFlight_)) {
                if (guestInFlight_ && now - guestLookMs_ >= LOOK_MS) {
                    guestLookMs_ = now;
                    bool inFlight;
                    String better = nextMovement(tracks, code, s.lat, s.lon, hex, guest_, now, inFlight);
                    if (better.length() && !inFlight) {
                        setGuest(better, false, now);
                    }
                }
                return Show::GUEST;
            }
        }
        bool inFlight;
        String other = nextMovement(tracks, code, s.lat, s.lon, hex, guest_, now, inFlight);
        if (other.length()) {
            setGuest(other, inFlight, now);
            return Show::GUEST;
        }
        clearGuest();  // nothing else anywhere: itself
        return Show::FOLLOWED;
    }

  private:
    void setGuest(const String &hex, bool inFlight, uint32_t now) {
        guest_ = hex;
        guestInFlight_ = inFlight;
        guestLookMs_ = now;
    }

    String guest_;               // empty for none
    bool guestInFlight_ = false;  // only for want of a runway movement
    uint32_t guestLookMs_ = 0;    // when one was last looked for, for it
    String sawLanding_;          // the one followed - its hex - seen in the air or landing
    uint32_t stoppedMs_ = 0;     // since when it has been reported stopped on the ground; 0 not
    String stoppedHex_;          // which that was of
    int atStand_ = -1;           // stopped there: on a stand 1, not 0, not yet known -1
};

}  // namespace subject3d
