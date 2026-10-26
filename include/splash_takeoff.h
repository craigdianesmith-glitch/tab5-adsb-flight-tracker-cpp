#pragma once

#include <math.h>

#include <algorithm>

#include "engine_noise.h"

// The splash screen's take-off, heard: an A320's two CFM56s at take-off
// power, made as it plays by engine_noise, as the 3D view's are - no
// recording, nor any clip made beforehand. Its way is the picture's (see
// splash.cpp's poseAt()): rolling down the runway at the viewer, lifting off
// 1.26s in, its gear tucked up by 2.4s, and out over the top of the screen at
// 3s - overhead a moment after - then climbing away behind, its roar going
// with it, while boot waits on the network.
//
// The picture is drawn to be seen, not flown: from a speck at the runway's
// far end to overhead in three seconds is faster than sound. This is flown:
// lifting off at 75m/s as an A320 does, climbing at 9 degrees, so where it
// starts is what three seconds of that leave - 242m out, its roar already
// coming - and the viewer, 14m up as the picture's is, stands 40m to the
// side of its way, 42m from its engines as they pass: under it, at the
// height three seconds' climb gives, they passed 12m over the viewer's
// head. Header only, for the sound's task and the PC preview's a320-splash
// alike (tools/engine_preview/preview.cpp).
namespace splash_takeoff {

constexpr float LIFT_OFF_S = 1.26f, GEAR_UP_S = 2.1f, OVERHEAD_S = 3.25f;
constexpr float LIFT_OFF_MS = 75, ROLL_ACCEL = 2.0f, CLIMB_ACCEL = 0.8f, CLIMB_DEG = 9;
// Close, the user's pick by ear against 120m off: louder, and its limiter
// rounding off 0.9% of samples as it passes, more than the 0.1% the 3D
// view's aircraft keep - a few seconds at boot, not an aircraft followed.
constexpr float EAR_SIDE_M = 40, EAR_UP_M = 14;
constexpr float ENGINES_UP_M = 2.5f;  // over its wheels, as the 3D view has an A320's
// Heard until then, and let go: fading out, as when the 3D view closes.
constexpr float END_S = 10;
// The airframe the PC preview's A320 scenes are, with these engines.
constexpr uint32_t WHO = 0x406a3cu + (uint32_t)engine_noise::Engine::CFM56_5B;

// How far it has gone from where it starts, along the runway and up, at `t`
// seconds, and how fast it is going.
inline void flown(float t, float &along, float &up, float &speed) {
    float v0 = LIFT_OFF_MS - ROLL_ACCEL * LIFT_OFF_S;
    float rt = std::min(t, LIFT_OFF_S);
    along = v0 * rt + ROLL_ACCEL * rt * rt / 2;
    up = 0;
    speed = v0 + ROLL_ACCEL * rt;
    if (t > LIFT_OFF_S) {
        float c = t - LIFT_OFF_S, s = LIFT_OFF_MS * c + CLIMB_ACCEL * c * c / 2, g = CLIMB_DEG * (float)M_PI / 180;
        along += s * cosf(g);
        up = s * sinf(g);
        speed = LIFT_OFF_MS + CLIMB_ACCEL * c;
    }
}

// What the synthesiser is asked for `t` seconds after the splash starts:
// set a 40ms block at a time, each as it is made.
inline engine_noise::Sound at(float t) {
    using engine_noise::Kind;
    engine_noise::Sound s;
    s.kind = t < END_S ? Kind::TURBOFAN : Kind::NONE;
    s.engine = engine_noise::Engine::CFM56_5B;
    s.who = WHO;
    s.power = 1;
    float along, up, speed, overAlong, overUp, overSpeed;
    flown(std::max(0.0f, t), along, up, speed);
    flown(OVERHEAD_S, overAlong, overUp, overSpeed);
    // Coming down x to the viewer's 0, overhead at OVERHEAD_S.
    float x = overAlong - along, z = up + ENGINES_UP_M;
    float pitch = t > LIFT_OFF_S ? CLIMB_DEG * (float)M_PI / 180 : 0;
    float nose[3] = {-cosf(pitch), 0, sinf(pitch)};
    float to[3] = {-x, EAR_SIDE_M, EAR_UP_M - z};
    float d = sqrtf(to[0] * to[0] + to[1] * to[1] + to[2] * to[2]);
    float c = (nose[0] * to[0] + nose[2] * to[2]) / std::max(1.0f, d);
    s.distanceM = d;
    s.aspect = acosf(std::max(-1.0f, std::min(1.0f, c))) / (float)M_PI;
    s.placed = true;
    s.at[0] = x, s.at[1] = 0, s.at[2] = z;
    s.nose[0] = nose[0], s.nose[1] = nose[1], s.nose[2] = nose[2];
    s.ear[0] = 0, s.ear[1] = EAR_SIDE_M, s.ear[2] = EAR_UP_M;
    s.sourceUpM = z;
    s.earUpM = EAR_UP_M;
    s.hardGround = true;
    s.rollingMs = up > 0 ? 0 : speed;
    s.airspeedMs = speed;
    s.gearDown = t < GEAR_UP_S;
    s.flapsOut = true;  // its take-off flaps, out long after it has gone
    s.ms = 1000 + (uint32_t)lroundf(std::max(0.0f, t) * 1000);
    return s;
}

}  // namespace splash_takeoff
