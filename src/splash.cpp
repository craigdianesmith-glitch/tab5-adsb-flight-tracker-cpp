#include "splash.h"

#include <M5Unified.h>
#include <esp_heap_caps.h>
#include <math.h>

#include <algorithm>

#include "config.h"
#include "screen.h"

namespace {

constexpr int W = 1280, H = 720;
constexpr int HORIZON = 470;
constexpr int VP_X = 640;  // where the runway's edges meet, on the horizon
constexpr uint32_t PLAY_MS = 3000;

struct Pt {
    float x, y;
};

uint16_t rgb(int r, int g, int b) { return M5.Display.color565(r, g, b); }

uint16_t mix(const int a[3], const int b[3], float t) {
    return rgb(a[0] + (int)((b[0] - a[0]) * t), a[1] + (int)((b[1] - a[1]) * t), a[2] + (int)((b[2] - a[2]) * t));
}

// Any simple polygon, filled a scanline at a time: the plane's outline isn't
// convex, which a fan of triangles would need it to be.
void fillPoly(const Pt *p, int n, uint16_t color) {
    auto &canvas = screen::canvas();
    float top = p[0].y, bottom = p[0].y;
    for (int i = 1; i < n; i++) {
        top = std::min(top, p[i].y);
        bottom = std::max(bottom, p[i].y);
    }
    int y0 = std::max(0, (int)ceilf(top)), y1 = std::min(H - 1, (int)floorf(bottom));
    float xs[24];
    for (int y = y0; y <= y1; y++) {
        float sy = y + 0.5f;
        int k = 0;
        for (int i = 0; i < n && k < 24; i++) {
            const Pt &a = p[i], &b = p[(i + 1) % n];
            if ((a.y <= sy && b.y > sy) || (b.y <= sy && a.y > sy)) {
                xs[k++] = a.x + (sy - a.y) * (b.x - a.x) / (b.y - a.y);
            }
        }
        std::sort(xs, xs + k);
        for (int i = 0; i + 1 < k; i += 2) {
            int x0 = (int)lroundf(xs[i]), x1 = (int)lroundf(xs[i + 1]);
            if (x1 > x0) {
                canvas.drawFastHLine(x0, y, x1 - x0, color);
            }
        }
    }
}

// --- the scene ---------------------------------------------------------------

// Dusk: night overhead, the sky warming down to a glow along the horizon.
void drawSky() {
    auto &canvas = screen::canvas();
    static const int STOPS_Y[] = {0, 250, 390, HORIZON};
    static const int STOPS[][3] = {{6, 10, 30}, {34, 34, 80}, {140, 66, 84}, {246, 142, 72}};
    for (int y = 0; y < HORIZON; y++) {
        int s = 0;
        while (s < 2 && y >= STOPS_Y[s + 1]) {
            s++;
        }
        float t = (float)(y - STOPS_Y[s]) / (STOPS_Y[s + 1] - STOPS_Y[s]);
        canvas.drawFastHLine(0, y, W, mix(STOPS[s], STOPS[s + 1], t));
    }
    // The sun just setting, behind the far end of the airfield.
    canvas.fillCircle(330, HORIZON + 6, 74, rgb(250, 160, 84));
    canvas.fillCircle(330, HORIZON + 6, 54, rgb(255, 190, 110));
    canvas.fillCircle(330, HORIZON + 6, 40, rgb(255, 222, 150));
}

// The airfield: dark ground, a runway running off to the horizon in
// perspective, with its centreline and edge lights.
void drawGround() {
    auto &canvas = screen::canvas();
    static const int NEAR[3] = {12, 11, 16}, FAR[3] = {38, 26, 34};
    for (int y = HORIZON; y < H; y++) {
        canvas.drawFastHLine(0, y, W, mix(FAR, NEAR, (float)(y - HORIZON) / (H - HORIZON)));
    }
    // Lights of somewhere along the horizon.
    for (int i = 0; i < 70; i++) {
        int x = (i * 977) % W;
        if (abs(x - VP_X) < 40) {
            continue;
        }
        canvas.drawPixel(x, HORIZON + 2 + (i * 7) % 5, rgb(255, 200 + (i * 13) % 50, 120));
    }

    // Along the runway, d is how far: 1 at the bottom of the screen, growing
    // towards the horizon. Its edges are a fixed half-width either side.
    constexpr float HALF = 410;  // the half-width at d = 1, in pixels
    auto yAt = [](float d) { return HORIZON + (H - HORIZON) / d; };
    auto xAt = [](float d, float side) { return VP_X + side * HALF / d; };
    Pt runway[] = {{xAt(1, -1), (float)H}, {xAt(1, 1), (float)H}, {xAt(40, 1), yAt(40)}, {xAt(40, -1), yAt(40)}};
    fillPoly(runway, 4, rgb(46, 46, 56));
    for (float side : {-1.0f, 1.0f}) {
        Pt edge[] = {{xAt(1, side * 0.985f), (float)H},
                     {xAt(1, side), (float)H},
                     {xAt(40, side), yAt(40)},
                     {xAt(40, side * 0.985f), yAt(40)}};
        fillPoly(edge, 4, rgb(150, 150, 160));
    }
    // The centreline, a dash and a gap at a time.
    for (float d = 1.0f; d < 30; d *= 1.18f) {
        float d1 = d * 1.09f, w0 = 0.014f, w1 = 0.014f;
        Pt dash[] = {{xAt(d, -w0), yAt(d)}, {xAt(d, w0), yAt(d)}, {xAt(d1, w1), yAt(d1)}, {xAt(d1, -w1), yAt(d1)}};
        fillPoly(dash, 4, rgb(226, 226, 232));
    }
    // Edge lights, warm white, getting smaller into the distance.
    for (float d = 1.05f; d < 36; d *= 1.12f) {
        for (float side : {-1.06f, 1.06f}) {
            int r = std::max(1, (int)lroundf(5 / d));
            canvas.fillCircle((int)xAt(d, side), (int)yAt(d), r, rgb(255, 214, 140));
        }
    }
}

// --- the aircraft ------------------------------------------------------------
// An airliner seen nose-on, coming down the runway at the viewer: in units of
// a hundredth of its wingspan, x across, y up, the middle of the fuselage at
// the origin. Each side is drawn from the left half, mirrored.

const Pt WING[] = {{-5, -1.5f}, {-49, 4.6f}, {-50, 3.6f}, {-5, -4.5f}};
const Pt WINGLET[] = {{-49, 3.6f}, {-50.4f, 3.6f}, {-51, 9.5f}, {-49.8f, 9.5f}};
const Pt TAILPLANE[] = {{-1, 3.5f}, {-16, 6.2f}, {-16, 5.4f}, {-1, 2}};
const Pt FIN[] = {{-0.9f, 5}, {0.9f, 5}, {0.6f, 23}, {-0.6f, 23}};
const Pt PYLON[] = {{-17, -2.6f}, {-16, -2.6f}, {-16.6f, -4.5f}, {-17.6f, -4.5f}};
const Pt WINDSCREEN[] = {{-3.6f, 2.4f}, {-0.5f, 3.4f}, {-0.5f, 1.9f}, {-3.9f, 1.1f}};
constexpr float FUSELAGE_R = 6;
constexpr float ENGINE_X = -17.5f, ENGINE_Y = -6.2f, ENGINE_R = 3.4f;
constexpr float GEAR_DROP = 11;  // from the middle of the fuselage to the bottom of the wheels

struct Pose {
    float x, y, scale;  // the middle of the fuselage, and pixels to a unit
    float gear;         // 1 down, 0 up
};

Pt place(const Pose &p, Pt u) { return {p.x + p.scale * u.x, p.y - p.scale * u.y}; }

template <size_t N>
void part(const Pose &p, const Pt (&shape)[N], uint16_t color, bool mirrored) {
    Pt out[N];
    for (size_t i = 0; i < N; i++) {
        out[i] = place(p, {mirrored ? -shape[i].x : shape[i].x, shape[i].y});
    }
    fillPoly(out, (int)N, color);
}

template <size_t N>
void pair(const Pose &p, const Pt (&shape)[N], uint16_t color) {
    part(p, shape, color, false);
    part(p, shape, color, true);
}

void disc(const Pose &p, Pt at, float r, uint16_t color) {
    Pt c = place(p, at);
    screen::canvas().fillCircle((int)lroundf(c.x), (int)lroundf(c.y), std::max(1, (int)lroundf(r * p.scale)), color);
}

// A light: its glow, and the light itself.
void light(const Pose &p, Pt at, float r, uint16_t glow, uint16_t core) {
    disc(p, at, r * 2, glow);
    disc(p, at, r, core);
}

void drawPlane(const Pose &p) {
    uint16_t body = rgb(14, 16, 28), shade = rgb(26, 28, 42), lit = rgb(186, 106, 74);
    pair(p, TAILPLANE, shade);
    part(p, FIN, shade, false);
    pair(p, WING, body);
    pair(p, WINGLET, body);
    // The sun behind it catching the wings' leading edges.
    for (float side : {-1.0f, 1.0f}) {
        Pt a = place(p, {side * 5, -1.5f}), b = place(p, {side * 49, 4.6f});
        screen::canvas().drawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, lit);
    }
    // The gear, tucking up into the wings and fuselage after lift-off.
    float drop = GEAR_DROP * p.gear;
    if (p.gear > 0.05f) {
        for (float gx : {-5.5f, 0.0f, 5.5f}) {
            Pt leg[] = {{gx - 0.4f, -3}, {gx + 0.4f, -3}, {gx + 0.4f, -drop + 1.5f}, {gx - 0.4f, -drop + 1.5f}};
            part(p, leg, body, false);
            float w = gx == 0 ? 1.2f : 2.4f;
            Pt wheels[] = {{gx - w, -drop + 3}, {gx + w, -drop + 3}, {gx + w, -drop}, {gx - w, -drop}};
            part(p, wheels, body, false);
        }
    }
    disc(p, {0, 0}, FUSELAGE_R, body);
    for (float side : {-1.0f, 1.0f}) {
        Pt pylon[4];
        for (int i = 0; i < 4; i++) {
            pylon[i] = place(p, {side * PYLON[i].x, PYLON[i].y});
        }
        fillPoly(pylon, 4, body);
        disc(p, {side * ENGINE_X, ENGINE_Y}, ENGINE_R, rgb(74, 62, 72));            // the intake's lip, lit
        disc(p, {side * ENGINE_X, ENGINE_Y}, ENGINE_R * 0.84f, rgb(22, 24, 36));  // the fan
        disc(p, {side * ENGINE_X, ENGINE_Y}, ENGINE_R * 0.22f, rgb(52, 50, 64));  // its spinner
    }
    uint16_t window = rgb(84, 92, 120);
    pair(p, WINDSCREEN, window);

    // Landing lights blazing at the wing roots, the beacon on top, and the
    // wingtips: red on its left - the viewer's right - green on its right.
    for (float side : {-1.0f, 1.0f}) {
        light(p, {side * 8.5f, -2.6f}, 1.3f, rgb(214, 196, 160), rgb(255, 252, 238));
    }
    light(p, {0, FUSELAGE_R + 0.6f}, 0.5f, rgb(110, 30, 40), rgb(255, 60, 60));
    light(p, {49.5f, 4.2f}, 0.5f, rgb(110, 30, 40), rgb(255, 70, 70));
    light(p, {-49.5f, 4.2f}, 0.5f, rgb(30, 100, 50), rgb(80, 255, 120));
}

// --- the titles --------------------------------------------------------------

// Drawn a letter at a time, opened out the way a logo would be.
void spaced(const char *text, int centreX, int y, int extra) {
    auto &canvas = screen::canvas();
    int w = 0;
    for (const char *c = text; *c; c++) {
        char s[2] = {*c, 0};
        w += canvas.textWidth(s) + (c[1] ? extra : 0);
    }
    int x = centreX - w / 2;
    for (const char *c = text; *c; c++) {
        char s[2] = {*c, 0};
        canvas.drawString(s, x, y);
        x += canvas.textWidth(s) + extra;
    }
}

void drawTitles() {
    auto &canvas = screen::canvas();
    canvas.setTextDatum(TL_DATUM);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextSize(3);
    canvas.setTextColor(rgb(4, 6, 18));
    spaced("OVERHEAD", W / 2 + 5, 30 + 5, 14);
    canvas.setTextColor(rgb(255, 255, 255));
    spaced("OVERHEAD", W / 2, 30, 14);

    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextSize(1);
    canvas.setTextColor(rgb(246, 170, 100));
    spaced("LIVE  FLIGHT  TRACKER", W / 2, 184, 6);

    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextColor(rgb(150, 146, 170));
    canvas.setTextDatum(BR_DATUM);
    canvas.drawString("v" FIRMWARE_VERSION, W - 16, H - 12);
    canvas.setTextDatum(TL_DATUM);
}

// The plane at `t`, 0 to 1: rolling down the runway from its far end, lifting
// off, and climbing out over the viewer - overhead - coming ever faster
// out of the screen. Its distance d is the runway's (see drawGround): 1 at
// the bottom of the screen. Its wingspan is 36m against the runway's 45m, and
// the viewer is about 14m up - 250 pixels at d = 1, 18 to the metre.
Pose poseAt(float t) {
    constexpr float D0 = 40, D1 = 0.32f, PX_PER_M = 18.2f;
    float d = D0 * powf(D1 / D0, powf(t, 1.6f));
    float lift = std::max(0.0f, (t - 0.42f) / 0.58f);
    float altM = 30 * lift * lift;
    float scale = 0.8f * 2 * 410 / 100 / d;
    float wheelsY = HORIZON + (250 - altM * PX_PER_M) / d;
    float gear = std::min(1.0f, std::max(0.0f, 1 - (t - 0.6f) / 0.2f));
    return {(float)VP_X, wheelsY - GEAR_DROP * gear * scale, scale, gear};
}

// What a frame of the plane can touch, as pixel rows and columns [x0, x1) and
// [y0, y1): its outline, and room round it for the lights' glows and wheels.
struct Box {
    int x0, y0, x1, y1;
};

Box planeBox(const Pose &p) {
    // The wingtips and winglets, the fin's top and the wheels' bottom.
    int x0 = (int)floorf(p.x - 52 * p.scale), x1 = (int)ceilf(p.x + 52 * p.scale);
    int y0 = (int)floorf(p.y - 24 * p.scale), y1 = (int)ceilf(p.y + (GEAR_DROP + 2) * p.scale);
    int m = (int)ceilf(2 * p.scale) + 3;  // the lights' glows
    Box b = {std::max(0, x0 - m), std::max(0, y0 - m), std::min(W, x1 + m), std::min(H, y1 + m)};
    if (b.x0 >= b.x1 || b.y0 >= b.y1) {
        return {0, 0, 0, 0};  // off the screen
    }
    return b;
}

// The scene without the plane, back over `b` from `bg`.
void restore(const uint16_t *bg, const Box &b) {
    auto *buf = (uint16_t *)screen::canvas().getBuffer();
    for (int y = b.y0; y < b.y1; y++) {
        memcpy(buf + (size_t)y * W + b.x0, bg + (size_t)y * W + b.x0, (size_t)(b.x1 - b.x0) * 2);
    }
}

}  // namespace

void splashPlay() {
    uint32_t start = millis();
    // Only the plane moves: the scene is drawn once and kept, and each frame
    // puts it back over where the plane was, draws the plane where it is now,
    // and pushes just that patch rather than the whole screen: a few
    // milliseconds a frame where the whole screen took 85. Held to the
    // panel's 60 a second, with the time between given to the speaker's task
    // playing the take-off.
    drawSky();
    drawGround();
    drawTitles();
    size_t bytes = (size_t)W * H * 2;
    auto *bg = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (bg != nullptr) {
        memcpy(bg, screen::canvas().getBuffer(), bytes);
    }
    Box shown = {0, 0, 0, 0};
    int frames = 0;
    constexpr uint32_t FRAME_MS = 16;
    for (;;) {
        uint32_t frameStart = millis();
        float t = std::min(1.0f, (float)(frameStart - start) / PLAY_MS);
        Pose pose = poseAt(t);
        Box now = planeBox(pose);
        if (bg == nullptr) {
            drawSky();  // short of memory for the copy: the whole scene each frame
            drawGround();
            drawPlane(pose);
            drawTitles();
            screen::markAllDirty();
        } else {
            Box both = frames == 0           ? Box{0, 0, W, H}
                       : shown.x1 == 0 ? now
                       : now.x1 == 0   ? shown
                                       : Box{std::min(shown.x0, now.x0), std::min(shown.y0, now.y0),
                                             std::max(shown.x1, now.x1), std::max(shown.y1, now.y1)};
            restore(bg, both);
            drawPlane(pose);
            // The titles back over it, where it passes behind them - they
            // are in front, the plane flying on behind the logo.
            auto &canvas = screen::canvas();
            canvas.setClipRect(both.x0, both.y0, both.x1 - both.x0, both.y1 - both.y0);
            drawTitles();
            canvas.clearClipRect();
            if (both.x1 > both.x0) {
                screen::markDirty(both.x0, both.y0, both.x1 - both.x0, both.y1 - both.y0);
            }
        }
        screen::flush();
        shown = now;
        frames++;
        if (t >= 1.0f) {
            break;
        }
        uint32_t spent = millis() - frameStart;
        delay(spent < FRAME_MS ? FRAME_MS - spent : 1);
    }
    heap_caps_free(bg);
    Serial.printf("[splash] %d frames in %lums\n", frames, (unsigned long)(millis() - start));
}

void splashStatus(const String &text) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(BL_DATUM);
    int w = canvas.textWidth(text);
    canvas.fillRoundRect(8, H - 44, w + 20, 34, 6, rgb(10, 10, 16));
    canvas.setTextColor(rgb(210, 206, 220));
    canvas.drawString(text, 18, H - 18);
    canvas.setTextDatum(TL_DATUM);
    screen::markDirty(8, H - 44, w + 20, 34);
    screen::flush();
}
