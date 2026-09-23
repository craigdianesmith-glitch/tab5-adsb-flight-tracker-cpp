#include "radar_screen.h"

#include <M5Unified.h>
#include <math.h>

#include "airports.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
// The plot is as large as the 720px height allows. The compass letters sit
// just inside the outer ring rather than outside it - outside, they were what
// capped the radius, and "S" ran off the bottom of the screen.
constexpr int CENTER_X = 640, CENTER_Y = 392, RADIUS = 318;
constexpr int COMPASS_INSET = 26;

// Labels are placed nearest-first and one may not overlap another already
// placed, so in a cluster the closest aircraft keeps its callsign and the rest
// stay as bare blips. Better than a fixed cap, which suppressed labels in
// empty sky as readily as in a crowd.
constexpr int MAX_LABELS = 64;
constexpr int LABEL_H = 16;  // Font0 at size 2
constexpr int LABEL_PAD = 4;

struct LabelBox {
    int16_t x, y, w, h;
};

// Where each contact was last plotted, so a tap can be matched back to one.
struct Blip {
    int16_t x, y;
    String hex;
};
constexpr int MAX_BLIPS = 64;
constexpr int TAP_RADIUS = 28;
Blip g_blips[MAX_BLIPS];
int g_blipCount = 0;

// A phosphor-green ramp rather than the rest of the app's blue-grey: on a
// plan-position plot the brightness of a mark is what carries the meaning, and
// a single hue leaves brightness free to do that.
uint16_t colorBg, colorText, colorMuted, colorFaint, colorBtnBg, colorRing, colorRingText, colorBlip, colorNewBlip,
    colorHome;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x04, 0x12, 0x0A);
    colorText = M5.Display.color565(0xCF, 0xFF, 0xE2);
    colorMuted = M5.Display.color565(0x7C, 0xD9, 0xA0);
    colorFaint = M5.Display.color565(0x5A, 0x9E, 0x78);
    colorBtnBg = M5.Display.color565(0x14, 0x3A, 0x28);
    colorRing = M5.Display.color565(0x1C, 0x5A, 0x38);
    colorRingText = M5.Display.color565(0x3F, 0x8E, 0x63);
    colorBlip = M5.Display.color565(0x3D, 0xE8, 0x7C);
    colorNewBlip = M5.Display.color565(0xE6, 0xFF, 0xEE);  // brightest: a new contact
    colorHome = M5.Display.color565(0xEA, 0xFF, 0xF0);
    colorsReady = true;
}

// Rings on round numbers rather than thirds of the range, so the labels read
// as distances instead of arbitrary fractions.
int ringStep(int rangeNm) {
    if (rangeNm <= 15) return 5;
    if (rangeNm <= 30) return 10;
    if (rangeNm <= 60) return 20;
    return 50;
}

}  // namespace

void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool military) {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);
    canvas.setFont(&fonts::Font0);

    // --- header -----------------------------------------------------------
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(colorText);
    const char *title = "Radar";
    canvas.drawString(title, 16, 35);
    canvas.setTextColor(colorMuted);
    canvas.drawString(military ? " - MIL -" : " - CIV -", 16 + canvas.textWidth(title), 35);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(colorText);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    // --- rings and bearings ----------------------------------------------
    int step = ringStep(rangeNm);
    canvas.setTextSize(2);
    canvas.setTextColor(colorRingText);
    canvas.setTextDatum(BC_DATUM);
    for (int nm = step; nm < rangeNm; nm += step) {
        int r = (int)lroundf((float)nm / rangeNm * RADIUS);
        canvas.drawCircle(CENTER_X, CENTER_Y, r, colorRing);
        canvas.drawString(String(nm), CENTER_X, CENTER_Y - r - 2);
    }
    canvas.drawCircle(CENTER_X, CENTER_Y, RADIUS, colorRing);
    canvas.drawCircle(CENTER_X, CENTER_Y, RADIUS - 1, colorRing);
    canvas.drawString(String(rangeNm), CENTER_X, CENTER_Y - RADIUS - 2);

    static const char *POINTS[] = {"N", "E", "S", "W"};
    canvas.setTextSize(2);
    canvas.setTextColor(colorFaint);
    canvas.setTextDatum(MC_DATUM);
    for (int i = 0; i < 4; i++) {
        float a = i * (float)M_PI / 2.0f;
        canvas.drawString(POINTS[i], CENTER_X + (int)lroundf(sinf(a) * (RADIUS - COMPASS_INSET)),
                          CENTER_Y - (int)lroundf(cosf(a) * (RADIUS - COMPASS_INSET)));
    }
    for (int deg = 0; deg < 360; deg += 30) {
        float a = deg * (float)M_PI / 180.0f;
        int x0 = CENTER_X + (int)lroundf(sinf(a) * (RADIUS - 8));
        int y0 = CENTER_Y - (int)lroundf(cosf(a) * (RADIUS - 8));
        int x1 = CENTER_X + (int)lroundf(sinf(a) * RADIUS);
        int y1 = CENTER_Y - (int)lroundf(cosf(a) * RADIUS);
        canvas.drawLine(x0, y0, x1, y1, colorRing);
    }

    // home marker, tagged with the nearest airport so the centre says where
    // it is rather than just being a cross
    canvas.drawLine(CENTER_X - 9, CENTER_Y, CENTER_X + 9, CENTER_Y, colorHome);
    canvas.drawLine(CENTER_X, CENTER_Y - 9, CENTER_X, CENTER_Y + 9, colorHome);
    String here = nearestAirportCode(lat, lon);
    if (here.length()) {
        canvas.setTextSize(2);
        canvas.setTextColor(colorHome);
        canvas.setTextDatum(TC_DATUM);
        canvas.drawString(here, CENTER_X, CENTER_Y + 13);
    }

    // --- contacts ---------------------------------------------------------
    // Flat-earth projection: over a 150nm radius the error is far smaller than
    // a blip, and it keeps this to two multiplications per aircraft.
    double nmPerDegLon = 60.0 * cos(lat * M_PI / 180.0);
    int plotted = 0, labelled = 0;
    LabelBox taken[MAX_LABELS];
    int takenCount = 0;
    g_blipCount = 0;

    for (size_t i = 0; i < aircraft.size(); i++) {
        const Aircraft &ac = aircraft[i];
        if (!ac.hasPos) {
            continue;
        }
        double dNorth = (ac.lat - lat) * 60.0;
        double dEast = (ac.lon - lon) * nmPerDegLon;
        float px = CENTER_X + (float)(dEast / rangeNm * RADIUS);
        float py = CENTER_Y - (float)(dNorth / rangeNm * RADIUS);

        // anything beyond the outer ring would be drawn outside the plot
        float dx = px - CENTER_X, dy = py - CENTER_Y;
        if (dx * dx + dy * dy > (float)RADIUS * RADIUS) {
            continue;
        }
        plotted++;

        bool isNewHere = (i < isNew.size()) && isNew[i];
        uint16_t color = isNewHere ? colorNewBlip : colorBlip;

        if (ac.hasTrack) {
            // One minute of flight at current groundspeed, with a floor so a
            // slow or speed-less contact still shows which way it's pointing.
            float gs = ac.speedStr.toFloat();
            float nm = gs / 60.0f;
            float len = nm / rangeNm * RADIUS;
            if (len < 14.0f) {
                len = 14.0f;
            }
            float a = ac.track * (float)M_PI / 180.0f;
            canvas.drawLine((int)px, (int)py, (int)(px + sinf(a) * len), (int)(py - cosf(a) * len), color);
        }
        canvas.fillCircle((int)px, (int)py, 5, color);
        if (g_blipCount < MAX_BLIPS) {
            g_blips[g_blipCount++] = {(int16_t)px, (int16_t)py, ac.hex};
        }

        canvas.setTextSize(2);
        int lw = canvas.textWidth(ac.callsign);
        int lx = (int)px + 9;
        int ly = (int)py + 7;
        if (lx + lw > CENTER_X + RADIUS) {
            lx = (int)px - 9 - lw;  // would run off the plot - put it on the left
        }
        bool clear = true;
        for (int j = 0; j < takenCount; j++) {
            const LabelBox &b = taken[j];
            if (lx - LABEL_PAD < b.x + b.w && lx + lw + LABEL_PAD > b.x && ly - LABEL_PAD < b.y + b.h &&
                ly + LABEL_H + LABEL_PAD > b.y) {
                clear = false;
                break;
            }
        }
        if (clear && takenCount < MAX_LABELS) {
            canvas.setTextColor(color);
            canvas.setTextDatum(TL_DATUM);
            canvas.drawString(ac.callsign, lx, ly);
            taken[takenCount++] = {(int16_t)lx, (int16_t)ly, (int16_t)lw, (int16_t)LABEL_H};
            labelled++;
        }
    }

    // --- footings ---------------------------------------------------------
    canvas.setTextSize(2);
    canvas.setTextColor(colorFaint);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(String(rangeNm) + " nm range   rings every " + String(step) + " nm", 16, 664);
    canvas.setTextDatum(TR_DATUM);
    canvas.drawString(String(plotted) + " contacts" + (labelled < plotted ? "   " + String(labelled) + " labelled" : ""),
                      1264, 664);

    screen::flush();
}

RadarAction radarScreenHandleTouch(int x, int y, String &outHex) {
    if (x >= BACK_X && x < BACK_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return RadarAction::BACK;
    }

    // Nearest blip within reach, not merely the first one found: in a cluster
    // the closest to the finger is the one meant.
    int best = -1;
    long bestDist = (long)TAP_RADIUS * TAP_RADIUS;
    for (int i = 0; i < g_blipCount; i++) {
        long dx = x - g_blips[i].x;
        long dy = y - g_blips[i].y;
        long d = dx * dx + dy * dy;
        if (d <= bestDist) {
            bestDist = d;
            best = i;
        }
    }
    if (best >= 0) {
        outHex = g_blips[best].hex;
        return RadarAction::SELECT;
    }
    return RadarAction::NONE;
}
