#include "radar_screen.h"

#include <M5Unified.h>
#include <math.h>

#include "airports.h"
#include "alerts.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
// The centre toggle sits beside Back, the same size, and is labelled with
// what the plot is centred on now.
constexpr int CENTRE_W = 124;
constexpr int CENTRE_X = BACK_X - 12 - CENTRE_W;
// Recording and playback follow along the same row.
constexpr int REC_W = 124;
constexpr int REC_X = CENTRE_X - 12 - REC_W;
constexpr int REPLAY_W = 124;
constexpr int REPLAY_X = REC_X - 12 - REPLAY_W;
// The plot is as large as the 720px height allows. The compass letters sit
// just inside the outer ring rather than outside it - outside, they were what
// capped the radius, and "S" ran off the bottom of the screen.
constexpr int CENTER_X = 640, CENTER_Y = 392, RADIUS = 318;
constexpr int COMPASS_INSET = 26;

// Labels are placed nearest-first and one may not overlap another already
// placed, so in a cluster the closest aircraft keeps its callsign and the rest
// stay as bare blips. Better than a fixed cap, which suppressed labels in
// empty sky as readily as in a crowd.
constexpr int FOOTER_Y = 664;
constexpr int BOX_PAD_X = 10, BOX_PAD_Y = 6;
constexpr int BOX_H = 16 + 2 * BOX_PAD_Y;  // one line of Font0 at size 2, boxed
constexpr int BOX_GAP = 10;
// Text top to text top, so the two stacked boxes clear each other by BOX_GAP.
constexpr int LEGEND_OFFSET = BOX_H + BOX_GAP;

// What a data refresh repaints, where entering the screen repaints the lot:
// the plot square, with room at its edges for a blip's chevron and callsign,
// down to the bottom of the canvas; and, separately, the footer readouts to
// its right. The header and the margins either side of the plot never change
// while the screen is up, and pushing them again was two-thirds of a refresh.
// Contacts are clipped to the square so nothing is drawn outside what gets
// erased - a vector at short range can otherwise reach well past the ring.
constexpr int PLOT_L = CENTER_X - RADIUS - 16, PLOT_R = CENTER_X + RADIUS + 16;
constexpr int PLOT_T = 56, PLOT_B = 720;
constexpr int FOOT_L = PLOT_R, FOOT_R = 1280;
constexpr int FOOT_T = FOOTER_Y - LEGEND_OFFSET - BOX_PAD_Y, FOOT_B = FOOTER_Y - BOX_PAD_Y + BOX_H;

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
// Above the poll's MAX_CONTACTS with room to spare: alerted aircraft are kept
// beyond that cap and come last, so a tighter limit here made exactly the
// ones worth tapping the ones that couldn't be.
constexpr int MAX_BLIPS = 128;
constexpr int TAP_RADIUS = 28;
Blip g_blips[MAX_BLIPS];
int g_blipCount = 0;

// A phosphor-green ramp rather than the rest of the app's blue-grey: on a
// plan-position plot the brightness of a mark is what carries the meaning, and
// a single hue leaves brightness free to do that.
uint16_t colorDisabled;
uint16_t colorBg, colorText, colorMuted, colorFaint, colorBtnBg, colorRing, colorRingText, colorBlip, colorNewBlip,
    colorHome, colorTrail, colorAlert, colorEmergency, colorRec;
RadarPalette g_palette;

RecButton g_recState = RecButton::IDLE;
uint32_t g_recElapsedS = 0;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x04, 0x12, 0x0A);
    colorText = M5.Display.color565(0xCF, 0xFF, 0xE2);
    colorMuted = M5.Display.color565(0x7C, 0xD9, 0xA0);
    colorFaint = M5.Display.color565(0x5A, 0x9E, 0x78);
    colorBtnBg = M5.Display.color565(0x14, 0x3A, 0x28);
    colorDisabled = M5.Display.color565(0x4A, 0x55, 0x50);
    colorRing = M5.Display.color565(0x1C, 0x5A, 0x38);
    colorRingText = M5.Display.color565(0x3F, 0x8E, 0x63);
    colorBlip = M5.Display.color565(0x3D, 0xE8, 0x7C);
    colorNewBlip = M5.Display.color565(0xE6, 0xFF, 0xEE);  // brightest: a new contact
    colorHome = M5.Display.color565(0xEA, 0xFF, 0xF0);
    colorTrail = M5.Display.color565(0x2A, 0x7A, 0x4E);  // dimmer than any blip: where it was, not where it is
    // The one exception to the single hue: an alerted contact has to stand out
    // from the sky around it at a glance, which no shade of green does.
    colorAlert = M5.Display.color565(0xFF, 0xB3, 0x2E);
    colorEmergency = M5.Display.color565(0xFF, 0x4D, 0x4D);
    colorRec = M5.Display.color565(0xC0, 0x39, 0x2B);
    g_palette = {colorBg, colorText, colorMuted, colorFaint, colorBtnBg, colorRing, colorRec, colorDisabled};
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

// The footer readouts sit in boxes, the way a scope's data blocks do. A
// right-aligned one is placed from its right edge so that the box, rather
// than the text inside it, lines up with the margin the plot is given.
void footerBox(const String &text, int edgeX, int textY, bool rightAligned) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    int w = canvas.textWidth(text) + 2 * BOX_PAD_X;
    int x = rightAligned ? edgeX - w : edgeX;
    canvas.drawRect(x, textY - BOX_PAD_Y, w, BOX_H, colorRing);
    canvas.setTextColor(colorFaint);
    canvas.setTextDatum(rightAligned ? TR_DATUM : TL_DATUM);
    canvas.drawString(text, rightAligned ? edgeX - BOX_PAD_X : edgeX + BOX_PAD_X, textY);
}

// Vertical trend as a shape rather than a colour or a brightness: the plot is
// one hue on purpose, and brightness is already spoken for marking a new
// contact, so neither was free to take on a second meaning. A chevron above
// the blip for a climb and below for a descent - pointing the way the
// aircraft is going - and nothing at all for level or ground traffic, which
// keeps the plot quiet when nothing is doing anything vertically.
//
// It sits directly over or under the blip, where the callsign (placed to one
// side, on the same row) can't reach it.
void drawTrend(int px, int py, const String &status, uint16_t color) {
    int dir = 0;
    if (status == "CLIMB") {
        dir = -1;
    } else if (status == "DESCEND") {
        dir = 1;
    }
    if (dir == 0) {
        return;
    }
    auto &canvas = screen::canvas();
    constexpr int GAP = 9, HEIGHT = 5, HALF_W = 5;
    // Two strokes: a single line reads as thin and accidental beside a 5px blip.
    for (int i = 0; i <= 1; i++) {
        int base = py + dir * (GAP + i);
        int tip = py + dir * (GAP + HEIGHT + i);
        canvas.drawLine(px - HALF_W, base, px, tip, color);
        canvas.drawLine(px, tip, px + HALF_W, base, color);
    }
}

// A button with nothing behind it: outlined rather than filled, its label
// in grey, so it reads as there but unavailable.
void drawDisabledButton(int x, int w, const char *label) {
    auto &canvas = screen::canvas();
    canvas.fillRect(x, BACK_Y, w, BACK_H, colorBg);
    canvas.drawRoundRect(x, BACK_Y, w, BACK_H, 6, colorDisabled);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(colorDisabled);
    canvas.drawString(label, x + w / 2, BACK_Y + BACK_H / 2);
    screen::markDirty(x, BACK_Y, w, BACK_H);
}

void drawReplaysButton() {
    if (g_recState == RecButton::NO_CARD) {
        drawDisabledButton(REPLAY_X, REPLAY_W, "Replays");
        return;
    }
    auto &canvas = screen::canvas();
    canvas.fillRect(REPLAY_X, BACK_Y, REPLAY_W, BACK_H, colorBg);
    canvas.fillRoundRect(REPLAY_X, BACK_Y, REPLAY_W, BACK_H, 6, colorBtnBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(colorText);
    canvas.drawString("Replays", REPLAY_X + REPLAY_W / 2, BACK_Y + BACK_H / 2);
    screen::markDirty(REPLAY_X, BACK_Y, REPLAY_W, BACK_H);
}

// Idle, it is a red dot and REC, the way every recorder has said it. Running,
// it turns red and counts, with a stop square: what a tap will now do.
void drawRecButton() {
    if (g_recState == RecButton::NO_CARD) {
        drawDisabledButton(REC_X, REC_W, "NO SD");
        return;
    }
    auto &canvas = screen::canvas();
    canvas.fillRect(REC_X, BACK_Y, REC_W, BACK_H, colorBg);
    bool running = (g_recState == RecButton::RECORDING);
    canvas.fillRoundRect(REC_X, BACK_Y, REC_W, BACK_H, 6, running ? colorRec : colorBtnBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    int midY = BACK_Y + BACK_H / 2;
    if (running) {
        canvas.fillRect(REC_X + 16, midY - 6, 12, 12, colorText);
        uint32_t s = g_recElapsedS;
        char buf[12];
        if (s >= 36000) {
            // Ten hours on, 10:00:00 is a character wider than the button has
            // room for beside the stop square - and the seconds no longer
            // matter - so it counts in hours and minutes instead.
            snprintf(buf, sizeof(buf), "%luh%02lum", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60));
        } else if (s >= 3600) {
            snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60),
                     (unsigned long)(s % 60));
        } else {
            snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
        }
        canvas.setTextColor(colorText);
        canvas.drawString(buf, REC_X + 78, midY);
    } else {
        canvas.fillCircle(REC_X + 34, midY, 7, colorEmergency);
        canvas.setTextColor(colorText);
        canvas.drawString("REC", REC_X + 74, midY);
    }
    screen::markDirty(REC_X, BACK_Y, REC_W, BACK_H);
}

// Title and buttons: drawn on a full repaint only, since nothing in them
// changes while the screen is up.
void drawHeader(bool military, RadarCentre centre) {
    auto &canvas = screen::canvas();
    // Set here rather than trusted from whatever drew last: the table leaves
    // its cells' FreeSans selected, and at size 3 the title came out huge.
    canvas.setFont(&fonts::Font0);
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

    canvas.fillRoundRect(CENTRE_X, BACK_Y, CENTRE_W, BACK_H, 6, colorBtnBg);
    canvas.drawString(centre == RadarCentre::HOME ? "HOME" : "AIRPORT", CENTRE_X + CENTRE_W / 2,
                      BACK_Y + BACK_H / 2);

    drawReplaysButton();
    drawRecButton();
}

}  // namespace

void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool military, RadarCentre centre, bool full) {
    ensureColors();
    if (full) {
        screen::clear(colorBg);
        drawHeader(military, centre);
    }
    radarPlotDraw({aircraft, isNew, nullptr, lat, lon, rangeNm, centre, true}, full);
    if (full) {
        screen::flush();
    }
}

void radarScreenSetRecording(RecButton state, uint32_t elapsedS, bool onScreen) {
    if (state == g_recState && elapsedS == g_recElapsedS) {
        return;
    }
    // Replays greys out with REC, but only a change of state touches it: the
    // count ticking over every second while recording is REC's alone.
    bool stateChanged = (state != g_recState);
    g_recState = state;
    g_recElapsedS = elapsedS;
    if (onScreen) {
        ensureColors();
        if (stateChanged) {
            drawReplaysButton();
        }
        drawRecButton();
        screen::flush();
    }
}

const RadarPalette &radarPalette() {
    ensureColors();
    return g_palette;
}

void radarPlotDraw(const RadarScene &scene, bool full) {
    ensureColors();
    auto &canvas = screen::canvas();
    const std::vector<Aircraft> &aircraft = scene.aircraft;
    const std::vector<uint8_t> &isNew = scene.isNew;
    double lat = scene.lat, lon = scene.lon;
    int rangeNm = scene.rangeNm;
    RadarCentre centre = scene.centre;
    if (!full) {
        screen::fillRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T, colorBg);
        canvas.fillRect(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T, colorBg);
    }
    canvas.setFont(&fonts::Font0);

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

    // Centred on the airport, the cross is tagged with its code so the centre
    // says where it is; centred on home, it is a bare cross. Airport mode with
    // nothing in range of the lookup falls back to home, since there is no
    // other point to centre on.
    Airport airport;
    if (centre == RadarCentre::AIRPORT && nearestAirport(lat, lon, airport)) {
        lat = airport.lat;
        lon = airport.lon;
        canvas.setTextSize(2);
        canvas.setTextColor(colorHome);
        canvas.setTextDatum(TC_DATUM);
        canvas.drawString(airport.code, CENTER_X, CENTER_Y + 13);
    }
    canvas.drawLine(CENTER_X - 9, CENTER_Y, CENTER_X + 9, CENTER_Y, colorHome);
    canvas.drawLine(CENTER_X, CENTER_Y - 9, CENTER_X, CENTER_Y + 9, colorHome);

    // --- contacts ---------------------------------------------------------
    // Flat-earth projection: over a 150nm radius the error is far smaller than
    // a blip, and it keeps this to two multiplications per aircraft.
    double nmPerDegLon = 60.0 * cos(lat * M_PI / 180.0);
    int plotted = 0, labelled = 0;
    canvas.setClipRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);
    LabelBox taken[MAX_LABELS];
    int takenCount = 0;
    g_blipCount = 0;

    // Trails first, so every blip sits on top of its own and anyone else's.
    if (scene.trails != nullptr) {
        bool penDown = false;
        int lastX = 0, lastY = 0;
        for (const replay::TrailPoint &p : *scene.trails) {
            if (isnan(p.lat)) {
                penDown = false;
                continue;
            }
            int x = CENTER_X + (int)lroundf((float)((p.lon - lon) * nmPerDegLon / rangeNm * RADIUS));
            int y = CENTER_Y - (int)lroundf((float)((p.lat - lat) * 60.0 / rangeNm * RADIUS));
            if (penDown) {
                canvas.drawLine(lastX, lastY, x, y, colorTrail);
            } else {
                canvas.fillCircle(x, y, 1, colorTrail);
            }
            lastX = x;
            lastY = y;
            penDown = true;
        }
    }

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
        uint16_t color = (ac.alert & ALERT_EMERGENCY) ? colorEmergency
                         : ac.alert                   ? colorAlert
                         : isNewHere                  ? colorNewBlip
                                                      : colorBlip;

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
        drawTrend((int)px, (int)py, ac.status, color);
        if (g_blipCount < MAX_BLIPS) {
            g_blips[g_blipCount++] = {(int16_t)px, (int16_t)py, ac.hex};
        }

        // The label flashes between red and an ordinary contact's colour. A
        // flashing one is drawn even where it would collide with another
        // label: it's the one being drawn attention to.
        AlertFlash flash = scene.flash ? alertFlash(ac.hex) : AlertFlash::NONE;
        uint16_t labelColor = flash == AlertFlash::RED ? colorEmergency
                              : flash == AlertFlash::OFF ? (isNewHere ? colorNewBlip : colorBlip)
                                                         : color;

        canvas.setTextSize(2);
        int lw = canvas.textWidth(ac.callsign);
        int lx = (int)px + 9;
        int ly = (int)py + 7;
        if (lx + lw > CENTER_X + RADIUS) {
            lx = (int)px - 9 - lw;  // would run off the plot - put it on the left
        }
        bool clear = true;
        for (int j = 0; j < takenCount && flash == AlertFlash::NONE; j++) {
            const LabelBox &b = taken[j];
            if (lx - LABEL_PAD < b.x + b.w && lx + lw + LABEL_PAD > b.x && ly - LABEL_PAD < b.y + b.h &&
                ly + LABEL_H + LABEL_PAD > b.y) {
                clear = false;
                break;
            }
        }
        if (clear && (takenCount < MAX_LABELS || flash != AlertFlash::NONE)) {
            canvas.setTextColor(labelColor);
            canvas.setTextDatum(TL_DATUM);
            canvas.drawString(ac.callsign, lx, ly);
            if (takenCount < MAX_LABELS) {
                taken[takenCount++] = {(int16_t)lx, (int16_t)ly, (int16_t)lw, (int16_t)LABEL_H};
            }
            labelled++;
        }
    }

    canvas.clearClipRect();

    // --- footings ---------------------------------------------------------
    // Stacked in the bottom right rather than run along the bottom left: the
    // plot is still 165px wide at the footer's height, and a legend appended
    // to the range line reached x=664, well inside it. Boxed from the right
    // edge, every one of them starts beyond the widest part of the circle at
    // the height it sits at.
    // The range readout doesn't change while the screen is up, but the plot
    // square cuts across it; on a refresh only the part inside is redrawn.
    if (!full) {
        canvas.setClipRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);
    }
    footerBox(String(rangeNm) + " nm range   rings every " + String(step) + " nm", 16, FOOTER_Y, false);
    canvas.clearClipRect();
    footerBox("^ climb   v descent", 1264, FOOTER_Y - LEGEND_OFFSET, true);
    footerBox(String(plotted) + " contacts" + (labelled < plotted ? "   " + String(labelled) + " labelled" : ""), 1264,
              FOOTER_Y, true);

    if (full) {
        return;  // the caller flushes, once it has drawn the rest of the screen
    }
    if (!scene.push) {
        // Marked rather than pushed: the plot was by its fill, the footer
        // wasn't, and whoever pushes next has to take both.
        screen::markDirty(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T);
        return;
    }
    // Two flushes rather than one: marked together, flush() would judge the
    // two regions close enough to send as their bounding box, which takes in
    // the empty margin between the plot and the right edge.
    screen::flush();  // the plot, marked by its fill
    screen::markDirty(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T);
    screen::flush();
}

RadarAction radarScreenHandleTouch(int x, int y, String &outHex) {
    if (y >= BACK_Y && y < BACK_Y + BACK_H) {
        if (x >= BACK_X && x < BACK_X + BACK_W) {
            return RadarAction::BACK;
        }
        if (x >= CENTRE_X && x < CENTRE_X + CENTRE_W) {
            return RadarAction::TOGGLE_CENTRE;
        }
        if (x >= REC_X && x < REC_X + REC_W) {
            return RadarAction::TOGGLE_RECORD;
        }
        if (x >= REPLAY_X && x < REPLAY_X + REPLAY_W) {
            return RadarAction::OPEN_RECORDINGS;
        }
    }
    return radarPlotHit(x, y, outHex) ? RadarAction::SELECT : RadarAction::NONE;
}

bool radarPlotHit(int x, int y, String &outHex) {
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
        return true;
    }
    return false;
}
