#include "radar_screen.h"

#include <M5Unified.h>
#include <algorithm>
#include <math.h>

#include "airports.h"
#include "alerts.h"
#include "runways.h"
#include "screen.h"

namespace {

bool colorsReady = false;

// The buttons stand in a column down the right-hand edge, in the margin
// beside the plot, grouped by what they do: Back; the centre and the
// airports, which are one choice of three; follow me; then recording. Each is its own button doing one
// thing on a tap - there is no long press anywhere on the screen.
constexpr int BTN_W = 150, BTN_H = 50;
constexpr int BTN_X = 1280 - 16 - BTN_W;
constexpr int BTN_GAP = 10, GROUP_GAP = 26;
enum Button { BTN_BACK, BTN_HOME, BTN_AIRPORT, BTN_ALL, BTN_FOLLOW, BTN_REC, BTN_REPLAYS, BTN_COUNT };
constexpr int BACK_Y = 8;
constexpr int HOME_Y = BACK_Y + BTN_H + GROUP_GAP;
constexpr int AIRPORT_Y = HOME_Y + BTN_H + BTN_GAP;
constexpr int ALL_Y = AIRPORT_Y + BTN_H + BTN_GAP;
constexpr int FOLLOW_Y = ALL_Y + BTN_H + GROUP_GAP;
constexpr int REC_Y = FOLLOW_Y + BTN_H + GROUP_GAP;
constexpr int REPLAY_Y = REC_Y + BTN_H + BTN_GAP;
constexpr int BTN_Y[BTN_COUNT] = {BACK_Y, HOME_Y, AIRPORT_Y, ALL_Y, FOLLOW_Y, REC_Y, REPLAY_Y};
// A tap counts for a button from anywhere in the margin to its left and
// halfway into the gaps around it, so one that lands a little off still
// gets it rather than nothing.
constexpr int BTN_REACH_X = BTN_X - 24;
// The title's band, above the plot and left of the column, repainted when
// following starts or stops.
constexpr int TITLE_H = 56;
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
constexpr int RANGE_BOX_T = FOOTER_Y - BOX_PAD_Y;  // the range readout, bottom left

constexpr int MAX_LABELS = 64;
constexpr int LABEL_H = 16;  // Font0 at size 2
constexpr int LABEL_PAD = 4;

// Where the centre's airport code goes, under the cross.
constexpr int CENTRE_CODE_Y = CENTER_Y + 13;

// Around 25 airports fall inside the widest range over the busiest parts of
// the table (London, New York, Frankfurt), so this is never the limit.
constexpr int MAX_AIRPORT_LABELS = 48;
constexpr int AIRPORT_R = 5, AIRPORT_TICK = 3;

struct LabelBox {
    int16_t x, y, w, h;
};

bool overlapsAny(const LabelBox *boxes, int count, int x, int y, int w, int h) {
    for (int j = 0; j < count; j++) {
        const LabelBox &b = boxes[j];
        if (x - LABEL_PAD < b.x + b.w && x + w + LABEL_PAD > b.x && y - LABEL_PAD < b.y + b.h &&
            y + h + LABEL_PAD > b.y) {
            return true;
        }
    }
    return false;
}

// Where each airport was last plotted, so a tap on its code or its symbol
// can open its zoom - by way of the chooser below where a contact is within
// reach too, as its own ground traffic always is.
struct AirportHit {
    int16_t x, y;    // the symbol
    LabelBox label;  // where its code went, or w 0 where it was left off
    char code[4];
};
constexpr int AIRPORT_TAP_R = 22;
constexpr int AIRPORT_LABEL_PAD = 10;
AirportHit g_airportHits[MAX_AIRPORT_LABELS + 1];  // and the centre's
int g_airportHitCount = 0;

void noteAirportHit(const String &code, int x, int y, const LabelBox &label) {
    if (g_airportHitCount < MAX_AIRPORT_LABELS + 1) {
        AirportHit &h = g_airportHits[g_airportHitCount++];
        h.x = (int16_t)x;
        h.y = (int16_t)y;
        h.label = label;
        strlcpy(h.code, code.c_str(), sizeof(h.code));
    }
}

// Where each contact was last plotted, so a tap can be matched back to one.
struct Blip {
    int16_t x, y;
    String hex;
    String label;  // what the chooser lists it as: callsign, type, altitude
    bool ground;
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
    colorHome, colorTrail, colorAlert, colorEmergency, colorRec, colorRunway;
RadarPalette g_palette;

// Where a plot's drawing time goes, for the [perf] line: the parts that
// don't move from one frame to the next - erasing, the scope, the airports
// and runways - then the contacts, then pushing it to the panel.
uint64_t g_backgroundUs = 0, g_contactsUs = 0, g_pushUs = 0;
uint32_t g_plotFrames = 0;

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
    // Greyed toward pavement, so a runway reads as ground under the traffic
    // rather than as a mark of the same kind as a contact.
    colorRunway = M5.Display.color565(0x4E, 0x6E, 0x5C);
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
    if (rangeNm <= 5) return 1;  // following one down to an airport
    if (rangeNm <= 15) return 5;
    if (rangeNm <= 30) return 10;
    if (rangeNm <= 60) return 20;
    return 50;
}

// A distance as a ring label: whole miles bare, the zoom's half miles to one
// decimal place.
String nmText(float nm) {
    return nm == floorf(nm) ? String((int)nm) : String(nm, 1);
}

// The scope itself: a ring every stepNm with its distance on it, the outer
// ring doubled, the compass points just inside it and a tick every 30 degrees.
void drawScope(float rangeNm, float stepNm) {
    auto &canvas = screen::canvas();
    canvas.setTextSize(2);
    canvas.setTextColor(colorRingText);
    canvas.setTextDatum(BC_DATUM);
    for (int i = 1; i * stepNm < rangeNm - 0.001f; i++) {
        float nm = i * stepNm;
        int r = (int)lroundf(nm / rangeNm * RADIUS);
        canvas.drawCircle(CENTER_X, CENTER_Y, r, colorRing);
        canvas.drawString(nmText(nm), CENTER_X, CENTER_Y - r - 2);
    }
    canvas.drawCircle(CENTER_X, CENTER_Y, RADIUS, colorRing);
    canvas.drawCircle(CENTER_X, CENTER_Y, RADIUS - 1, colorRing);
    canvas.drawString(nmText(rangeNm), CENTER_X, CENTER_Y - RADIUS - 2);

    static const char *POINTS[] = {"N", "E", "S", "W"};
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

// A chart's airfield: a ring with four ticks, so it can't be read as a blip.
void drawAirfield(int x, int y, uint16_t color) {
    auto &canvas = screen::canvas();
    canvas.drawCircle(x, y, AIRPORT_R, color);
    constexpr int T0 = AIRPORT_R + 1, T1 = AIRPORT_R + AIRPORT_TICK;
    canvas.drawLine(x, y - T0, x, y - T1, color);
    canvas.drawLine(x, y + T0, x, y + T1, color);
    canvas.drawLine(x - T0, y, x - T1, y, color);
    canvas.drawLine(x + T0, y, x + T1, y, color);
}

// The other airports in range, as map rather than traffic: drawn before
// anything that moves, in the shade the compass points use. The symbol is a
// chart's airfield - a ring with four ticks - so it can't be read as a blip,
// and the code goes under it, as the centre's does. Codes are placed nearest
// first and one that would overlap another is left off, keeping the symbol,
// so a cluster of airfields still shows as one. Contacts' callsigns don't
// check against them: the traffic is what the plot is for.
//
// `originCode`, while following a departure, is the airport it took off from:
// drawn brighter and always labelled, so where it came from stays on show.
void drawAirports(double lat, double lon, int rangeNm, const String &centreCode, const String &originCode) {
    auto &canvas = screen::canvas();
    double nmPerDegLon = 60.0 * cos(lat * M_PI / 180.0);
    canvas.setTextSize(2);
    canvas.setTextColor(colorFaint);
    canvas.setTextDatum(TL_DATUM);
    LabelBox taken[MAX_AIRPORT_LABELS];
    int takenCount = 0;
    if (centreCode.length()) {
        int w = canvas.textWidth(centreCode);
        taken[takenCount++] = {(int16_t)(CENTER_X - w / 2), (int16_t)CENTRE_CODE_Y, (int16_t)w, (int16_t)LABEL_H};
    }
    for (const Airport &a : airportsWithin(lat, lon, rangeNm)) {
        if (a.code == centreCode) {
            continue;  // the cross already marks it
        }
        int x = CENTER_X + (int)lroundf((float)((a.lon - lon) * nmPerDegLon / rangeNm * RADIUS));
        int y = CENTER_Y - (int)lroundf((float)((a.lat - lat) * 60.0 / rangeNm * RADIUS));
        bool origin = originCode.length() && a.code == originCode;
        drawAirfield(x, y, origin ? colorHome : colorFaint);
        constexpr int T1 = AIRPORT_R + AIRPORT_TICK;

        // Under the symbol unless that runs off the bottom of the plot, and
        // kept inside its sides: one due east or west at the very edge of the
        // range would otherwise lose a character to the clip.
        int w = canvas.textWidth(a.code);
        int lx = std::min(std::max(x - w / 2, PLOT_L), PLOT_R - w);
        int ly = y + T1 + 3;
        if (ly + LABEL_H > PLOT_B) {
            ly = y - T1 - 3 - LABEL_H;
        }
        LabelBox label = {(int16_t)lx, (int16_t)ly, 0, (int16_t)LABEL_H};
        if (origin || (takenCount < MAX_AIRPORT_LABELS && !overlapsAny(taken, takenCount, lx, ly, w, LABEL_H))) {
            canvas.setTextColor(origin ? colorHome : colorFaint);
            canvas.drawString(a.code, lx, ly);
            label.w = (int16_t)w;
            taken[takenCount++] = label;
        }
        noteAirportHit(a.code, x, y, label);
    }
}

// A button's three looks: an ordinary one filled; a lit one - the centre in
// use, the airports on, following - filled brighter with dark text, so the
// column reads as a set of switches at a glance; and one with nothing behind
// it outlined, its label in grey, so it reads as there but unavailable.
enum class Look { PLAIN, LIT, GREYED };

void drawButtonFrame(int y, Look look) {
    auto &canvas = screen::canvas();
    canvas.fillRect(BTN_X, y, BTN_W, BTN_H, colorBg);
    if (look == Look::GREYED) {
        canvas.drawRoundRect(BTN_X, y, BTN_W, BTN_H, 6, colorDisabled);
    } else {
        canvas.fillRoundRect(BTN_X, y, BTN_W, BTN_H, 6, look == Look::LIT ? colorMuted : colorBtnBg);
    }
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(look == Look::GREYED ? colorDisabled : look == Look::LIT ? colorBg : colorText);
    screen::markDirty(BTN_X, y, BTN_W, BTN_H);
}

void drawButton(int y, const char *label, Look look) {
    drawButtonFrame(y, look);
    screen::canvas().drawString(label, BTN_X + BTN_W / 2, y + BTN_H / 2);
}

void drawReplaysButton() {
    drawButton(REPLAY_Y, "Replays", g_recState == RecButton::NO_CARD ? Look::GREYED : Look::PLAIN);
}

// Idle, it is a red dot and REC, the way every recorder has said it. Running,
// it turns red and counts, with a stop square: what a tap will now do.
void drawRecButton() {
    if (g_recState == RecButton::NO_CARD) {
        drawButton(REC_Y, "NO SD", Look::GREYED);
        return;
    }
    auto &canvas = screen::canvas();
    bool running = (g_recState == RecButton::RECORDING);
    drawButtonFrame(REC_Y, Look::PLAIN);
    if (running) {
        canvas.fillRoundRect(BTN_X, REC_Y, BTN_W, BTN_H, 6, colorRec);
    }
    int midY = REC_Y + BTN_H / 2;
    canvas.setTextColor(colorText);
    if (running) {
        canvas.fillRect(BTN_X + 20, midY - 6, 12, 12, colorText);
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
        canvas.drawString(buf, BTN_X + 88, midY);
    } else {
        canvas.fillCircle(BTN_X + 46, midY, 7, colorEmergency);
        canvas.drawString("REC", BTN_X + 86, midY);
    }
}

// --- the controls -------------------------------------------------------------

RadarCentre g_centre = DEFAULT_RADAR_CENTRE;
bool g_airportsOn = false;
FollowButton g_follow = FollowButton::OFF;
String g_followHex, g_followCallsign;
String g_followOrigin;  // a followed departure's airport
std::vector<FollowSample> g_heights;  // the followed aircraft's, for its telemetry
uint32_t g_lostS = 0;

// Where the live radar and zoom put the telemetry: the margin left of the
// plot, under the title, which is otherwise empty.
constexpr int LIVE_TEL_X = 16, LIVE_TEL_Y = 80;

void drawCentreButtons() {
    bool following = (g_follow == FollowButton::ON);
    bool home = !following && g_centre == RadarCentre::HOME;
    bool airport = !following && g_centre == RadarCentre::AIRPORT;
    drawButton(HOME_Y, "HOME", home ? Look::LIT : Look::PLAIN);
    drawButton(AIRPORT_Y, "AIRPORT", airport && !g_airportsOn ? Look::LIT : Look::PLAIN);
    drawButton(ALL_Y, "ALL", airport && g_airportsOn ? Look::LIT : Look::PLAIN);
}

void drawFollowButton() {
    drawButton(FOLLOW_Y, g_follow == FollowButton::PICKING ? "TAP PLANE" : "FOLLOW",
               g_follow == FollowButton::OFF ? Look::PLAIN : Look::LIT);
}

// "Radar - CIV -", and then what following is doing: asking for an aircraft
// to be tapped, or naming the one followed.
void drawTitle(bool military) {
    auto &canvas = screen::canvas();
    canvas.fillRect(0, 0, BTN_REACH_X, TITLE_H, colorBg);
    // Set here rather than trusted from whatever drew last: the table leaves
    // its cells' FreeSans selected, and at size 3 the title came out huge.
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(colorText);
    const char *title = "Radar";
    int x = 16;
    canvas.drawString(title, x, 35);
    x += canvas.textWidth(title);
    canvas.setTextColor(colorMuted);
    String sub = military ? " - MIL -" : " - CIV -";
    canvas.drawString(sub, x, 35);
    x += canvas.textWidth(sub);
    if (g_follow == FollowButton::PICKING) {
        canvas.drawString("  tap a plane to follow", x, 35);
    } else if (g_follow == FollowButton::ON) {
        const char *what = "  following ";
        canvas.drawString(what, x, 35);
        canvas.setTextColor(colorHome);
        canvas.drawString(g_followCallsign, x + canvas.textWidth(what), 35);
    }
    screen::markDirty(0, 0, BTN_REACH_X, TITLE_H);
}

bool g_military = false;

// Title and buttons: drawn on a full repaint, and again in part as the
// controls change.
void drawHeader(bool military) {
    g_military = military;
    drawTitle(military);
    drawButton(BACK_Y, "Back", Look::PLAIN);
    drawCentreButtons();
    drawFollowButton();
    drawRecButton();
    drawReplaysButton();
}

// Which button a tap at (x, y) is for, or BTN_COUNT for none.
int buttonAt(int x, int y) {
    if (x < BTN_REACH_X) {
        return BTN_COUNT;
    }
    for (int i = 0; i < BTN_COUNT; i++) {
        if (y >= BTN_Y[i] - BTN_GAP / 2 && y < BTN_Y[i] + BTN_H + BTN_GAP / 2) {
            return i;
        }
    }
    return BTN_COUNT;
}

// --- choosing between targets ------------------------------------------------
// A tap within reach of more than one thing - an airport under its own ground
// traffic, or a cluster of contacts - brings up a list of them beside it to
// pick from, rather than the nearest being guessed at. Anywhere outside the
// list, or Cancel, closes it. It stays up over the plot's refreshes, and a
// full draw - arriving at the screen - starts without one.

constexpr int MAX_CHOICES = 6;
constexpr int CHOICE_W = 400;
constexpr int CHOICE_PAD = 12, CHOICE_TITLE_H = 32, CHOICE_ROW_H = 52, CHOICE_GAP = 8;
constexpr int CHOICE_OFFSET = 40;  // from the tap to the near side of the list

struct Choice {
    bool airport;
    String id;  // the airport's IATA code, or the contact's ICAO hex
    String text;
    bool ground;
};
Choice g_choices[MAX_CHOICES];
int g_choiceCount = 0;  // 0 while no list is up
int g_choiceX = 0, g_choiceY = 0, g_choiceH = 0;

int choiceRowY(int i) { return g_choiceY + CHOICE_PAD + CHOICE_TITLE_H + i * (CHOICE_ROW_H + CHOICE_GAP); }

// The list, with Cancel as its last row, over whatever is drawn under it.
void drawChooser() {
    auto &canvas = screen::canvas();
    canvas.clearClipRect();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.fillRoundRect(g_choiceX, g_choiceY, CHOICE_W, g_choiceH, 10, colorBg);
    canvas.drawRoundRect(g_choiceX, g_choiceY, CHOICE_W, g_choiceH, 10, colorRing);
    canvas.drawRoundRect(g_choiceX + 1, g_choiceY + 1, CHOICE_W - 2, g_choiceH - 2, 9, colorRing);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(colorMuted);
    canvas.drawString("Which one?", g_choiceX + CHOICE_PAD + 4, g_choiceY + CHOICE_PAD + CHOICE_TITLE_H / 2 - 4);

    int rowX = g_choiceX + CHOICE_PAD, rowW = CHOICE_W - 2 * CHOICE_PAD;
    for (int i = 0; i < g_choiceCount; i++) {
        const Choice &c = g_choices[i];
        int y = choiceRowY(i), midY = y + CHOICE_ROW_H / 2;
        canvas.fillRoundRect(rowX, y, rowW, CHOICE_ROW_H, 6, colorBtnBg);
        // The mark it has on the plot, so the row reads as the thing tapped.
        int iconX = rowX + 24;
        if (c.airport) {
            drawAirfield(iconX, midY, colorText);
        } else if (c.ground) {
            canvas.drawCircle(iconX, midY, 5, colorBlip);
            canvas.drawCircle(iconX, midY, 4, colorBlip);
        } else {
            canvas.fillCircle(iconX, midY, 5, colorBlip);
        }
        canvas.setTextDatum(ML_DATUM);
        canvas.setTextColor(colorText);
        canvas.drawString(c.text, rowX + 48, midY);
    }
    int y = choiceRowY(g_choiceCount);
    canvas.drawRoundRect(rowX, y, rowW, CHOICE_ROW_H, 6, colorRing);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(colorMuted);
    canvas.drawString("Cancel", rowX + rowW / 2, y + CHOICE_ROW_H / 2);
    screen::markDirty(g_choiceX, g_choiceY, CHOICE_W, g_choiceH);
}

// Puts the list up beside a tap at (x, y): to its right where there's room,
// otherwise its left, level with it as far as the screen allows - and never
// over the buttons, which repaint on their own: REC's count, every second
// while recording, painted over a list that reached across the column.
void openChooser(int x, int y) {
    constexpr int RIGHT = BTN_X - 16;
    g_choiceH = 2 * CHOICE_PAD + CHOICE_TITLE_H + (g_choiceCount + 1) * (CHOICE_ROW_H + CHOICE_GAP) - CHOICE_GAP;
    g_choiceX = (x + CHOICE_OFFSET + CHOICE_W <= RIGHT) ? x + CHOICE_OFFSET : x - CHOICE_OFFSET - CHOICE_W;
    g_choiceX = std::min(std::max(g_choiceX, 16), RIGHT - CHOICE_W);
    g_choiceY = std::min(std::max(y - g_choiceH / 2, PLOT_T + 4), 720 - 8 - g_choiceH);
    drawChooser();
    screen::flush();
}

// Everything within reach of a tap at (x, y) on the last plot drawn, into
// g_choices: the airports first, since an airport is what a crowd of its own
// traffic hides, then the contacts, each nearest first. Returns how many.
int collectChoices(int x, int y, bool withAirports) {
    std::vector<std::pair<long, int>> airports, blips;
    if (withAirports) {
        for (int i = 0; i < g_airportHitCount; i++) {
            const AirportHit &h = g_airportHits[i];
            long dx = x - h.x, dy = y - h.y;
            long d = dx * dx + dy * dy;
            const LabelBox &b = h.label;
            bool onLabel = b.w && x >= b.x - AIRPORT_LABEL_PAD && x < b.x + b.w + AIRPORT_LABEL_PAD &&
                           y >= b.y - AIRPORT_LABEL_PAD && y < b.y + b.h + AIRPORT_LABEL_PAD;
            if (onLabel || d <= (long)AIRPORT_TAP_R * AIRPORT_TAP_R) {
                airports.push_back({d, i});
            }
        }
    }
    for (int i = 0; i < g_blipCount; i++) {
        long dx = x - g_blips[i].x, dy = y - g_blips[i].y;
        long d = dx * dx + dy * dy;
        if (d <= (long)TAP_RADIUS * TAP_RADIUS) {
            blips.push_back({d, i});
        }
    }
    std::sort(airports.begin(), airports.end());
    std::sort(blips.begin(), blips.end());
    int n = 0;
    for (const auto &a : airports) {
        if (n < MAX_CHOICES) {
            const AirportHit &h = g_airportHits[a.second];
            g_choices[n++] = {true, String(h.code), String("Zoom in on ") + h.code, false};
        }
    }
    for (const auto &b : blips) {
        if (n < MAX_CHOICES) {
            const Blip &bl = g_blips[b.second];
            g_choices[n++] = {false, bl.hex, bl.label, bl.ground};
        }
    }
    return n;
}

enum class Target { NONE, AIRCRAFT, AIRPORT, DISMISSED };

// What a tap on the plot is for. With a list up, it picks from the list -
// or, anywhere else, dismisses it. Otherwise it is the one target within
// reach, or NONE: either nothing is, or several are and the list has gone up.
Target tapTarget(int x, int y, bool withAirports, String &outId) {
    if (g_choiceCount) {
        bool inside = x >= g_choiceX && x < g_choiceX + CHOICE_W && y >= g_choiceY && y < g_choiceY + g_choiceH;
        if (!inside) {
            g_choiceCount = 0;
            return Target::DISMISSED;
        }
        for (int i = 0; i <= g_choiceCount; i++) {
            int top = choiceRowY(i) - CHOICE_GAP / 2;
            if (y >= top && y < top + CHOICE_ROW_H + CHOICE_GAP) {
                if (i == g_choiceCount) {
                    g_choiceCount = 0;  // Cancel
                    return Target::DISMISSED;
                }
                Choice c = g_choices[i];
                g_choiceCount = 0;
                outId = c.id;
                return c.airport ? Target::AIRPORT : Target::AIRCRAFT;
            }
        }
        return Target::NONE;  // the list's title or frame
    }
    int n = collectChoices(x, y, withAirports);
    if (n == 1) {
        outId = g_choices[0].id;
        return g_choices[0].airport ? Target::AIRPORT : Target::AIRCRAFT;
    }
    if (n > 1) {
        g_choiceCount = n;
        openChooser(x, y);
    }
    return Target::NONE;
}

}  // namespace

void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool military, bool full) {
    ensureColors();
    if (full) {
        g_choiceCount = 0;
        screen::clear(colorBg);
        drawHeader(military);
    }
    // With a list up, a refresh is drawn under it and pushed along with it.
    bool following = (g_follow == FollowButton::ON);
    radarPlotDraw({aircraft, isNew, nullptr, lat, lon, rangeNm, following ? RadarCentre::HOME : g_centre, g_airportsOn,
                   true, g_choiceCount == 0, following ? g_followHex : String(),
                   following ? g_followOrigin : String()},
                  full);
    if (following) {
        radarTelemetryDraw(LIVE_TEL_X, LIVE_TEL_Y, g_heights, millis(), INT32_MIN, g_lostS);
    }
    if (g_choiceCount) {
        drawChooser();
    }
    // Following, the telemetry goes in a push of its own: with the plot's,
    // flush() would send the margin between them too.
    if (full || g_choiceCount || following) {
        screen::flush();
    }
}

void radarScreenSetRecording(RecButton state, uint32_t elapsedS, bool onScreen, bool zoom) {
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
        if (stateChanged && !zoom) {
            drawReplaysButton();
        }
        drawRecButton();
        screen::flush();
    }
}

void radarScreenSetControls(RadarCentre centre, bool airports, FollowButton follow, const String &followHex,
                            const String &followCallsign, const String &followOrigin, bool onScreen) {
    g_followOrigin = followOrigin;
    // Following unlights the centre, so starting or stopping it changes those too.
    bool wasFollowing = (g_follow == FollowButton::ON), following = (follow == FollowButton::ON);
    bool centreChanged = (centre != g_centre || airports != g_airportsOn || following != wasFollowing);
    bool followChanged = (follow != g_follow || followCallsign != g_followCallsign);
    g_centre = centre;
    g_airportsOn = airports;
    g_follow = follow;
    g_followHex = followHex;
    g_followCallsign = followCallsign;
    if (!onScreen || (!centreChanged && !followChanged)) {
        return;
    }
    ensureColors();
    if (centreChanged) {
        drawCentreButtons();
    }
    if (followChanged) {
        drawFollowButton();
        drawTitle(g_military);
    }
    screen::flush();
}

const RadarPalette &radarPalette() {
    ensureColors();
    return g_palette;
}

String radarTakeTimings() {
    if (g_plotFrames == 0) {
        return String("-");
    }
    String s = "background " + String((uint32_t)(g_backgroundUs / g_plotFrames / 1000)) + ", contacts " +
               String((uint32_t)(g_contactsUs / g_plotFrames / 1000)) + ", push " +
               String((uint32_t)(g_pushUs / g_plotFrames / 1000)) + "ms avg";
    g_backgroundUs = g_contactsUs = g_pushUs = 0;
    g_plotFrames = 0;
    return s;
}

namespace {

// How drawContacts() places and draws what it is given: the radar's and the
// airport zoom's plots differ only in these.
struct ContactView {
    double lat, lon;  // the point at the plot's centre
    float pxPerNm;
    float leadS;  // how far ahead a contact's vector reaches
    // Ground traffic as a ring rather than a dot, and the altitude under each
    // callsign: at an airport, which contacts are down and how high the rest
    // are is most of what there is to see.
    bool zoomed;
    const String &followHex;  // ringed, and always labelled; empty for none
};

// Trails, then contacts with their vectors and callsigns, each one noted for
// radarPlotHit(). Inside the plot's clip, which the caller sets.
void drawContacts(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew,
                  const std::vector<replay::TrailPoint> *trails, bool flashing, const ContactView &v,
                  int &plotted, int &labelled) {
    auto &canvas = screen::canvas();
    // Flat-earth projection: over a 150nm radius the error is far smaller than
    // a blip, and it keeps this to two multiplications per aircraft.
    double nmPerDegLon = 60.0 * cos(v.lat * M_PI / 180.0);
    plotted = 0;
    labelled = 0;
    LabelBox taken[MAX_LABELS];
    int takenCount = 0;
    g_blipCount = 0;

    // Trails first, so every blip sits on top of its own and anyone else's.
    if (trails != nullptr) {
        bool penDown = false;
        int lastX = 0, lastY = 0;
        for (const replay::TrailPoint &p : *trails) {
            if (isnan(p.lat)) {
                penDown = false;
                continue;
            }
            int x = CENTER_X + (int)lroundf((float)((p.lon - v.lon) * nmPerDegLon) * v.pxPerNm);
            int y = CENTER_Y - (int)lroundf((float)((p.lat - v.lat) * 60.0) * v.pxPerNm);
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
        double dNorth = (ac.lat - v.lat) * 60.0;
        double dEast = (ac.lon - v.lon) * nmPerDegLon;
        float px = CENTER_X + (float)dEast * v.pxPerNm;
        float py = CENTER_Y - (float)dNorth * v.pxPerNm;

        // anything beyond the outer ring would be drawn outside the plot
        float dx = px - CENTER_X, dy = py - CENTER_Y;
        if (dx * dx + dy * dy > (float)RADIUS * RADIUS) {
            continue;
        }
        plotted++;

        bool isNewHere = (i < isNew.size()) && isNew[i];
        // A position the feed has lost, kept from the last it had, dimmed to
        // the trails' shade: where it was, as near as anyone can say.
        uint16_t color = ac.posStale                  ? colorTrail
                         : (ac.alert & ALERT_EMERGENCY) ? colorEmergency
                         : ac.alert                     ? colorAlert
                         : isNewHere                    ? colorNewBlip
                                                        : colorBlip;

        if (ac.hasTrack) {
            // How far it will fly in the lead time at its current groundspeed,
            // with a floor so a slow or speed-less contact still shows which
            // way it's pointing.
            float gs = ac.speedStr.toFloat();
            float len = gs * v.leadS / 3600.0f * v.pxPerNm;
            if (len < 14.0f) {
                len = 14.0f;
            }
            float a = ac.track * (float)M_PI / 180.0f;
            canvas.drawLine((int)px, (int)py, (int)(px + sinf(a) * len), (int)(py - cosf(a) * len), color);
        }
        bool onGround = (ac.status == "GROUND" || ac.status == "TAXI");
        if (v.zoomed && onGround) {
            canvas.fillCircle((int)px, (int)py, 5, colorBg);  // hollow, over its own vector
            canvas.drawCircle((int)px, (int)py, 5, color);
            canvas.drawCircle((int)px, (int)py, 4, color);
        } else {
            canvas.fillCircle((int)px, (int)py, 5, color);
        }
        drawTrend((int)px, (int)py, ac.status, color);
        bool followed = v.followHex.length() && ac.hex == v.followHex;
        if (followed) {
            // Clear of the trend chevrons, which reach 15px above and below.
            canvas.drawCircle((int)px, (int)py, 17, colorHome);
            canvas.drawCircle((int)px, (int)py, 18, colorHome);
        }
        if (g_blipCount < MAX_BLIPS) {
            String label = ac.callsign;
            if (ac.type.length() && ac.type != "----") {
                label += "  " + ac.type;
            }
            if (ac.altStr != "?") {
                label += "  " + (onGround ? String("GND") : ac.altStr + "ft");
            }
            g_blips[g_blipCount++] = {(int16_t)px, (int16_t)py, ac.hex, label, onGround};
        }

        // The label flashes between red and an ordinary contact's colour. A
        // flashing one is drawn even where it would collide with another
        // label: it's the one being drawn attention to - as is the one being
        // followed.
        AlertFlash flash = flashing ? alertFlash(ac.hex) : AlertFlash::NONE;
        uint16_t labelColor = flash == AlertFlash::RED ? colorEmergency
                              : flash == AlertFlash::OFF ? (isNewHere ? colorNewBlip : colorBlip)
                              : ac.posStale              ? colorFaint
                                                         : color;

        String sub;
        if (v.zoomed && ac.altStr != "?") {
            sub = onGround ? String("GND") : ac.altStr + "ft";
        }
        canvas.setTextSize(2);
        int lw = std::max(canvas.textWidth(ac.callsign), sub.length() ? canvas.textWidth(sub) : 0);
        int lh = sub.length() ? 2 * LABEL_H + 2 : LABEL_H;
        // The followed one's goes outside its ring.
        int lx = (int)px + (followed ? 21 : 9);
        int ly = (int)py + (followed ? 12 : 7);
        if (lx + lw > CENTER_X + RADIUS) {
            lx = (int)px - (followed ? 21 : 9) - lw;  // would run off the plot - put it on the left
        }
        bool forced = followed || flash != AlertFlash::NONE;
        bool clear = forced || !overlapsAny(taken, takenCount, lx, ly, lw, lh);
        if (clear && (takenCount < MAX_LABELS || forced)) {
            canvas.setTextColor(labelColor);
            canvas.setTextDatum(TL_DATUM);
            canvas.drawString(ac.callsign, lx, ly);
            if (sub.length()) {
                // Dimmer than the callsign, unless the contact is alerted.
                canvas.setTextColor(color == colorBlip || color == colorNewBlip ? colorFaint : color);
                canvas.drawString(sub, lx, ly + LABEL_H + 2);
            }
            if (takenCount < MAX_LABELS) {
                taken[takenCount++] = {(int16_t)lx, (int16_t)ly, (int16_t)lw, (int16_t)lh};
            }
            labelled++;
        }
    }
}

// Pushes a plot redrawn on a refresh: the plot square, marked by its fill,
// and the footer readouts in (FOOT_L, footTop) to the bottom right.
void pushPlot(bool push, int footTop) {
    if (!push) {
        // Marked rather than pushed: the plot was by its fill, the footer
        // wasn't, and whoever pushes next has to take both.
        screen::markDirty(FOOT_L, footTop, FOOT_R - FOOT_L, FOOT_B - footTop);
        screen::markDirty(0, RANGE_BOX_T, PLOT_L, BOX_H);
        return;
    }
    // Two flushes rather than one: marked together, flush() would judge the
    // two regions close enough to send as their bounding box, which takes in
    // the empty margin between the plot and the right edge. The range
    // readout's end in the left margin goes with the footer, a strip too thin
    // beside it to be merged into a box with it.
    screen::flush();
    screen::markDirty(FOOT_L, footTop, FOOT_R - FOOT_L, FOOT_B - footTop);
    screen::markDirty(0, RANGE_BOX_T, PLOT_L, BOX_H);
    screen::flush();
}

}  // namespace

void radarPlotDraw(const RadarScene &scene, bool full) {
    ensureColors();
    uint32_t t0 = micros();
    auto &canvas = screen::canvas();
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
    drawScope((float)rangeNm, (float)step);

    // Centred on the airport, the cross is tagged with its code so the centre
    // says where it is; centred on home, it is a bare cross. Airport mode with
    // nothing in range of the lookup falls back to home, since there is no
    // other point to centre on.
    g_airportHitCount = 0;
    Airport airport;
    String centreCode;
    bool following = scene.followHex.length() > 0;
    if (!following && centre == RadarCentre::AIRPORT && nearestAirport(lat, lon, airport)) {
        lat = airport.lat;
        lon = airport.lon;
        centreCode = airport.code;
        canvas.setTextSize(2);
        canvas.setTextColor(colorHome);
        canvas.setTextDatum(TC_DATUM);
        canvas.drawString(airport.code, CENTER_X, CENTRE_CODE_Y);
        int w = canvas.textWidth(airport.code);
        noteAirportHit(airport.code, CENTER_X, CENTER_Y,
                       {(int16_t)(CENTER_X - w / 2), (int16_t)CENTRE_CODE_Y, (int16_t)w, (int16_t)LABEL_H});
    }
    // Following, the centre is the aircraft, ringed where it is drawn.
    if (!following) {
        canvas.drawLine(CENTER_X - 9, CENTER_Y, CENTER_X + 9, CENTER_Y, colorHome);
        canvas.drawLine(CENTER_X, CENTER_Y - 9, CENTER_X, CENTER_Y + 9, colorHome);
    }

    // --- contacts ---------------------------------------------------------
    canvas.setClipRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);

    // Part of the airport view, so centred on home they are off, as they are
    // on a replay switched to HOME. Following, they are always on, whichever
    // view it was picked from: they say where it might be going - and
    // following a departure, where it came from.
    if ((scene.airports && centreCode.length()) || following) {
        drawAirports(lat, lon, rangeNm, centreCode, following ? scene.originCode : String());
    }

    int plotted = 0, labelled = 0;
    uint32_t t1 = micros();
    drawContacts(scene.aircraft, scene.isNew, scene.trails, scene.flash,
                 {lat, lon, (float)RADIUS / rangeNm, 60.0f, false, scene.followHex}, plotted, labelled);

    canvas.clearClipRect();

    // --- footings ---------------------------------------------------------
    // Stacked in the bottom right rather than run along the bottom left: the
    // plot is still 165px wide at the footer's height, and a legend appended
    // to the range line reached x=664, well inside it. Boxed from the right
    // edge, every one of them starts beyond the widest part of the circle at
    // the height it sits at.
    // The range readout reaches from the margin into the plot square. Following
    // changes the range as it goes, so a refresh redraws all of it - the part
    // in the margin erased first, and pushed with the footer.
    if (!full) {
        canvas.fillRect(0, RANGE_BOX_T, PLOT_L, BOX_H, colorBg);
    }
    footerBox(String(rangeNm) + " nm range   rings every " + String(step) + " nm", 16, FOOTER_Y, false);
    footerBox("^ climb   v descent", 1264, FOOTER_Y - LEGEND_OFFSET, true);
    footerBox(String(plotted) + " contacts" + (labelled < plotted ? "   " + String(labelled) + " labelled" : ""), 1264,
              FOOTER_Y, true);

    uint32_t t2 = micros();
    g_backgroundUs += t1 - t0;
    g_contactsUs += t2 - t1;
    g_plotFrames++;
    if (full) {
        return;  // the caller flushes, once it has drawn the rest of the screen
    }
    pushPlot(scene.push, FOOT_T);
    g_pushUs += micros() - t2;
}

// --- the airport zoom --------------------------------------------------------

namespace {

// The range is the furthest runway end from the middle times the margin, so
// there is room around the runways for traffic on final and climbing out,
// rounded up to the half mile. Kept between a range where a runway is still
// wider than a line and one where the approach is still on the plot.
constexpr float ZOOM_MARGIN = 2.2f;
constexpr float ZOOM_MIN_NM = 2.0f, ZOOM_MAX_NM = 6.0f;
constexpr float ZOOM_NO_RUNWAYS_NM = 3.0f;
// A minute's flying would reach across the whole plot from most of it.
constexpr float ZOOM_LEAD_S = 20.0f;
constexpr int MAX_ZOOM_RUNWAYS = 16;  // O'Hare has the most, at 11 with its closed ones
// The footer stacks one box higher than the radar's, for the runways in use.
constexpr int ZOOM_FOOT_T = FOOT_T - LEGEND_OFFSET;
// Between the end of a runway and its number, and its number and its
// extended centreline.
constexpr float RUNWAY_LABEL_GAP = 5.0f;

// An end is in use while something is lined up on it going the way that end
// faces: rolling along the runway at take-off or landing speed, or in the air
// below circuit height, pointing along it and near its extended centreline -
// on either side, since departures go the same way as arrivals.
constexpr float IN_USE_ALIGN_DEG = 15.0f;
constexpr int IN_USE_MAX_AGL_FT = 2500;
constexpr float IN_USE_AIR_OFFSET_NM = 0.5f;
constexpr float IN_USE_GROUND_OFFSET_NM = 0.05f;  // about 90m: on it, not on a taxiway beside it
constexpr float IN_USE_ROLL_KT = 40.0f;

struct ZoomFrame {
    String code;
    double lat = 0, lon = 0;  // the middle of its runways, or the airport where it has none
    float rangeNm = ZOOM_NO_RUNWAYS_NM;
    const Runway *runways = nullptr;
    int runwayCount = 0;  // no more than MAX_ZOOM_RUNWAYS
};
ZoomFrame g_zoom;
String g_zoomFollowHex, g_zoomFollowCallsign;
bool g_zoomPicking = false;

// A point in a zoom's flat-earth frame, in nm east and north of its middle.
struct Vec {
    float x, y;
};

Vec toLocal(const ZoomFrame &f, float lat, float lon) {
    double nmPerDegLon = 60.0 * cos(f.lat * M_PI / 180.0);
    return {(float)((lon - f.lon) * nmPerDegLon), (float)((lat - f.lat) * 60.0)};
}

// The zoom on the airport with `code`, framed - false where the airport table
// hasn't got it.
bool frameZoom(const String &code, ZoomFrame &f) {
    Airport airport;
    if (!airportByCode(code, airport)) {
        return false;
    }
    f = ZoomFrame();
    f.code = airport.code;
    f.lat = airport.lat;
    f.lon = airport.lon;
    f.runways = runwaysAt(airport.code, f.runwayCount);
    f.runwayCount = std::min(f.runwayCount, MAX_ZOOM_RUNWAYS);
    if (f.runwayCount) {
        // The middle of the runways rather than the airport's own point, which
        // can sit well off to one side of them.
        float minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
        for (int r = 0; r < f.runwayCount; r++) {
            for (const RunwayEnd &e : f.runways[r].end) {
                minLat = std::min(minLat, e.lat);
                maxLat = std::max(maxLat, e.lat);
                minLon = std::min(minLon, e.lon);
                maxLon = std::max(maxLon, e.lon);
            }
        }
        f.lat = (minLat + maxLat) / 2.0;
        f.lon = (minLon + maxLon) / 2.0;
        float furthest = 0;
        for (int r = 0; r < f.runwayCount; r++) {
            for (const RunwayEnd &e : f.runways[r].end) {
                Vec v = toLocal(f, e.lat, e.lon);
                furthest = std::max(furthest, hypotf(v.x, v.y));
            }
        }
        float range = ceilf(furthest * ZOOM_MARGIN * 2.0f) / 2.0f;
        f.rangeNm = std::min(std::max(range, ZOOM_MIN_NM), ZOOM_MAX_NM);
    }
    return true;
}

// Which way each runway end is being used, from the contacts lined up on it.
// A contact counts for one end at most - the one whose centreline it is
// nearest - so a pair of close parallels doesn't light up together for the
// one aircraft.
void runwaysInUse(const ZoomFrame &f, const std::vector<Aircraft> &aircraft, bool inUse[][2]) {
    for (int r = 0; r < f.runwayCount; r++) {
        inUse[r][0] = inUse[r][1] = false;
    }
    for (const Aircraft &ac : aircraft) {
        if (!ac.hasPos || !ac.hasTrack || ac.altStr == "?") {
            continue;
        }
        bool onGround = (ac.status == "GROUND" || ac.status == "TAXI");
        if (onGround && ac.speedStr.toFloat() < IN_USE_ROLL_KT) {
            continue;  // taxiing, or holding short
        }
        Vec p = toLocal(f, ac.lat, ac.lon);
        int bestR = -1, bestK = 0;
        float bestOffset = 1e9f;
        for (int r = 0; r < f.runwayCount; r++) {
            const Runway &rw = f.runways[r];
            if (rw.closed || (!onGround && ac.altStr.toInt() - rw.elevationFt > IN_USE_MAX_AGL_FT)) {
                continue;
            }
            Vec ends[2] = {toLocal(f, rw.end[0].lat, rw.end[0].lon), toLocal(f, rw.end[1].lat, rw.end[1].lon)};
            float len = hypotf(ends[1].x - ends[0].x, ends[1].y - ends[0].y);
            if (len <= 0) {
                continue;
            }
            for (int k = 0; k < 2; k++) {
                // From this end towards the other: the way an aircraft using it goes.
                const Vec &from = ends[k], &to = ends[1 - k];
                float ux = (to.x - from.x) / len, uy = (to.y - from.y) / len;
                float heading = atan2f(ux, uy) * 180.0f / (float)M_PI;
                float off = fabsf(fmodf(ac.track - heading + 540.0f, 360.0f) - 180.0f);
                if (off > IN_USE_ALIGN_DEG) {
                    continue;
                }
                float rx = p.x - from.x, ry = p.y - from.y;
                float along = rx * ux + ry * uy;
                float offset = fabsf(rx * uy - ry * ux);
                bool lined = onGround ? (offset <= IN_USE_GROUND_OFFSET_NM && along >= 0 && along <= len)
                                      : offset <= IN_USE_AIR_OFFSET_NM;
                if (lined && offset < bestOffset) {
                    bestOffset = offset;
                    bestR = r;
                    bestK = k;
                }
            }
        }
        if (bestR >= 0) {
            inUse[bestR][bestK] = true;
        }
    }
}

void drawDashed(float x0, float y0, float x1, float y1, float on, float off, uint16_t color) {
    auto &canvas = screen::canvas();
    float len = hypotf(x1 - x0, y1 - y0);
    if (len < 1.0f) {
        return;
    }
    float ux = (x1 - x0) / len, uy = (y1 - y0) / len;
    for (float t = 0; t < len; t += on + off) {
        float t1 = std::min(t + on, len);
        canvas.drawLine((int)lroundf(x0 + ux * t), (int)lroundf(y0 + uy * t), (int)lroundf(x0 + ux * t1),
                        (int)lroundf(y0 + uy * t1), color);
    }
}

// How far a line from (x, y) along (ux, uy) runs before it meets the outer
// ring, or 0 if it starts outside it.
float toRing(float x, float y, float ux, float uy) {
    float dx = x - CENTER_X, dy = y - CENTER_Y;
    float r = RADIUS - 2;
    float b = dx * ux + dy * uy;
    float c = dx * dx + dy * dy - r * r;
    if (c > 0) {
        return 0;
    }
    return -b + sqrtf(b * b - c);
}

// Each runway to scale, from its two ends and its width, with the number of
// each end just beyond it, the way an airport diagram has them. Its
// centreline runs on from beyond each number to the edge of the plot, as the
// approach to that end - brighter where that end is in use. All the lines go
// down first, so none crosses a runway.
//
// A closed runway is drawn as a chart draws one: outlined rather than filled
// and crossed out, without numbers or an approach - and under the open ones,
// which it often crosses.
void drawRunways(const ZoomFrame &f, const bool inUse[][2]) {
    auto &canvas = screen::canvas();
    float pxPerNm = RADIUS / f.rangeNm;
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);

    // Each end in pixels, the way out beyond it, and how far out along that
    // its number sits.
    struct EndPx {
        float x, y, ox, oy;
        float reach;  // how far the number's box reaches along the runway's line
        float dist;   // how far out along the line the number is
        int labelX, labelY;  // the middle of the number
    };
    EndPx ends[MAX_ZOOM_RUNWAYS][2];
    bool drawn[MAX_ZOOM_RUNWAYS];
    LabelBox placed[MAX_ZOOM_RUNWAYS * 2];
    int placedCount = 0;
    for (int r = 0; r < f.runwayCount; r++) {
        const Runway &rw = f.runways[r];
        Vec a = toLocal(f, rw.end[0].lat, rw.end[0].lon), b = toLocal(f, rw.end[1].lat, rw.end[1].lon);
        float ax = CENTER_X + a.x * pxPerNm, ay = CENTER_Y - a.y * pxPerNm;
        float bx = CENTER_X + b.x * pxPerNm, by = CENTER_Y - b.y * pxPerNm;
        float len = hypotf(bx - ax, by - ay);
        drawn[r] = len >= 1.0f;
        if (!drawn[r]) {
            continue;
        }
        float ux = (bx - ax) / len, uy = (by - ay) / len;  // from end 0 to end 1
        for (int k = 0; k < 2; k++) {
            EndPx &e = ends[r][k];
            e.x = k ? bx : ax;
            e.y = k ? by : ay;
            e.ox = k ? ux : -ux;
            e.oy = k ? uy : -uy;
            if (rw.closed) {
                continue;  // no number to place
            }
            int w = canvas.textWidth(rw.end[k].ident);
            e.reach = fabsf(e.ox) * w / 2.0f + fabsf(e.oy) * LABEL_H / 2.0f;
            // Clear of the numbers already placed: at a big airport the ends
            // of different runways can sit close enough together for theirs
            // to collide. Beyond the end, then beside it either way, then the
            // same a little further out - and where none of that is clear,
            // beyond the end regardless.
            float nx = -e.oy, ny = e.ox;
            float sideStep = 2 * (fabsf(nx) * w / 2.0f + fabsf(ny) * LABEL_H / 2.0f) + LABEL_PAD;
            e.dist = e.reach + RUNWAY_LABEL_GAP;
            e.labelX = (int)lroundf(e.x + e.ox * e.dist);
            e.labelY = (int)lroundf(e.y + e.oy * e.dist);
            bool clear = false;
            for (int out = 0; out < 4 && !clear; out++) {
                for (int s = 0; s < 3 && !clear; s++) {
                    float dist = e.reach + RUNWAY_LABEL_GAP + out * LABEL_H / 2;
                    float side = s == 0 ? 0 : (s == 1 ? sideStep : -sideStep);
                    int cx = (int)lroundf(e.x + e.ox * dist + nx * side);
                    int cy = (int)lroundf(e.y + e.oy * dist + ny * side);
                    if (!overlapsAny(placed, placedCount, cx - w / 2, cy - LABEL_H / 2, w, LABEL_H)) {
                        e.dist = dist;
                        e.labelX = cx;
                        e.labelY = cy;
                        clear = true;
                    }
                }
            }
            placed[placedCount++] = {(int16_t)(e.labelX - w / 2), (int16_t)(e.labelY - LABEL_H / 2), (int16_t)w,
                                     (int16_t)LABEL_H};
        }
    }

    for (int r = 0; r < f.runwayCount; r++) {
        if (!drawn[r] || f.runways[r].closed) {
            continue;
        }
        for (int k = 0; k < 2; k++) {
            const EndPx &e = ends[r][k];
            float start = e.dist + e.reach + RUNWAY_LABEL_GAP;
            float sx = e.x + e.ox * start, sy = e.y + e.oy * start;
            float t = toRing(sx, sy, e.ox, e.oy);
            drawDashed(sx, sy, sx + e.ox * t, sy + e.oy * t, 6, 6, inUse[r][k] ? colorFaint : colorRing);
        }
    }

    // The closed ones first, so the open ones lie over them.
    for (int pass = 0; pass < 2; pass++) {
        for (int r = 0; r < f.runwayCount; r++) {
            bool closed = f.runways[r].closed;
            if (!drawn[r] || closed != (pass == 0)) {
                continue;
            }
            const EndPx &a = ends[r][0], &b = ends[r][1];
            float ux = b.ox, uy = b.oy;  // end 0 to end 1
            float halfW = std::max(f.runways[r].widthFt * 0.3048f / 1852.0f * pxPerNm / 2.0f, 2.0f);
            float nx = -uy * halfW, ny = ux * halfW;
            int x0 = (int)lroundf(a.x + nx), y0 = (int)lroundf(a.y + ny);
            int x1 = (int)lroundf(b.x + nx), y1 = (int)lroundf(b.y + ny);
            int x2 = (int)lroundf(b.x - nx), y2 = (int)lroundf(b.y - ny);
            int x3 = (int)lroundf(a.x - nx), y3 = (int)lroundf(a.y - ny);
            if (!closed) {
                canvas.fillTriangle(x0, y0, x1, y1, x2, y2, colorRunway);
                canvas.fillTriangle(x0, y0, x2, y2, x3, y3, colorRunway);
                if (halfW >= 3.5f) {
                    drawDashed(a.x + ux * 10, a.y + uy * 10, b.x - ux * 10, b.y - uy * 10, 8, 8, colorBg);
                }
                continue;
            }
            canvas.fillTriangle(x0, y0, x1, y1, x2, y2, colorBg);  // hollow, over any approach line
            canvas.fillTriangle(x0, y0, x2, y2, x3, y3, colorBg);
            canvas.drawLine(x0, y0, x1, y1, colorRunway);
            canvas.drawLine(x1, y1, x2, y2, colorRunway);
            canvas.drawLine(x2, y2, x3, y3, colorRunway);
            canvas.drawLine(x3, y3, x0, y0, colorRunway);
            // A cross a quarter of the way in from each end.
            constexpr float ARM = 6.0f;
            float dx1 = (ux - uy) * ARM / (float)M_SQRT2, dy1 = (uy + ux) * ARM / (float)M_SQRT2;
            float dx2 = (ux + uy) * ARM / (float)M_SQRT2, dy2 = (uy - ux) * ARM / (float)M_SQRT2;
            for (float f : {0.25f, 0.75f}) {
                float cx = a.x + (b.x - a.x) * f, cy = a.y + (b.y - a.y) * f;
                canvas.drawLine((int)lroundf(cx - dx1), (int)lroundf(cy - dy1), (int)lroundf(cx + dx1),
                                (int)lroundf(cy + dy1), colorFaint);
                canvas.drawLine((int)lroundf(cx - dx2), (int)lroundf(cy - dy2), (int)lroundf(cx + dx2),
                                (int)lroundf(cy + dy2), colorFaint);
            }
        }
    }

    for (int r = 0; r < f.runwayCount; r++) {
        if (!drawn[r] || f.runways[r].closed) {
            continue;
        }
        for (int k = 0; k < 2; k++) {
            const EndPx &e = ends[r][k];
            canvas.setTextColor(inUse[r][k] ? colorText : colorMuted);
            canvas.drawString(f.runways[r].end[k].ident, e.labelX, e.labelY);
        }
    }
}

// A magnifier with a minus in it, then the word: what the button undoes.
// Where the radar's Back is.
void drawUnzoomButton() {
    auto &canvas = screen::canvas();
    canvas.fillRoundRect(BTN_X, BACK_Y, BTN_W, BTN_H, 6, colorBtnBg);
    int cx = BTN_X + 30, cy = BACK_Y + BTN_H / 2 - 3;
    canvas.drawCircle(cx, cy, 8, colorText);
    canvas.drawCircle(cx, cy, 7, colorText);
    canvas.drawFastHLine(cx - 4, cy, 9, colorText);
    for (int i = 0; i <= 1; i++) {
        canvas.drawLine(cx + 6 + i, cy + 6, cx + 11 + i, cy + 11, colorText);
    }
    canvas.setTextSize(2);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(colorText);
    canvas.drawString("Unzoom", BTN_X + 52, BACK_Y + BTN_H / 2);
}

void drawZoomHeader() {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(colorText);
    canvas.drawString(g_zoom.code, 16, 35);
    canvas.setTextColor(colorMuted);
    int closed = 0;
    for (int r = 0; r < g_zoom.runwayCount; r++) {
        closed += g_zoom.runways[r].closed ? 1 : 0;
    }
    int open = g_zoom.runwayCount - closed;
    String what = g_zoom.runwayCount == 0 ? String(" - no runway data")
                                          : " - " + String(open) + (open == 1 ? " runway" : " runways");
    if (closed) {
        what += ", " + String(closed) + " closed";
    }
    int x = 16 + canvas.textWidth(g_zoom.code);
    canvas.drawString(what, x, 35);
    // FOLLOW where the radar has it, working the same way: a tap, then a
    // contact - one at the gate, say, to follow it out - with what it is
    // doing said in the title. Before Unzoom, which leaves the text size
    // smaller than the title's.
    x += canvas.textWidth(what);
    canvas.setTextColor(colorMuted);
    if (g_zoomFollowHex.length()) {
        const char *following = "  following ";
        canvas.drawString(following, x, 35);
        canvas.setTextColor(colorHome);
        canvas.drawString(g_zoomFollowCallsign, x + canvas.textWidth(following), 35);
    } else if (g_zoomPicking) {
        canvas.drawString("  tap a plane", x, 35);
    }
    drawButton(FOLLOW_Y, g_zoomPicking ? "TAP PLANE" : "FOLLOW",
               g_zoomFollowHex.length() || g_zoomPicking ? Look::LIT : Look::PLAIN);
    drawRecButton();
    drawUnzoomButton();
}

}  // namespace

bool radarZoomFrame(const String &code, double &lat, double &lon, float &rangeNm) {
    ZoomFrame f;
    if (!frameZoom(code, f)) {
        return false;
    }
    lat = f.lat;
    lon = f.lon;
    rangeNm = f.rangeNm;
    return true;
}

bool radarZoomOpen(const String &code, double &lat, double &lon, float &rangeNm) {
    if (!frameZoom(code, g_zoom)) {
        return false;
    }
    lat = g_zoom.lat;
    lon = g_zoom.lon;
    rangeNm = g_zoom.rangeNm;
    return true;
}

void radarZoomSetFollow(const String &hex, const String &callsign, bool picking) {
    g_zoomPicking = picking;
    g_zoomFollowHex = hex;
    g_zoomFollowCallsign = callsign;
}

namespace {

// The zoom's plot and footer readouts, for frame `f`: what the live zoom and
// a replay of one both draw. As radarPlotDraw(): with `full` the caller has
// cleared the screen and flushes; without it, only the plot and the footer
// are erased and redrawn, and pushed as `push` says.
void zoomPlot(const ZoomFrame &f, const std::vector<Aircraft> &aircraft,
              const std::vector<replay::TrailPoint> *trails, bool flash, const String &followHex, bool full,
              bool push) {
    auto &canvas = screen::canvas();
    uint32_t t0 = micros();
    if (!full) {
        screen::fillRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T, colorBg);
        canvas.fillRect(FOOT_L, ZOOM_FOOT_T, FOOT_R - FOOT_L, FOOT_B - ZOOM_FOOT_T, colorBg);
    }
    canvas.setFont(&fonts::Font0);

    float range = f.rangeNm;
    float step = range > 3.0f ? 1.0f : 0.5f;
    drawScope(range, step);

    canvas.setClipRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);
    bool inUse[MAX_ZOOM_RUNWAYS][2];
    runwaysInUse(f, aircraft, inUse);
    drawRunways(f, inUse);
    static const std::vector<uint8_t> noneNew;  // the radar marks new contacts; here they are all just traffic
    int plotted = 0, labelled = 0;
    uint32_t t1 = micros();
    drawContacts(aircraft, noneNew, trails, flash, {f.lat, f.lon, RADIUS / range, ZOOM_LEAD_S, true, followHex},
                 plotted, labelled);
    uint32_t t2 = micros();
    g_backgroundUs += t1 - t0;
    g_contactsUs += t2 - t1;
    g_plotFrames++;
    canvas.clearClipRect();

    // As the radar's, with the runways in use on top when there are any.
    if (!full) {
        canvas.fillRect(0, RANGE_BOX_T, PLOT_L, BOX_H, colorBg);
    }
    footerBox(nmText(range) + " nm range   rings every " + nmText(step) + " nm", 16, FOOTER_Y, false);
    String used;
    for (int r = 0; r < f.runwayCount; r++) {
        for (int k = 0; k < 2; k++) {
            if (inUse[r][k]) {
                used += String(" ") + f.runways[r].end[k].ident;
            }
        }
    }
    if (used.length()) {
        footerBox("in use:" + used, 1264, FOOTER_Y - 2 * LEGEND_OFFSET, true);
    }
    footerBox("o ground   ^ climb   v descent", 1264, FOOTER_Y - LEGEND_OFFSET, true);
    footerBox(String(plotted) + " contacts" + (labelled < plotted ? "   " + String(labelled) + " labelled" : ""), 1264,
              FOOTER_Y, true);

    if (!full) {
        uint32_t t3 = micros();
        pushPlot(push, ZOOM_FOOT_T);
        g_pushUs += micros() - t3;
    }
}

// The frame a replay last drew its zoom in, kept apart from the live zoom's.
ZoomFrame g_replayZoom;

}  // namespace

void radarZoomDraw(const std::vector<Aircraft> &aircraft, bool full) {
    ensureColors();
    if (full) {
        g_choiceCount = 0;
        screen::clear(colorBg);
        drawZoomHeader();
    }
    // With a list up, a refresh is drawn under it and pushed along with it.
    zoomPlot(g_zoom, aircraft, nullptr, true, g_zoomFollowHex, full, g_choiceCount == 0);
    // Following into the zoom, its height against the field's.
    bool following = g_zoomFollowHex.length() > 0;
    if (following) {
        radarTelemetryDraw(LIVE_TEL_X, LIVE_TEL_Y, g_heights, millis(),
                           g_zoom.runwayCount ? g_zoom.runways[0].elevationFt : INT32_MIN, g_lostS);
    }
    if (g_choiceCount) {
        drawChooser();
    }
    if (full || g_choiceCount || following) {
        screen::flush();
    }
}

bool radarZoomPlotDraw(const RadarScene &scene, const String &code, bool full) {
    ensureColors();
    if (g_replayZoom.code != code && !frameZoom(code, g_replayZoom)) {
        return false;
    }
    zoomPlot(g_replayZoom, scene.aircraft, scene.trails, scene.flash, scene.followHex, full, scene.push);
    return true;
}

ZoomAction radarZoomHandleTouch(int x, int y, String &outHex) {
    if (!g_choiceCount) {
        int b = buttonAt(x, y);
        if (b == BTN_BACK) {
            return ZoomAction::UNZOOM;
        }
        if (b == BTN_FOLLOW) {
            return ZoomAction::TOGGLE_FOLLOW;
        }
        if (b == BTN_REC) {
            return ZoomAction::TOGGLE_RECORD;
        }
    }
    switch (tapTarget(x, y, false, outHex)) {
    case Target::AIRCRAFT:
        return ZoomAction::SELECT;
    case Target::DISMISSED:
        return ZoomAction::DISMISS;
    default:
        return ZoomAction::NONE;
    }
}

RadarAction radarScreenHandleTouch(int x, int y, String &outHex, String &outAirport) {
    // With the list up, every tap is the list's: outside it is a dismissal.
    if (!g_choiceCount) {
        switch (buttonAt(x, y)) {
        case BTN_BACK:
            return RadarAction::BACK;
        case BTN_HOME:
            return RadarAction::CENTRE_HOME;
        case BTN_AIRPORT:
            return RadarAction::CENTRE_AIRPORT;
        case BTN_ALL:
            return RadarAction::CENTRE_ALL;
        case BTN_FOLLOW:
            return RadarAction::TOGGLE_FOLLOW;
        case BTN_REC:
            return RadarAction::TOGGLE_RECORD;
        case BTN_REPLAYS:
            return RadarAction::OPEN_RECORDINGS;
        default:
            break;
        }
    }
    String id;
    switch (tapTarget(x, y, true, id)) {
    case Target::AIRCRAFT:
        outHex = id;
        return RadarAction::SELECT;
    case Target::AIRPORT:
        outAirport = id;
        return RadarAction::ZOOM_AIRPORT;
    case Target::DISMISSED:
        return RadarAction::DISMISS;
    case Target::NONE:
        break;
    }
    return RadarAction::NONE;
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

// --- the followed aircraft's height --------------------------------------------

namespace {

// 12,345: an altitude reads at a glance with its thousands marked.
String withCommas(int32_t v) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)(v < 0 ? -v : v));
    String digits(buf), out;
    for (int i = 0; i < (int)digits.length(); i++) {
        if (i && (digits.length() - i) % 3 == 0) {
            out += ',';
        }
        out += digits[i];
    }
    return v < 0 ? "-" + out : out;
}

// The top of the chart: the highest it shows rounded up to a step that keeps
// the scale readable, and never less than a circuit's height, so a take-off
// roll has somewhere to climb into.
int32_t chartTop(int32_t highest) {
    int32_t step = highest > 10000 ? 5000 : 1000;
    int32_t top = (std::max(highest, (int32_t)1000) + step - 1) / step * step;
    return std::max(top, (int32_t)2000);
}

constexpr int TEL_LABEL_H = 16;
constexpr int TEL_VALUE_Y = 20;   // the altitude, large, under its label
constexpr int TEL_ROWS_Y = 64;    // the readouts under it, a row each
constexpr int TEL_ROW_H = 22;
constexpr int TEL_CHART_Y = 140;  // from the top of the box
constexpr int TEL_CHART_H = 166;
// Two polls further apart than this aren't joined: the line between them
// would claim a climb or descent nobody saw.
constexpr uint32_t TEL_GAP_MS = 180000;

}  // namespace

void radarSetHeights(const std::vector<FollowSample> &samples, uint32_t lostS) {
    g_heights = samples;
    g_lostS = lostS;
}

void radarTelemetryDraw(int x, int y, const std::vector<FollowSample> &samples, uint32_t nowMs, int32_t fieldFt,
                        uint32_t lostS) {
    ensureColors();
    auto &canvas = screen::canvas();
    canvas.fillRect(x, y, TELEMETRY_W, TELEMETRY_H, colorBg);
    screen::markDirty(x, y, TELEMETRY_W, TELEMETRY_H);
    canvas.setFont(&fonts::Font0);

    // The latest report by now, and the window of them the chart covers.
    uint32_t from = nowMs > TELEMETRY_WINDOW_MS ? nowMs - TELEMETRY_WINDOW_MS : 0;
    const FollowSample *latest = nullptr;
    int32_t highest = fieldFt == INT32_MIN ? 0 : fieldFt;
    for (const FollowSample &s : samples) {
        if (s.ms > nowMs) {
            break;
        }
        latest = &s;
        if (s.ms >= from && s.hasAlt) {
            highest = std::max(highest, s.altFt);
        }
    }

    // --- readouts ---
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextSize(2);
    canvas.setTextColor(colorFaint);
    canvas.drawString("ALTITUDE", x, y);
    if (lostS) {
        canvas.setTextColor(colorAlert);
        canvas.drawString("lost " + String(lostS) + "s", x + canvas.textWidth("ALTITUDE  "), y);
    }
    String alt = "-", vs = "-", gs = "-", agl;
    if (latest != nullptr) {
        if (latest->ground) {
            alt = "GND";
        } else if (latest->hasAlt) {
            alt = withCommas(latest->altFt);
        }
        if (latest->hasVs) {
            vs = (latest->vsFpm > 0 ? "+" : "") + withCommas(latest->vsFpm) + " fpm";
        } else if (latest->ground) {
            vs = "on the ground";
        }
        if (latest->hasGs) {
            gs = String(latest->gsKt) + " kt";
        }
        if (fieldFt != INT32_MIN && latest->hasAlt) {
            agl = latest->ground ? String("0 ft") : withCommas(latest->altFt - fieldFt) + " ft";
        }
    }
    canvas.setTextSize(4);
    canvas.setTextColor(colorHome);
    canvas.drawString(alt, x, y + TEL_VALUE_Y);
    if (alt != "GND" && alt != "-") {
        canvas.setTextSize(2);
        canvas.setTextColor(colorMuted);
        canvas.drawString("ft", x + 4 * 6 * (int)alt.length() + 8, y + TEL_VALUE_Y + 14);
    }
    canvas.setTextSize(2);
    const char *names[] = {"V/S", "GS", "AGL"};
    const String *values[] = {&vs, &gs, &agl};
    int rows = agl.length() ? 3 : 2;
    for (int i = 0; i < rows; i++) {
        int ry = y + TEL_ROWS_Y + i * TEL_ROW_H;
        canvas.setTextColor(colorFaint);
        canvas.drawString(names[i], x, ry);
        canvas.setTextColor(colorText);
        canvas.drawString(*values[i], x + 60, ry);
    }

    // --- the chart ---
    int cx = x, cy = y + TEL_CHART_Y, cw = TELEMETRY_W, ch = TEL_CHART_H;
    canvas.drawRect(cx, cy, cw, ch, colorRing);
    int32_t top = chartTop(highest);
    auto px = [&](uint32_t ms) { return cx + 1 + (int)((int64_t)(cw - 3) * (int64_t)(ms - from) / TELEMETRY_WINDOW_MS); };
    auto py = [&](int32_t ft) {
        int32_t clamped = std::min(std::max(ft, (int32_t)0), top);
        return cy + ch - 2 - (int)((int64_t)(ch - 4) * clamped / top);
    };
    // Gridlines a step apart, faint, with the top one labelled.
    int32_t grid = top > 10000 ? 5000 : top > 4000 ? 2000 : 1000;
    for (int32_t ft = grid; ft < top; ft += grid) {
        int gy = py(ft);
        for (int gx = cx + 2; gx < cx + cw - 2; gx += 6) {
            canvas.drawPixel(gx, gy, colorRing);
        }
    }
    canvas.setTextSize(1);
    canvas.setTextColor(colorRingText);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(withCommas(top) + " ft", cx + 4, cy + 4);
    if (fieldFt != INT32_MIN) {
        int fy = py(fieldFt);
        for (int gx = cx + 2; gx < cx + cw - 2; gx += 4) {
            canvas.drawFastHLine(gx, fy, 2, colorRunway);
        }
        canvas.setTextDatum(BR_DATUM);
        canvas.drawString("field " + withCommas(fieldFt), cx + cw - 4, fy - 2);
        canvas.setTextDatum(TL_DATUM);
    }

    canvas.setClipRect(cx + 1, cy + 1, cw - 2, ch - 2);
    const FollowSample *prev = nullptr;
    for (const FollowSample &s : samples) {
        if (s.ms > nowMs) {
            break;
        }
        if (s.ms < from || !s.hasAlt) {
            continue;
        }
        int sx = px(s.ms), sy = py(s.altFt);
        if (prev != nullptr && s.ms - prev->ms <= TEL_GAP_MS) {
            int qx = px(prev->ms), qy = py(prev->altFt);
            canvas.drawLine(qx, qy, sx, sy, colorBlip);
            canvas.drawLine(qx, qy - 1, sx, sy - 1, colorBlip);
            // Between the air and the ground: a touchdown or a take-off,
            // marked where it was first seen.
            if (prev->ground != s.ground) {
                for (int my = cy + 14; my < cy + ch - 2; my += 6) {
                    canvas.drawFastVLine(sx, my, 3, colorFaint);
                }
                canvas.setTextColor(colorText);
                canvas.setTextDatum(TC_DATUM);
                canvas.drawString(s.ground ? "TD" : "TO", sx, cy + 4);
                canvas.setTextDatum(TL_DATUM);
            }
        }
        if (s.ground) {
            canvas.drawCircle(sx, sy, 2, colorBlip);
        } else {
            canvas.fillCircle(sx, sy, 2, colorBlip);
        }
        prev = &s;
    }
    canvas.clearClipRect();

    canvas.setTextSize(2);
    canvas.setTextColor(colorFaint);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("-" + String(TELEMETRY_WINDOW_MS / 60000) + "m", cx, cy + ch + 6);
    canvas.setTextDatum(TR_DATUM);
    canvas.drawString("now", cx + cw, cy + ch + 6);
}
