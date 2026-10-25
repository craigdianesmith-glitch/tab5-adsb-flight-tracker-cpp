#include "radar_screen.h"

#include "aircraft_db.h"
#include "airfield_pack.h"

#include <M5Unified.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <math.h>
#if defined(ESP_PLATFORM)
#include <esp_cpu.h>
#endif

#include "airport_info.h"
#include "airports.h"
#include "coast.h"
#include "filter.h"
#include "names.h"
#include "navaids.h"
#include "alerts.h"
#include "runways.h"
#include "screen.h"
#include "sound.h"

namespace {

bool colorsReady = false;

// The buttons stand in a column down the right-hand edge, in the margin
// beside the plot, grouped by what they do: Back; the centre and the
// airports, which are one choice of three; follow me; then recording; then
// the range. Each is its own button doing one thing on a tap - there is no
// long press anywhere on the screen.
constexpr int BTN_W = 150, BTN_H = 50;
constexpr int BTN_X = 1280 - 16 - BTN_W;
constexpr int BTN_GAP = 10, GROUP_GAP = 26;
enum Button { BTN_BACK, BTN_HOME, BTN_AIRPORT, BTN_ALL, BTN_FOLLOW, BTN_REC, BTN_REPLAYS, BTN_ZOOM_IN, BTN_ZOOM_OUT,
              BTN_COUNT };
constexpr int BACK_Y = 8;
constexpr int HOME_Y = BACK_Y + BTN_H + GROUP_GAP;
constexpr int AIRPORT_Y = HOME_Y + BTN_H + BTN_GAP;
constexpr int ALL_Y = AIRPORT_Y + BTN_H + BTN_GAP;
constexpr int FOLLOW_Y = ALL_Y + BTN_H + GROUP_GAP;
constexpr int REC_Y = FOLLOW_Y + BTN_H + GROUP_GAP;
constexpr int REPLAY_Y = REC_Y + BTN_H + BTN_GAP;
// Clear of the footer readouts below them, on the radar; the zoom has none.
constexpr int ZOOM_IN_Y = REPLAY_Y + BTN_H + GROUP_GAP;
constexpr int ZOOM_OUT_Y = ZOOM_IN_Y + BTN_H + BTN_GAP;
constexpr int BTN_Y[BTN_COUNT] = {BACK_Y, HOME_Y, AIRPORT_Y, ALL_Y, FOLLOW_Y, REC_Y, REPLAY_Y, ZOOM_IN_Y, ZOOM_OUT_Y};
// A tap counts for a button from anywhere in the margin to its left and
// halfway into the gaps around it, so one that lands a little off still
// gets it rather than nothing.
constexpr int BTN_REACH_X = BTN_X - 24;
// Following, beside FOLLOW in the margin between the plot and the column -
// clear of the column's reach - the way to the 3D view of the aircraft.
constexpr int B3D_W = 86, B3D_X = BTN_REACH_X - 6 - B3D_W;
// The plot is as large as the 720px height allows: the full height, the
// outer ring's label just fitting above it, with the title in the corner to
// its left rather than in a band across the top. Its sides are what the
// replay's panels either side leave, and the follow telemetry and the button
// column the live screens have. The compass letters sit just inside the
// outer ring rather than outside it - outside, they were what capped the
// radius, and "S" ran off the bottom of the screen.
constexpr int CENTER_X = 640, CENTER_Y = 362, RADIUS = 342;
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
// the plot's column, with a little room at its sides for a blip's chevron,
// the full height of the canvas; and, separately, the footer readouts to its
// right. The title and the margins either side of the plot never change
// while the screen is up, and pushing them again was two-thirds of a refresh.
// Contacts are clipped to the column so nothing is drawn outside what gets
// erased - a vector at short range can otherwise reach well past the ring.
constexpr int PLOT_L = CENTER_X - RADIUS - 6, PLOT_R = CENTER_X + RADIUS + 6;
constexpr int PLOT_T = 0, PLOT_B = 720;
// The title, in the corner left of the plot: a line at the size it always
// was, and up to two smaller ones under it saying what is going on.
constexpr int TITLE_Y = 24, TITLE_LINE2_Y = 56, TITLE_LINE3_Y = 78;
constexpr int TITLE_BLOCK_H = 90;
constexpr int FOOT_L = PLOT_R, FOOT_R = 1280;
constexpr int FOOT_T = FOOTER_Y - LEGEND_OFFSET - BOX_PAD_Y, FOOT_B = FOOTER_Y - BOX_PAD_Y + BOX_H;
constexpr int RANGE_BOX_T = FOOTER_Y - BOX_PAD_Y;  // the range readout, bottom left
// And over it in the zoom, the runways in use - moved there from the stack
// at the bottom right, which the zoom's ZOOM buttons now come down to.
constexpr int IN_USE_BOX_T = RANGE_BOX_T - LEGEND_OFFSET;

constexpr int MAX_LABELS = 64;
constexpr int LABEL_H = 16;  // Font0 at size 2
constexpr int LABEL_PAD = 4;
// A label tried first where it was on the last plot may stay there with this
// much room rather than LABEL_PAD - see PlacedNavaidLabel.
constexpr int KEPT_LABEL_PAD = 1;

// Where the centre's airport code goes, under the cross.
constexpr int CENTRE_CODE_Y = CENTER_Y + 13;

// Around 25 airports fall inside the widest range over the busiest parts of
// the table (London, New York, Frankfurt), so this is never the limit.
constexpr int MAX_AIRPORT_LABELS = 48;
constexpr int AIRPORT_R = 5, AIRPORT_TICK = 3;

struct LabelBox {
    int16_t x, y, w, h;
};

// The map's labels on the plot being drawn - the centre's code, the other
// airports', then the navaids' - placed in that order, each kept clear of
// those before it.
constexpr int MAX_MAP_LABELS = 96;
LabelBox g_mapLabels[MAX_MAP_LABELS];
int g_mapLabelCount = 0;

// `skip`, the index of one box not to count - a label's own symbol's.
bool overlapsAny(const LabelBox *boxes, int count, int x, int y, int w, int h, int pad = LABEL_PAD,
                 int skip = -1) {
    for (int j = 0; j < count; j++) {
        const LabelBox &b = boxes[j];
        if (j != skip && x - pad < b.x + b.w && x + w + pad > b.x && y - pad < b.y + b.h && y + h + pad > b.y) {
            return true;
        }
    }
    return false;
}

// What else is on the plot that a navaid's label keeps off, besides the other
// labels: the scope's ring distances and compass points, the centre's cross
// or the followed aircraft's ring, and every airfield and navaid symbol.
// drawScope starts it afresh, as the first thing drawn on a plot - and with it
// g_airfieldSpots and g_runwayBars below.
std::vector<LabelBox> g_mapMarks;
// And the contacts' own labels, as drawContacts() last placed them: the map's
// names are drawn after the traffic, and keep off these.
std::vector<LabelBox> g_contactLabels;
// The fastest of the contacts it last drew, in knots - see radarFastestKt().
float g_fastestKt = 0;

void noteMark(int x, int y, int w, int h) {
    g_mapMarks.push_back({(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h});
}

// Where each airfield symbol went, unrounded, so a navaid that would sit on one
// - an airport's own VOR, as GOW is Glasgow's - can be left off rather than
// drawn over it. Unrounded, so the two stay the same distance apart as the map
// moves under a following, rather than a pixel either way.
std::vector<std::pair<float, float>> g_airfieldSpots;

// The zoom's runways, each as its centreline and how far either side of it is
// taken up - the runway, or an in-use arrow or closed runway's crosses where
// those are wider - for a navaid's label to keep off.
struct RunwayBar {
    float x0, y0, x1, y1, half;
};
std::vector<RunwayBar> g_runwayBars;

// Whether a box comes within `pad` of a runway: its centreline clipped
// against the box grown by the runway's half-width and the pad.
bool overlapsRunway(int x, int y, int w, int h, int pad) {
    for (const RunwayBar &b : g_runwayBars) {
        float grow = b.half + pad;
        float dx = b.x1 - b.x0, dy = b.y1 - b.y0;
        const float p[4] = {-dx, dx, -dy, dy};
        const float q[4] = {b.x0 - (x - grow), (x + w + grow) - b.x0, b.y0 - (y - grow), (y + h + grow) - b.y0};
        float t0 = 0, t1 = 1;
        bool hit = true;
        for (int i = 0; i < 4 && hit; i++) {
            if (p[i] == 0) {
                hit = q[i] >= 0;
            } else if (p[i] < 0) {
                t0 = std::max(t0, q[i] / p[i]);
            } else {
                t1 = std::min(t1, q[i] / p[i]);
            }
            hit = hit && t0 <= t1;
        }
        if (hit) {
            return true;
        }
    }
    return false;
}

// Whether a box lies wholly inside the outer ring, `pad` clear of it.
bool insideScope(int x, int y, int w, int h, int pad) {
    long r = RADIUS - 2 - pad;
    for (int cx : {x, x + w}) {
        for (int cy : {y, y + h}) {
            long dx = cx - CENTER_X, dy = cy - CENTER_Y;
            if (dx * dx + dy * dy > r * r) {
                return false;
            }
        }
    }
    return true;
}

// Where each airport was last plotted, so a tap on its code or its symbol
// can open its zoom - by way of the chooser below where a contact is within
// reach too, as its own ground traffic always is.
struct AirportHit {
    int16_t x, y;    // the symbol
    LabelBox label;  // where its code went, or w 0 where it was left off
    char code[5];
};
constexpr int AIRPORT_TAP_R = 22;
constexpr int AIRPORT_LABEL_PAD = 10;
AirportHit g_airportHits[MAX_AIRPORT_LABELS + 1];  // and the centre's
int g_airportHitCount = 0;

// The same for the navaids, to open one's card.
struct NavaidHit {
    int16_t x, y;
    LabelBox label;
    const Navaid *navaid;
};
constexpr int MAX_NAVAID_HITS = 64;
NavaidHit g_navaidHits[MAX_NAVAID_HITS];
int g_navaidHitCount = 0;
bool g_navaidsOn = true;  // the setting
bool g_coastOn = true;    // and the one for the map's outlines
bool g_ringsOn = false;   // and for the radar's range rings
// Zoomed out past where there is traffic: the map alone - see
// radarScreenSetMapOnly().
bool g_mapOnly = false;
bool g_filtered = false;  // a filter cuts the list - said under the title
bool g_autoRange = false;  // following with auto zoom choosing the range - see radarScreenSetAutoRange()

// Free look: a plot dragged off its own centre - see radarDragTo(). The
// radar's and the zoom's each keep theirs, with what it was dragged away
// from: the radar's home, the zoom's own centre. A radar whose home has since
// moved is back on its own; its centre's mode can change under it - ALL,
// tapped on a free look, marks every airport there and leaves it where it is.
struct FreeLook {
    bool active = false;
    double lat = 0, lon = 0;
    double fromLat = 0, fromLon = 0;
};
FreeLook g_radarLook, g_zoomLook;

// What the last plot drawn was of, for a drag to be measured on: its middle,
// its scale - at its own range, not one a drag has it drawn at - and where
// its middle is when nothing has dragged it.
struct PlotView {
    double lat = 0, lon = 0;
    float pxPerNm = 1;
    double ownLat = 0, ownLon = 0;
    double homeLat = 0, homeLon = 0;  // the radar's: what a free look is of - see FreeLook
};
PlotView g_radarView, g_zoomView;

// A finger down on a plot that may be dragging it, with the plot as it was
// when it landed - and once it has gone DRAG_MIN_PX from there, the line it
// has stretched out from the middle: so far east and north, which is `nm` on
// `bearing` and would put the middle at (lat, lon).
constexpr int DRAG_MIN_PX = 12;
struct Drag {
    bool down = false, zoom = false, showing = false;
    int startX = 0, startY = 0;
    PlotView view;
    float eastNm = 0, northNm = 0;
    float nm = 0, bearing = 0;
    double lat = 0, lon = 0;
};
Drag g_drag;

// The ranges a plot is drawn at for a line dragged past its edge: the first
// that takes the line, with room for the ring on its end. A swipe across the
// whole screen is twice the plot's own range, so it is seldom more than a
// step or two up.
constexpr float DRAG_RANGES_NM[] = {1,   1.5, 2,   3,   4,   5,   6,   8,   10,  15,   20,   30,   40,   60,  80,
                                    100, 120, 150, 200, 250, 300, 400, 500, 600, 800, 1000, 1500, 2000, 3000};
constexpr float DRAG_LINE_FILLS = 0.92f;
float dragRangeNm(float ownNm) {
    if (g_drag.nm <= ownNm * DRAG_LINE_FILLS) {
        return ownNm;
    }
    for (float r : DRAG_RANGES_NM) {
        if (r > ownNm && g_drag.nm <= r * DRAG_LINE_FILLS) {
            return r;
        }
    }
    // Past the widest, as wide as it goes - or the plot's own, if wider.
    return std::max(ownNm, DRAG_RANGES_NM[sizeof(DRAG_RANGES_NM) / sizeof(DRAG_RANGES_NM[0]) - 1]);
}

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
    // What the chooser lists it as - callsign, type, altitude - kept as its
    // parts, each short enough to be held without the heap, and put together
    // only if the chooser comes up: built for every contact on every draw, it
    // was 32us a contact, a busy sky's 3.8ms a frame.
    String callsign, type, altStr;
    bool ground;
};

String blipLabel(const Blip &b) {
    String label = b.callsign;
    if (b.type.length() && b.type != "----") {
        label += "  " + b.type;
    }
    if (b.altStr != "?") {
        label += "  " + (b.ground ? String("GND") : b.altStr + "ft");
    }
    return label;
}
// Above the poll's MAX_CONTACTS with room to spare: alerted aircraft are kept
// beyond that cap and come last, so a tighter limit here made exactly the
// ones worth tapping the ones that couldn't be.
constexpr int MAX_BLIPS = 128;
constexpr int TAP_RADIUS = 28;
std::vector<Blip> g_blips(MAX_BLIPS);  // 9KB: on the heap - in the PSRAM - not the chip's own memory
int g_blipCount = 0;

// A phosphor-green ramp rather than the rest of the app's blue-grey: on a
// plan-position plot the brightness of a mark is what carries the meaning, and
// a single hue leaves brightness free to do that.
uint16_t colorDisabled;
uint16_t colorBg, colorText, colorMuted, colorFaint, colorBtnBg, colorRing, colorRingText, colorBlip, colorNewBlip,
    colorHome, colorTrail, colorAlert, colorEmergency, colorCargo, colorRec, colorRunway, colorCoast, colorCoastAlone,
    colorLandName, colorWaterName, colorMilitaryField;
RadarPalette g_palette;

// Where a plot's drawing time goes, for the [perf] line: the parts that
// don't move from one frame to the next - erasing, the scope, the airports
// and runways - then the contacts, then pushing it to the panel. And of the
// first, how much was the map's outlines, in microseconds: it is a fraction
// of a millisecond most places and several over the lakes of Finland.
uint64_t g_backgroundUs = 0, g_contactsUs = 0, g_pushUs = 0, g_coastUs = 0;
uint32_t g_plotFrames = 0;

// And a whole plot's background - all a refresh of the traffic alone keeps -
// a layer at a time, in the CPU's cycles summed in 64 bits over the frames
// that drew each, so that a minute's refreshes don't water it down, nor the
// zoom's frames the radar's map: see radarTakeLayers().
enum Layer { LAYER_FILL, LAYER_COAST, LAYER_SCOPE, LAYER_CENTRE, LAYER_AIRPORTS, LAYER_NAVAIDS, LAYER_NAMES,
             LAYER_RUNWAYS, LAYER_TOWER, LAYER_COUNT };
const char *const LAYER_LABELS[LAYER_COUNT] = {"fill",    "coast", "scope",   "centre", "airports",
                                              "navaids", "names", "runways", "tower"};
uint64_t g_layerCycles[LAYER_COUNT] = {};
uint32_t g_layerFrames[LAYER_COUNT] = {};

// The CPU's cycle counter - none on the PC, which render2d_check builds this
// for, and where nothing reads the sums.
inline uint32_t cycleCount() {
#if defined(ESP_PLATFORM)
    return esp_cpu_get_cycle_count();
#else
    return 0;
#endif
}

// The cycles from `since` to now added to `layer`'s, a frame more of it; now,
// for the next one's.
uint32_t layerDone(Layer layer, uint32_t since) {
    uint32_t now = cycleCount();
    g_layerCycles[layer] += now - since;
    g_layerFrames[layer]++;
    return now;
}

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
    // A military airfield's symbol and code: as faint as a civil one's,
    // turned olive - told apart on a look, without standing out.
    colorMilitaryField = M5.Display.color565(0x8E, 0x92, 0x58);
    colorTrail = M5.Display.color565(0x2A, 0x7A, 0x4E);  // dimmer than any blip: where it was, not where it is
    // Greyed toward pavement, so a runway reads as ground under the traffic
    // rather than as a mark of the same kind as a contact.
    colorRunway = M5.Display.color565(0x4E, 0x6E, 0x5C);
    // The map under it all, greyed the same way and no brighter than the
    // rings: it says where the traffic is, and is not to be looked at for
    // itself.
    colorCoast = M5.Display.color565(0x26, 0x50, 0x48);
    // And a little brighter with the rings switched off, when it is all there
    // is under the traffic and has no rings to be told from.
    colorCoastAlone = M5.Display.color565(0x30, 0x66, 0x5A);
    // Its names a step brighter than its lines, to be read when looked for -
    // a sea's or a lake's a little bluer than the land's.
    colorLandName = M5.Display.color565(0x36, 0x6E, 0x60);
    colorWaterName = M5.Display.color565(0x32, 0x68, 0x74);
    // The one exception to the single hue: an alerted contact has to stand out
    // from the sky around it at a glance, which no shade of green does.
    colorAlert = M5.Display.color565(0xFF, 0xB3, 0x2E);
    colorEmergency = M5.Display.color565(0xFF, 0x4D, 0x4D);
    colorCargo = M5.Display.color565(0x7F, 0xC8, 0xFF);  // a cargo operator's label, as the table has it
    colorRec = M5.Display.color565(0xC0, 0x39, 0x2B);
    g_palette = {colorBg, colorText, colorMuted, colorFaint, colorBtnBg, colorRing, colorRec, colorDisabled};
    colorsReady = true;
}

// Rings on round numbers rather than thirds of the range, so the labels read
// as distances instead of arbitrary fractions.
int ringStep(int rangeNm) {
    if (rangeNm <= 5) return 1;  // following one down to an airport
    if (rangeNm <= 8) return 2;  // and auto zoom's steps on the way
    if (rangeNm <= 15) return 5;
    if (rangeNm <= 30) return 10;
    if (rangeNm <= 60) return 20;
    if (rangeNm <= 150) return 50;
    // The map's own ranges, out past the traffic's.
    if (rangeNm <= 300) return 100;
    if (rangeNm <= 750) return 250;
    if (rangeNm <= 1500) return 500;
    return 1000;
}

// A distance as a ring label: whole miles bare, the zoom's half miles to one
// decimal place and its quarters to two.
String nmText(float nm) {
    if (nm == floorf(nm)) {
        return String((int)nm);
    }
    return String(nm, nm * 2 == floorf(nm * 2) ? 1 : 2);
}

// The map under the plot - coasts and lakes, borders and state lines: see
// coast.h - drawn before anything else, so that everything else is over it.
void drawCoast(double lat, double lon, float rangeNm) {
    if (!g_coastOn) {
        return;
    }
    auto &canvas = screen::canvas();
    uint32_t t0 = micros();
    // Onto the canvas's own pixels, which it keeps high byte first.
    coastDraw({(uint16_t *)canvas.getBuffer(), canvas.width(), CENTER_X, CENTER_Y, RADIUS, lat, lon, rangeNm,
               __builtin_bswap16(g_ringsOn ? colorCoast : colorCoastAlone)});
    g_coastUs += micros() - t0;
}

// The scope itself: a ring every stepNm with its distance on it - unless
// they are switched off, which is a stepNm of 0 - the outer ring doubled, the
// compass points just inside it and a tick every 30 degrees.
void drawScope(float rangeNm, float stepNm) {
    auto &canvas = screen::canvas();
    g_mapMarks.clear();
    g_airfieldSpots.clear();
    g_runwayBars.clear();
    canvas.setTextSize(2);
    canvas.setTextColor(colorRingText);
    canvas.setTextDatum(BC_DATUM);
    for (int i = 1; stepNm > 0 && i * stepNm < rangeNm - 0.001f; i++) {
        float nm = i * stepNm;
        int r = (int)lroundf(nm / rangeNm * RADIUS);
        canvas.drawCircle(CENTER_X, CENTER_Y, r, colorRing);
        String text = nmText(nm);
        canvas.drawString(text, CENTER_X, CENTER_Y - r - 2);
        int w = canvas.textWidth(text);
        noteMark(CENTER_X - w / 2, CENTER_Y - r - 2 - LABEL_H, w, LABEL_H);
    }
    canvas.drawCircle(CENTER_X, CENTER_Y, RADIUS, colorRing);
    canvas.drawCircle(CENTER_X, CENTER_Y, RADIUS - 1, colorRing);
    canvas.drawString(nmText(rangeNm), CENTER_X, CENTER_Y - RADIUS - 2);

    static const char *POINTS[] = {"N", "E", "S", "W"};
    canvas.setTextColor(colorFaint);
    canvas.setTextDatum(MC_DATUM);
    for (int i = 0; i < 4; i++) {
        float a = i * (float)M_PI / 2.0f;
        int x = CENTER_X + (int)lroundf(sinf(a) * (RADIUS - COMPASS_INSET));
        int y = CENTER_Y - (int)lroundf(cosf(a) * (RADIUS - COMPASS_INSET));
        canvas.drawString(POINTS[i], x, y);
        int w = canvas.textWidth(POINTS[i]);
        noteMark(x - w / 2, y - LABEL_H / 2, w, LABEL_H);
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
// Says where the box went. Below the raster, which draws it.
LabelBox footerBox(const String &text, int edgeX, int textY, bool rightAligned);

// The range readout: "40 nm range   rings every 10 nm" - "40 nm auto range"
// while auto zoom has it, following.
String rangeText(int rangeNm, int step, bool autoRange) {
    return String(rangeNm) + (autoRange ? " nm auto range" : " nm range") +
           (step ? "   rings every " + String(step) + " nm" : String());
}

// The count in the corner: "1 contact", "40 contacts   31 labelled".
String contactsText(int plotted, int labelled) {
    return String(plotted) + (plotted == 1 ? " contact" : " contacts") +
           (labelled < plotted ? "   " + String(labelled) + " labelled" : "");
}

std::vector<LabelBox> g_trafficBoxes;  // see Background

// A line's pixels noted for the next frame to put back - see Background - as
// boxes along it rather than one round the whole: a long vector's box, on a
// slant, is mostly plot with nothing of it there, put back and pushed for
// nothing. Each piece as long as keeps what it takes in beside the line to
// about what a push of its own costs, 1600 pixels - see mergeBoxes() - and
// one for a line near level or upright. Its ends rounded to the pixel, a
// pixel either side takes in every pixel the line is drawn with.
void noteLine(int x0, int y0, int x1, int y1) {
    int dx = x1 - x0, dy = y1 - y0;
    int along = std::max(abs(dx), abs(dy)), across = std::min(abs(dx), abs(dy));
    int pieces = across ? std::max(1, (int)ceilf(along / sqrtf(1600.0f * along / across))) : 1;
    int ax = x0, ay = y0;
    for (int i = 1; i <= pieces; i++) {
        int bx = x0 + (int)lroundf((float)dx * i / pieces), by = y0 + (int)lroundf((float)dy * i / pieces);
        g_trafficBoxes.push_back({(int16_t)(std::min(ax, bx) - 1), (int16_t)(std::min(ay, by) - 1),
                                  (int16_t)(abs(bx - ax) + 3), (int16_t)(abs(by - ay) + 3)});
        ax = bx;
        ay = by;
    }
}

// The count last drawn, for a refresh of the traffic alone to tell whether
// the footer has changed.
String g_countShown;

// The readouts outside the plot's column as the live radar's last frame left
// them - the range readout's text, whose start is in the left margin, and the
// footer's count, under the legend: empty where the footer was left empty, out
// on the map - for a whole frame to leave them as they are where they haven't
// changed: see radarPlotDraw(). Not `kept` where something else - a replay,
// the zoom, a card or a list - may have drawn there since.
struct Footings {
    String range, count;
    bool kept = false;
};
Footings g_footings;

// Whether a box takes in any of the plot's column - which a whole frame fills
// afresh, and the traffic is put back and drawn in.
bool inColumn(const LabelBox &b) { return b.x < PLOT_R && b.x + b.w > PLOT_L; }

// The count drawn, in the footer. Long, its box reaches into the plot's
// column, which the footer's own erasing stops short of: noted with the
// traffic, so a refresh of the traffic alone puts back what is under it and
// a count grown shorter leaves nothing of the longer behind.
void drawCount(const String &text) {
    LabelBox b = footerBox(text, 1264, FOOTER_Y, true);
    g_countShown = text;
    if (b.x < PLOT_R) {
        g_trafficBoxes.push_back(b);
    }
}

// Font0's letters - 5 by 7, a byte a column - for the traffic's labels and the
// map's names, which are drawn straight into the canvas's pixels.
namespace glcd {
#include <lgfx/Fonts/glcdfont.h>
}


// The traffic drawn straight into the canvas's pixels - its blips, vectors,
// chevrons, rings and labels - by the canvas's own algorithms, ported line
// for line from LovyanGFX so the pixels are the same, without the cost of a
// call through it for each: in a busy sky the canvas's drawing was 47us a
// contact for the vector, blip and chevrons, and 106us a label.
struct Raster {
    uint16_t *px;
    int stride;
    int l, t, r, b;  // the clip, inclusive, as the canvas keeps it
    uint16_t ink;    // as the canvas stores a pixel, high byte first
    uint16_t colour;

    void use(uint16_t c) {
        colour = c;
        ink = __builtin_bswap16(c);
    }
    void hline(int x, int y, int w) {
        if (y < t || y > b || w <= 0) {
            return;
        }
        int x0 = std::max(x, l), x1 = std::min(x + w - 1, r);
        uint16_t *p = px + y * stride + x0;
        for (int i = x0; i <= x1; i++) {
            *p++ = ink;
        }
    }
    void vline(int x, int y, int h) {
        if (x < l || x > r || h <= 0) {
            return;
        }
        int y0 = std::max(y, t), y1 = std::min(y + h - 1, b);
        uint16_t *p = px + y0 * stride + x;
        for (int i = y0; i <= y1; i++, p += stride) {
            *p = ink;
        }
    }
    void rect(int x, int y, int w, int h) {
        for (int i = 0; i < h; i++) {
            hline(x, y + i, w);
        }
    }
    // LGFXBase::drawLine.
    void line(int x0, int y0, int x1, int y1) {
        bool steep = abs(y1 - y0) > abs(x1 - x0);
        int xstart = l, ystart = t, xend = r, yend = b;
        if (steep) {
            std::swap(xstart, ystart);
            std::swap(xend, yend);
            std::swap(x0, y0);
            std::swap(x1, y1);
        }
        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }
        if (x0 > xend || x1 < xstart) {
            return;
        }
        xend = std::min(x1, xend);
        int dy = abs(y1 - y0), ystep = y1 > y0 ? 1 : -1, dx = x1 - x0, err = dx >> 1;
        while (x0 < xstart || y0 < ystart || y0 > yend) {
            err -= dy;
            if (err < 0) {
                err += dx;
                y0 += ystep;
            }
            if (++x0 > xend) {
                return;
            }
        }
        int xs = x0, dlen = 0;
        if (ystep < 0) {
            std::swap(ystart, yend);
        }
        yend += ystep;
        do {
            ++dlen;
            if ((err -= dy) < 0) {
                steep ? vline(y0, xs, dlen) : hline(xs, y0, dlen);
                err += dx;
                xs = x0 + 1;
                dlen = 0;
                y0 += ystep;
                if (y0 == yend) {
                    break;
                }
            }
        } while (++x0 <= xend);
        if (dlen) {
            steep ? vline(y0, xs, dlen) : hline(xs, y0, dlen);
        }
    }
    // LGFXBase::fillCircle.
    void disc(int x, int y, int rad) {
        hline(x - rad, y, (rad << 1) + 1);
        if (rad <= 0) {
            return;
        }
        int f = 1 - rad, ddFy = -(rad << 1), ddFx = 1, i = 0;
        do {
            int len = 0;
            while (f < 0) {
                f += (ddFx += 2);
                ++len;
            }
            i += len;
            f += (ddFy += 2);
            if (len) {
                rect(x - rad, y + i - len + 1, (rad << 1) + 1, len);
            }
            hline(x - i, y + rad, (i << 1) + 1);
            hline(x - i, y - rad, (i << 1) + 1);
            if (len) {
                rect(x - rad, y - i, (rad << 1) + 1, len);
            }
        } while (i < --rad);
    }
    // LGFXBase::drawCircle.
    void ring(int x, int y, int rad) {
        if (rad <= 0) {
            hline(x, y, 1);
            return;
        }
        int f = 1 - rad, ddFy = -(rad << 1), ddFx = 1, i = 0, j = -1;
        do {
            while (f < 0) {
                ++i;
                f += (ddFx += 2);
            }
            f += (ddFy += 2);
            hline(x - i, y + rad, i - j);
            hline(x - i, y - rad, i - j);
            hline(x + j + 1, y - rad, i - j);
            hline(x + j + 1, y + rad, i - j);
            vline(x + rad, y + j + 1, i - j);
            vline(x + rad, y - i, i - j);
            vline(x - rad, y - i, i - j);
            vline(x - rad, y + j + 1, i - j);
            j = i;
        } while (i < --rad);
    }
    // Font0 at size 2, as the canvas draws it from the top left with no
    // background: a glyph's five columns of eight bits, each bit a 2x2 block,
    // on a cell 12 wide. A scan line at a time across the whole string, so
    // each line of the cache it lands in is filled once.
    void text(const String &s, int x, int y) {
        int n = s.length();
        const char *c = s.c_str();
        for (int row = 0; row < 8; row++) {
            for (int dy = 0; dy < 2; dy++) {
                int sy = y + row * 2 + dy;
                if (sy < t || sy > b) {
                    continue;
                }
                for (int k = 0; k < n; k++) {
                    const uint8_t *glyph = glcd::font + (uint8_t)c[k] * 5;
                    int gx = x + k * 12;
                    for (int col = 0; col < 5; col++) {
                        if (glyph[col] >> row & 1) {
                            hline(gx + col * 2, sy, 2);
                        }
                    }
                }
            }
        }
    }
    static int textWidth(const String &s) { return (int)s.length() * 12; }
};

// A box drawn through the raster as the canvas draws it - drawRect's four
// lines, and the text in Font0 at size 2 - in a fifth of the time; the
// canvas's text settings left as they were, for whatever is drawn next.
// Where footerBox() puts a box, without drawing it.
LabelBox footerBoxAt(const String &text, int edgeX, int textY, bool rightAligned) {
    int w = Raster::textWidth(text) + 2 * BOX_PAD_X;
    return {(int16_t)(rightAligned ? edgeX - w : edgeX), (int16_t)(textY - BOX_PAD_Y), (int16_t)w, (int16_t)BOX_H};
}

LabelBox footerBox(const String &text, int edgeX, int textY, bool rightAligned) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextColor(colorFaint);
    canvas.setTextDatum(rightAligned ? TR_DATUM : TL_DATUM);
    LabelBox at = footerBoxAt(text, edgeX, textY, rightAligned);
    int w = at.w;
    int x = at.x, y = at.y;
    Raster out = {(uint16_t *)canvas.getBuffer(), canvas.width(), 0, 0, canvas.width() - 1, canvas.height() - 1, 0, 0};
    out.use(colorRing);
    out.hline(x, y, w);
    out.hline(x, y + BOX_H - 1, w);
    out.vline(x, y + 1, BOX_H - 2);
    out.vline(x + w - 1, y + 1, BOX_H - 2);
    out.use(colorFaint);
    out.text(text, x + BOX_PAD_X, textY);
    return {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)BOX_H};
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
void drawTrend(Raster &out, int px, int py, const String &status) {
    int dir = 0;
    if (status == "CLIMB") {
        dir = -1;
    } else if (status == "DESCEND") {
        dir = 1;
    }
    if (dir == 0) {
        return;
    }
    constexpr int GAP = 9, HEIGHT = 5, HALF_W = 5;
    // Two strokes: a single line reads as thin and accidental beside a 5px blip.
    for (int i = 0; i <= 1; i++) {
        int base = py + dir * (GAP + i);
        int tip = py + dir * (GAP + HEIGHT + i);
        out.line(px - HALF_W, base, px, tip);
        out.line(px, tip, px + HALF_W, base);
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
// Out on the map's own ranges, from SPARSE_FROM_NM, the airports are thinned
// to the large ones and those no nearer another already drawn than
// SPARSE_PX, nearest the middle first: ALL at 750nm round Frankfurt was 316
// symbols and a wall of codes over the map.
constexpr int SPARSE_FROM_NM = 200;
constexpr float SPARSE_PX = 32;
// Whether the last drawAirports() put a symbol at the plot's middle - a free
// look opened on an airport, from its zoom - so the middle's own small cross
// isn't drawn over it.
bool g_airportAtCentre = false;

// Where an airport's label can go, in the order tried: under its symbol,
// over it, or to either side.
enum AirportSlot : uint8_t { AIRPORT_BELOW, AIRPORT_ABOVE, AIRPORT_RIGHT, AIRPORT_LEFT, AIRPORT_SLOT_COUNT };

void airportLabelAt(int slot, int x, int y, int w, int &lx, int &ly) {
    constexpr int OFF = AIRPORT_R + AIRPORT_TICK + 3;
    lx = slot == AIRPORT_RIGHT ? x + OFF : slot == AIRPORT_LEFT ? x - OFF - w : x - w / 2;
    ly = slot == AIRPORT_BELOW ? y + OFF : slot == AIRPORT_ABOVE ? y - OFF - LABEL_H : y - LABEL_H / 2;
}

// Where each airport's label went on the last plot - see PlacedNavaidLabel.
struct PlacedAirportLabel {
    String code;
    uint8_t slot;
};
std::vector<PlacedAirportLabel> g_airportLabelsWere, g_airportLabelsNow;

void drawAirports(double lat, double lon, int rangeNm, const String &centreCode, const String &originCode) {
    auto &canvas = screen::canvas();
    // Every symbol before any label, as the navaids are, so a label keeps
    // clear of the symbols of airports after it as well as before.
    struct Shown {
        String code;  // copied: airportsWithin()'s list is only lent
        int16_t x, y;
        int mark;  // its symbol's, in g_mapMarks
        bool origin;
        bool military;
    };
    static std::vector<Shown> shown;
    shown.clear();
    double nmPerDegLon = 60.0 * cos(lat * M_PI / 180.0);
    canvas.setTextSize(2);
    canvas.setTextColor(colorFaint);
    canvas.setTextDatum(TL_DATUM);
    bool sparse = rangeNm >= SPARSE_FROM_NM;
    size_t firstSpot = g_airfieldSpots.size();
    for (const Airport &a : airportsWithin(lat, lon, rangeNm)) {
        if (a.code == centreCode) {
            continue;  // the cross already marks it
        }
        float ox = (float)((a.lon - lon) * nmPerDegLon / rangeNm * RADIUS);
        float oy = (float)((a.lat - lat) * 60.0 / rangeNm * RADIUS);
        if (sparse) {
            const AirportInfo *info = airportInfo(a.code.c_str());
            if (!info || !info->large) {
                continue;
            }
            bool crowded = false;
            for (size_t i = firstSpot; i < g_airfieldSpots.size() && !crowded; i++) {
                crowded = fabsf(g_airfieldSpots[i].first - (CENTER_X + ox)) < SPARSE_PX &&
                          fabsf(g_airfieldSpots[i].second - (CENTER_Y - oy)) < SPARSE_PX;
            }
            if (crowded) {
                continue;
            }
        }
        int x = CENTER_X + (int)lroundf(ox);
        int y = CENTER_Y - (int)lroundf(oy);
        if (abs(x - CENTER_X) <= 2 && abs(y - CENTER_Y) <= 2) {
            g_airportAtCentre = true;
        }
        g_airfieldSpots.push_back({CENTER_X + ox, CENTER_Y - oy});
        bool origin = originCode.length() && a.code == originCode;
        drawAirfield(x, y, origin ? colorHome : a.military ? colorMilitaryField : colorFaint);
        constexpr int T1 = AIRPORT_R + AIRPORT_TICK;
        noteMark(x - T1, y - T1, 2 * T1 + 1, 2 * T1 + 1);
        shown.push_back({a.code, (int16_t)x, (int16_t)y, (int)g_mapMarks.size() - 1, origin, a.military});
    }

    // Then their labels, each where it was on the last plot while that is
    // still clear - so one going past a compass point as the map moves under
    // a following doesn't flick from side to side - or else under its
    // symbol, over it, or to either side. Clear of the other labels and of
    // what is on the scope - the compass points, the ring distances, the
    // centre's cross or the followed one's ring, the other airports' symbols,
    // all of which a code used to be written over; and where nowhere is, left
    // off - but for the airport a departure took off from, under its symbol.
    g_airportLabelsNow.clear();
    for (const Shown &sh : shown) {
        int x = sh.x, y = sh.y;
        int w = canvas.textWidth(sh.code);
        int was = -1;
        for (const PlacedAirportLabel &p : g_airportLabelsWere) {
            if (p.code == sh.code) {
                was = p.slot;
                break;
            }
        }
        LabelBox label = {(int16_t)x, (int16_t)y, 0, (int16_t)LABEL_H};
        bool placed = false;
        for (int k = -1; k < AIRPORT_SLOT_COUNT && g_mapLabelCount < MAX_MAP_LABELS && !placed; k++) {
            int slot = k < 0 ? was : k;
            if (slot < 0 || (k >= 0 && slot == was)) {
                continue;
            }
            int lx, ly;
            airportLabelAt(slot, x, y, w, lx, ly);
            if (lx < PLOT_L || lx + w > PLOT_R || ly < PLOT_T || ly + LABEL_H > PLOT_B) {
                continue;
            }
            int pad = slot == was ? KEPT_LABEL_PAD : LABEL_PAD;
            if (!overlapsAny(g_mapLabels, g_mapLabelCount, lx, ly, w, LABEL_H, pad) &&
                !overlapsAny(g_mapMarks.data(), (int)g_mapMarks.size(), lx, ly, w, LABEL_H, pad, sh.mark)) {
                label = {(int16_t)lx, (int16_t)ly, (int16_t)w, (int16_t)LABEL_H};
                g_airportLabelsNow.push_back({sh.code, (uint8_t)slot});
                placed = true;
            }
        }
        if (!placed && sh.origin && g_mapLabelCount < MAX_MAP_LABELS) {
            // Under its symbol, or over it off the bottom, and inside the
            // plot's sides.
            int lx, ly;
            airportLabelAt(AIRPORT_BELOW, x, y, w, lx, ly);
            if (ly + LABEL_H > PLOT_B) {
                airportLabelAt(AIRPORT_ABOVE, x, y, w, lx, ly);
            }
            lx = std::min(std::max(lx, PLOT_L), PLOT_R - w);
            label = {(int16_t)lx, (int16_t)ly, (int16_t)w, (int16_t)LABEL_H};
            placed = true;
        }
        if (placed) {
            canvas.setTextColor(sh.origin ? colorHome : sh.military ? colorMilitaryField : colorFaint);
            canvas.drawString(sh.code, label.x, label.y);
            g_mapLabels[g_mapLabelCount++] = label;
        }
        noteAirportHit(sh.code, x, y, label);
    }
    std::swap(g_airportLabelsWere, g_airportLabelsNow);
}

// A navaid as a chart draws one, small: a VOR a hexagon with a dot in it,
// boxed when there is a DME with it; a DME alone a box with a dot, beside an
// NDB a box round a ring of dots; a TACAN three tabs, round a VOR's hexagon
// for a VORTAC.
constexpr int NAVAID_R = 7, NAVAID_TAB = 3;
void drawNavaidSymbol(int x, int y, NavaidType type, uint16_t color) {
    auto &canvas = screen::canvas();
    // A flat-topped hexagon's corners, from east anticlockwise.
    int vx[6], vy[6];
    for (int i = 0; i < 6; i++) {
        float a = i * (float)M_PI / 3.0f;
        vx[i] = x + (int)lroundf(NAVAID_R * cosf(a));
        vy[i] = y - (int)lroundf(NAVAID_R * sinf(a));
    }
    bool hexagon = type == NavaidType::VOR || type == NavaidType::VOR_DME || type == NavaidType::VORTAC;
    bool box = type == NavaidType::VOR_DME || type == NavaidType::DME || type == NavaidType::NDB_DME;
    bool tabs = type == NavaidType::VORTAC || type == NavaidType::TACAN;
    if (hexagon) {
        for (int i = 0; i < 6; i++) {
            canvas.drawLine(vx[i], vy[i], vx[(i + 1) % 6], vy[(i + 1) % 6], color);
        }
    }
    if (box) {
        // Clear of a VOR's hexagon, which would otherwise touch it at its corners.
        int half = hexagon ? NAVAID_R + 3 : NAVAID_R;
        canvas.drawRect(x - half, y - half, 2 * half + 1, 2 * half + 1, color);
    }
    if (tabs) {
        // On the top edge and the two lower slanted ones, pushed outward.
        for (int i : {1, 3, 5}) {
            int ax = vx[i], ay = vy[i], bx = vx[(i + 1) % 6], by = vy[(i + 1) % 6];
            float mx = (ax + bx) / 2.0f - x, my = (ay + by) / 2.0f - y;
            float len = hypotf(mx, my);
            int ox = (int)lroundf(mx / len * NAVAID_TAB), oy = (int)lroundf(my / len * NAVAID_TAB);
            canvas.fillTriangle(ax, ay, bx, by, bx + ox, by + oy, color);
            canvas.fillTriangle(ax, ay, bx + ox, by + oy, ax + ox, ay + oy, color);
        }
    }
    if (type == NavaidType::NDB_DME) {
        for (int i = 0; i < 8; i++) {
            float a = i * (float)M_PI / 4.0f;
            canvas.drawPixel(x + (int)lroundf(3.5f * cosf(a)), y + (int)lroundf(3.5f * sinf(a)), color);
        }
    }
    canvas.fillCircle(x, y, 1, color);
}

// How far a navaid symbol reaches from its middle - a TACAN's tabs, or the
// box round a VOR-DME - and the gap between that and its label.
constexpr int NAVAID_REACH = NAVAID_R + NAVAID_TAB, NAVAID_LABEL_GAP = 4;

// Where a navaid's label can go, in the order tried.
enum NavaidSlot : uint8_t { SLOT_RIGHT, SLOT_LEFT, SLOT_ABOVE, SLOT_BELOW, SLOT_COUNT };

void navaidLabelAt(int slot, int x, int y, int w, int &lx, int &ly) {
    constexpr int OFF = NAVAID_REACH + NAVAID_LABEL_GAP;
    lx = slot == SLOT_RIGHT ? x + OFF : slot == SLOT_LEFT ? x - OFF - w : x - w / 2;
    ly = slot == SLOT_ABOVE ? y - OFF - LABEL_H : slot == SLOT_BELOW ? y + OFF : y - LABEL_H / 2;
}

// Where each navaid's label went on the last plot, to be tried there first on
// the next and let stay with less room around it. Following, the map moves
// under the centre and everything on it rounds to its own pixel, so a label
// with a pixel to spare came and went from frame to frame - as GOW's did beside
// GLA's code, all through a replay of a flight out of Glasgow.
struct PlacedNavaidLabel {
    const Navaid *navaid;
    uint8_t slot;
};
std::vector<PlacedNavaidLabel> g_navaidLabelsWere, g_navaidLabelsNow;

// The navaids in range, under the airports: dimmer, as the part of the map
// least often looked for, with ident and frequency beside each - to the right,
// or failing that the left, above or below - kept inside the outer ring and
// clear of every label already placed, every symbol on the map and, in the
// zoom, the runways. One with no room anywhere keeps its symbol only.
//
// One whose symbol would touch an airfield's is left off altogether: it is
// that airport's own, and at a range where the two can't be told apart. The
// airport's zoom, or a closer range, has room for it.
//
// Without `labels`, the symbols alone - still there to tap for a card.
void drawNavaids(double lat, double lon, float rangeNm, bool labels = true) {
    auto &canvas = screen::canvas();
    double nmPerDegLon = 60.0 * cos(lat * M_PI / 180.0);
    constexpr float AIRFIELD_CLEAR = AIRPORT_R + AIRPORT_TICK + NAVAID_REACH;

    // Every symbol before any label, so a near one's label can't go over one
    // further out.
    static std::vector<const Navaid *> navaids;
    static std::vector<std::pair<int16_t, int16_t>> spots;
    navaids.clear();
    spots.clear();
    int firstMark = (int)g_mapMarks.size();
    for (const Navaid *n : navaidsWithin(lat, lon, rangeNm)) {
        double dLon = n->lon - lon;
        if (dLon > 180) dLon -= 360;
        if (dLon < -180) dLon += 360;
        float ox = (float)(dLon * nmPerDegLon / rangeNm * RADIUS);
        float oy = (float)((n->lat - lat) * 60.0 / rangeNm * RADIUS);
        bool onAirfield = false;
        for (const auto &a : g_airfieldSpots) {
            if (fabsf(CENTER_X + ox - a.first) <= AIRFIELD_CLEAR && fabsf(CENTER_Y - oy - a.second) <= AIRFIELD_CLEAR) {
                onAirfield = true;
                break;
            }
        }
        if (onAirfield) {
            continue;
        }
        int x = CENTER_X + (int)lroundf(ox);
        int y = CENTER_Y - (int)lroundf(oy);
        drawNavaidSymbol(x, y, n->type, colorRingText);
        noteMark(x - NAVAID_REACH, y - NAVAID_REACH, 2 * NAVAID_REACH + 1, 2 * NAVAID_REACH + 1);
        navaids.push_back(n);
        spots.push_back({(int16_t)x, (int16_t)y});
    }

    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(colorRingText);
    g_navaidHitCount = 0;
    g_navaidLabelsNow.clear();
    for (size_t i = 0; i < navaids.size(); i++) {
        const Navaid *n = navaids[i];
        int x = spots[i].first, y = spots[i].second;
        LabelBox label = {0, (int16_t)(y - LABEL_H / 2), 0, (int16_t)LABEL_H};
        // Its label made only where one may be placed: a frequency is
        // newlib's double formatting and three Strings, for every navaid in
        // range - past 150nm, where none are labelled, for nothing.
        if (labels && g_mapLabelCount < MAX_MAP_LABELS) {
            String text = navaidLabel(*n);
            int w = canvas.textWidth(text);
            int was = -1;
            for (const PlacedNavaidLabel &p : g_navaidLabelsWere) {
                if (p.navaid == n) {
                    was = p.slot;
                    break;
                }
            }
            // Where it was first, then the rest in order.
            for (int k = -1; k < SLOT_COUNT && g_mapLabelCount < MAX_MAP_LABELS; k++) {
                int slot = k < 0 ? was : k;
                if (slot < 0 || (k >= 0 && slot == was)) {
                    continue;
                }
                int lx, ly;
                navaidLabelAt(slot, x, y, w, lx, ly);
                int pad = slot == was ? KEPT_LABEL_PAD : LABEL_PAD;
                if (insideScope(lx, ly, w, LABEL_H, pad) && !overlapsRunway(lx, ly, w, LABEL_H, pad) &&
                    !overlapsAny(g_mapLabels, g_mapLabelCount, lx, ly, w, LABEL_H, pad) &&
                    !overlapsAny(g_mapMarks.data(), (int)g_mapMarks.size(), lx, ly, w, LABEL_H, pad,
                                 firstMark + (int)i)) {
                    canvas.drawString(text, lx, ly);
                    label = {(int16_t)lx, (int16_t)ly, (int16_t)w, (int16_t)LABEL_H};
                    g_mapLabels[g_mapLabelCount++] = label;
                    g_navaidLabelsNow.push_back({n, (uint8_t)slot});
                    break;
                }
            }
        }
        if (g_navaidHitCount < MAX_NAVAID_HITS) {
            g_navaidHits[g_navaidHitCount++] = {(int16_t)x, (int16_t)y, label, n};
        }
    }
    std::swap(g_navaidLabelsWere, g_navaidLabelsNow);
}

// What a name on the map keeps off: the labels placed before it, the
// traffic's among them - a name half under a callsign was neither - and
// every mark on the plot: the ring distances and compass points, the centre,
// and the airfield and navaid symbols.
bool nameBlocked(int x, int y, int w, int h) {
    return overlapsAny(g_mapLabels, g_mapLabelCount, x, y, w, h) ||
           overlapsAny(g_contactLabels.data(), (int)g_contactLabels.size(), x, y, w, h) ||
           overlapsAny(g_mapMarks.data(), (int)g_mapMarks.size(), x, y, w, h);
}

// The names of the countries, regions, seas and lakes on the plot, as the
// last full draw placed them - see drawNames().
std::vector<PlacedName> g_names;

// The letters of g_names that fall inside x0..x1, y0..y1 (the far edges not
// included). Under everything else: only onto the plot's background and the
// map's own lines, so a contact, its vector or a trail crossing a name stays
// whole - and drawing a letter again where it is already changes nothing.
// In Font0's own letters, a pixel at a time - 5 by 7, a byte a column - which
// the canvas's text drawing has no way to put under what is there.
void drawNameLetters(int x0, int y0, int x1, int y1) {
    auto &canvas = screen::canvas();
    // As the canvas stores a pixel, high byte first - see drawCoast().
    uint16_t *pixels = (uint16_t *)canvas.getBuffer();
    int stride = canvas.width();
    const uint16_t under[] = {__builtin_bswap16(colorBg), __builtin_bswap16(colorCoast),
                              __builtin_bswap16(colorCoastAlone), __builtin_bswap16(colorRing)};
    for (const PlacedName &n : g_names) {
        if (n.x >= x1 || n.x + n.w <= x0 || n.y >= y1 || n.y + n.h <= y0) {
            continue;
        }
        uint16_t ink = __builtin_bswap16(n.kind == NameKind::SEA || n.kind == NameKind::LAKE ? colorWaterName
                                                                                           : colorLandName);
        int s = n.size;
        nameLetters(n, [&](char c, int x, int y) {
            if (x >= x1 || x + 5 * s <= x0 || y >= y1 || y + 8 * s <= y0) {
                return;
            }
            const uint8_t *glyph = glcd::font + (uint8_t)c * 5;
            for (int col = 0; col < 5; col++) {
                int left = std::max(x + col * s, x0), right = std::min(x + col * s + s, x1);
                for (int row = 0; row < 8; row++) {
                    if (!(glyph[col] >> row & 1)) {
                        continue;
                    }
                    int top = std::max(y + row * s, y0), bottom = std::min(y + row * s + s, y1);
                    for (int py = top; py < bottom; py++) {
                        uint16_t *p = pixels + py * stride + left;
                        for (int px = left; px < right; px++, p++) {
                            if (*p == under[0] || *p == under[1] || *p == under[2] || *p == under[3]) {
                                *p = ink;
                            }
                        }
                    }
                }
            }
        });
    }
}

// The names placed afresh - see names.h - and drawn. Drawn after everything
// but the traffic, to keep clear of its labels as they were when it was last
// drawn, which for a refresh of the same sky is where they are now. So where
// they go changes with the traffic, and the background a refresh puts back is
// kept without them: refreshTraffic() draws them again into each box of it.
void drawNames(double lat, double lon, float rangeNm) {
    if (!g_coastOn) {
        g_names.clear();
        return;
    }
    namesPlace({CENTER_X, CENTER_Y, RADIUS, lat, lon, rangeNm}, nameBlocked, g_names);
    drawNameLetters(PLOT_L, PLOT_T, PLOT_R, PLOT_B);
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
constexpr int LIVE_TEL_X = 16, LIVE_TEL_Y = 96;

void drawCentreButtons() {
    // None of them lit while following; HOME and AIRPORT none either dragged
    // off the centre they name, where ALL is still lit if every airport is
    // marked - see FreeLook.
    bool own = g_follow != FollowButton::ON && !g_radarLook.active;
    bool home = own && g_centre == RadarCentre::HOME;
    bool airport = own && g_centre == RadarCentre::AIRPORT;
    bool all = g_follow != FollowButton::ON && g_centre == RadarCentre::AIRPORT && g_airportsOn;
    drawButton(HOME_Y, "HOME", home ? Look::LIT : Look::PLAIN);
    drawButton(AIRPORT_Y, "AIRPORT", airport && !g_airportsOn ? Look::LIT : Look::PLAIN);
    drawButton(ALL_Y, "ALL", all ? Look::LIT : Look::PLAIN);
}

bool g_canZoomIn = false, g_canZoomOut = false;

// ZOOM IN or OUT, each after a magnifier with a plus or a minus in it, the
// two centred together on the button.
void drawZoomButton(int y, bool in, Look look) {
    drawButtonFrame(y, look);
    auto &canvas = screen::canvas();
    uint16_t color = look == Look::GREYED ? colorDisabled : look == Look::LIT ? colorBg : colorText;
    const char *label = in ? "ZOOM IN" : "ZOOM OUT";
    constexpr int ICON_W = 21, ICON_GAP = 8;  // the lens's left edge to the handle's end
    int left = BTN_X + (BTN_W - ICON_W - ICON_GAP - canvas.textWidth(label)) / 2;
    int cx = left + 8, cy = y + BTN_H / 2 - 3;
    canvas.drawCircle(cx, cy, 8, color);
    canvas.drawCircle(cx, cy, 7, color);
    canvas.drawFastHLine(cx - 4, cy, 9, color);
    if (in) {
        canvas.drawFastVLine(cx, cy - 4, 9, color);
    }
    for (int i = 0; i <= 1; i++) {
        canvas.drawLine(cx + 6 + i, cy + 6, cx + 11 + i, cy + 11, color);
    }
    canvas.setTextDatum(ML_DATUM);
    canvas.drawString(label, left + ICON_W + ICON_GAP, y + BTN_H / 2);
}

void drawZoomButtons() {
    drawZoomButton(ZOOM_IN_Y, true, g_canZoomIn ? Look::PLAIN : Look::GREYED);
    drawZoomButton(ZOOM_OUT_Y, false, g_canZoomOut ? Look::PLAIN : Look::GREYED);
}

// 3D, beside FOLLOW while following - or, not, the margin where it was.
bool g_3dShown = false;
void draw3dButton(bool following) {
    auto &canvas = screen::canvas();
    g_3dShown = following;
    canvas.fillRect(B3D_X, FOLLOW_Y, B3D_W, BTN_H, colorBg);
    if (following) {
        canvas.fillRoundRect(B3D_X, FOLLOW_Y, B3D_W, BTN_H, 6, colorBtnBg);
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(2);
        canvas.setTextDatum(MC_DATUM);
        canvas.setTextColor(colorText);
        canvas.drawString("3D", B3D_X + B3D_W / 2, FOLLOW_Y + BTN_H / 2);
    }
    screen::markDirty(B3D_X, FOLLOW_Y, B3D_W, BTN_H);
}

bool hit3dButton(int x, int y) {
    return g_3dShown && x >= B3D_X - 4 && x < B3D_X + B3D_W + 4 && y >= FOLLOW_Y - BTN_GAP / 2 &&
           y < FOLLOW_Y + BTN_H + BTN_GAP / 2;
}

void drawFollowButton() {
    drawButton(FOLLOW_Y, g_follow == FollowButton::PICKING ? "TAP PLANE" : "FOLLOW",
               g_follow == FollowButton::OFF ? Look::PLAIN : Look::LIT);
    draw3dButton(g_follow == FollowButton::ON);
}

// The sound's switch, at the end of the radar's and the zoom's titles, as the
// table has it in its header: the voice speaks on these screens, the zoom's
// clearances above all, and the table was the one place to quieten it.
constexpr int MUTE_CX = 268, MUTE_CY = 24, MUTE_R = 14, MUTE_HIT = 22;
bool g_soundMuted = false;

void drawMuteIcon() {
    auto &canvas = screen::canvas();
    canvas.fillRect(MUTE_CX - MUTE_R - 2, MUTE_CY - MUTE_R - 2, 2 * MUTE_R + 5, 2 * MUTE_R + 5, colorBg);
    uint16_t color = g_soundMuted ? colorDisabled : colorFaint;
    int r = MUTE_R, cx = MUTE_CX, cy = MUTE_CY;
    int boxW = (int)lroundf(r * 0.45f), boxH = (int)lroundf(r * 0.44f);
    int left = cx - r, coneX = left + boxW;
    int coneR = (int)lroundf(r * 0.30f), coneH = (int)lroundf(r * 0.92f);
    canvas.fillRect(left, cy - boxH / 2, boxW + 1, boxH, color);
    canvas.fillTriangle(coneX, cy - boxH / 2, coneX, cy + boxH / 2, cx + coneR, cy - coneH, color);
    canvas.fillTriangle(coneX, cy + boxH / 2, cx + coneR, cy - coneH, cx + coneR, cy + coneH, color);
    if (g_soundMuted) {
        for (int i = -1; i <= 1; i++) {
            canvas.drawLine(left + i, cy - coneH, cx + coneR + i, cy + coneH, color);
        }
    } else {
        for (int i = 1; i <= 2; i++) {
            int rr = coneR + i * (int)lroundf(r * 0.26f);
            canvas.drawArc(cx, cy, rr, rr + 2, 300, 360, color);
            canvas.drawArc(cx, cy, rr, rr + 2, 0, 60, color);
        }
    }
    screen::markDirty(MUTE_CX - MUTE_R - 2, MUTE_CY - MUTE_R - 2, 2 * MUTE_R + 5, 2 * MUTE_R + 5);
}

// The filter's way in from the radar, beside the speaker - out over the
// plot's corner, which its ring is well clear of up here: lit while a filter
// is on - see filter.h. Not on the zoom, which is never filtered.
constexpr int FILTER_CX = MUTE_CX + 46, FILTER_CY = MUTE_CY, FILTER_R = 13, FILTER_HIT = 22;
bool g_livePlot = false;  // set round the live radar's plot draws, for them to put the funnel back

// A funnel `r` across from its middle at (cx, cy), lit for a filter on.
void drawFunnel(int cx, int cy, int r, bool lit) {
    auto &canvas = screen::canvas();
    canvas.fillRect(cx - r - 2, cy - r - 2, 2 * r + 5, 2 * r + 5, colorBg);
    uint16_t color = lit ? colorText : colorFaint;
    int top = cy - (int)lroundf(r * 0.80f), neck = cy + (int)lroundf(r * 0.15f), bottom = cy + (int)lroundf(r * 0.85f);
    int spout = (int)lroundf(r * 0.22f);
    canvas.fillTriangle(cx - r, top, cx + r, top, cx - spout, neck, color);
    canvas.fillTriangle(cx + r, top, cx + spout, neck, cx - spout, neck, color);
    canvas.fillRect(cx - spout, neck, 2 * spout + 1, bottom - neck, color);
    screen::markDirty(cx - r - 2, cy - r - 2, 2 * r + 5, 2 * r + 5);
}

void drawFilterIcon() { drawFunnel(FILTER_CX, FILTER_CY, FILTER_R, g_filtered); }

// The time - in UTC as aviation keeps it, "14:32Z", or a zone's, "14:32 BST"
// - in the margin left of the plot, above its readouts, on the radar and the
// zoom: for a tracker left on a desk or a wall. A setting on the display
// page; redrawn as the minute turns over, by radarScreenTickClock().
constexpr int CLOCK_Y = 588, CLOCK_H = 32;
ClockZone g_clockZone = ClockZone::UTC;
int g_clockMinute = -1;
int g_clockAt = -1;  // following: the zone the aircraft is in, as timeZoneAt() has it; -1 for home's

void drawClock() {
    auto &canvas = screen::canvas();
    canvas.fillRect(0, CLOCK_Y - CLOCK_H / 2, PLOT_L, CLOCK_H, colorBg);
    time_t now = time(nullptr);
    if (g_clockZone != ClockZone::OFF && now > 1600000000) {  // set from the network, not 1970
        struct tm utc;
        gmtime_r(&now, &utc);
        char text[16];
        // Following: the time where the aircraft is, by a blip for it.
        bool aircraft = g_clockAt >= 0 && g_clockZone != ClockZone::UTC;
        if (aircraft) {
            timeZoneFormat(now, g_clockAt, text, sizeof(text));
            canvas.fillCircle(22, CLOCK_Y, 5, colorBlip);
        } else {
            clockZoneFormat(now, g_clockZone, text, sizeof(text));
        }
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(3);
        canvas.setTextDatum(ML_DATUM);
        canvas.setTextColor(aircraft ? colorText : colorFaint);
        canvas.drawString(text, aircraft ? 36 : 16, CLOCK_Y);
        g_clockMinute = utc.tm_hour * 60 + utc.tm_min;
    }
    screen::markDirty(0, CLOCK_Y - CLOCK_H / 2, PLOT_L, CLOCK_H);
}

// "Radar - CIV" - no closing dash, which ran up against the speaker after it - and under it what following is doing: asking for an
// aircraft to be tapped, or naming the one followed.
void drawTitle() {
    auto &canvas = screen::canvas();
    canvas.fillRect(0, 0, PLOT_L, TITLE_BLOCK_H, colorBg);
    // Set here rather than trusted from whatever drew last: the table leaves
    // its cells' FreeSans selected, and at size 3 the title came out huge.
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(colorText);
    const char *title = "Radar";
    canvas.drawString(title, 16, TITLE_Y);
    canvas.setTextColor(colorMuted);
    // What it shows, by kind, as the table's title has it in words: two of
    // them - "CRG+MIL" - a size down, to stay clear of the speaker.
    String kinds = shownKinds(true);
    int x = 16 + canvas.textWidth(title);
    if (kinds.length() > 3) {
        canvas.drawString(" - ", x, TITLE_Y);
        x += canvas.textWidth(" - ");
        canvas.setTextSize(2);
        canvas.drawString(kinds, x, TITLE_Y);
    } else {
        canvas.drawString(" - " + kinds, x, TITLE_Y);
    }
    canvas.setTextSize(2);
    if (g_follow == FollowButton::PICKING) {
        canvas.drawString("tap a plane to follow", 16, TITLE_LINE2_Y);
    } else if (g_follow == FollowButton::ON) {
        const char *what = "following ";
        canvas.drawString(what, 16, TITLE_LINE2_Y);
        canvas.setTextColor(colorHome);
        canvas.drawString(g_followCallsign, 16 + canvas.textWidth(what), TITLE_LINE2_Y);
    } else if (g_mapOnly) {
        canvas.drawString("map only", 16, TITLE_LINE2_Y);
    } else if (g_radarLook.active) {
        canvas.drawString(g_filtered ? "free look, filtered" : "free look", 16, TITLE_LINE2_Y);
    } else if (g_filtered) {
        canvas.drawString("filtered", 16, TITLE_LINE2_Y);
    }
    drawMuteIcon();
    drawFilterIcon();
    screen::markDirty(0, 0, PLOT_L, TITLE_BLOCK_H);
}

// Title and buttons: drawn on a full repaint, and again in part as the
// controls change.
void drawHeader() {
    drawTitle();
    drawButton(BACK_Y, "Back", Look::PLAIN);
    drawCentreButtons();
    drawFollowButton();
    drawRecButton();
    drawReplaysButton();
    drawZoomButtons();
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
//
// A navaid picked, or tapped on its own, puts up its card the same way:
// what it is and how to tune it, until the next tap anywhere.

constexpr int MAX_CHOICES = 6;
constexpr int CHOICE_W = 400;
constexpr int CHOICE_PAD = 12, CHOICE_TITLE_H = 32, CHOICE_ROW_H = 52, CHOICE_GAP = 8;
constexpr int CHOICE_OFFSET = 40;  // from the tap to the near side of the list

struct Choice {
    enum Kind : uint8_t { AIRCRAFT, AIRPORT, NAVAID } kind;
    String id;  // the airport's IATA code, or the contact's ICAO hex
    String text;
    bool ground;
    const Navaid *navaid;
};
Choice g_choices[MAX_CHOICES];
int g_choiceCount = 0;  // 0 while no list is up
int g_choiceX = 0, g_choiceY = 0, g_choiceH = 0;

int choiceRowY(int i) { return g_choiceY + CHOICE_PAD + CHOICE_TITLE_H + i * (CHOICE_ROW_H + CHOICE_GAP); }

// The navaid card up, or nullptr. Picked from the list, it is put up by the
// full redraw that takes the list off, which would otherwise close it too.
const Navaid *g_card = nullptr;
bool g_cardSurvivesRedraw = false;
int g_cardX = 0, g_cardY = 0, g_cardH = 0;
constexpr int CARD_LINE_H = 26;

bool overlayUp();  // a list, or either card: an airport's is further down

// Beside a tap at (x, y), as openChooser() places the list.
void placeBeside(int x, int y, int h, int &outX, int &outY) {
    constexpr int RIGHT = BTN_X - 16;
    outX = (x + CHOICE_OFFSET + CHOICE_W <= RIGHT) ? x + CHOICE_OFFSET : x - CHOICE_OFFSET - CHOICE_W;
    outX = std::min(std::max(outX, 16), RIGHT - CHOICE_W);
    outY = std::min(std::max(y - h / 2, PLOT_T + 4), 720 - 8 - h);
}

// The card's lines under its title: what it is, then what to tune.
std::vector<String> cardLines(const Navaid &n) {
    std::vector<String> lines;
    lines.push_back(navaidTypeName(n.type));
    String freq = navaidFrequency(n);
    if (n.type == NavaidType::NDB_DME) {
        if (freq.length()) {
            lines.push_back("NDB " + freq + " kHz");
        }
        if (n.channel[0]) {
            lines.push_back(String("DME channel ") + n.channel);
        }
    } else if (n.type == NavaidType::TACAN) {
        if (n.channel[0]) {
            lines.push_back(String("Channel ") + n.channel);
        }
        if (freq.length()) {
            lines.push_back("Paired VHF " + freq + " MHz");
        }
    } else {
        if (freq.length()) {
            lines.push_back(freq + " MHz");
        }
        if (n.channel[0]) {
            lines.push_back(String(n.type == NavaidType::VORTAC ? "TACAN channel " : "DME channel ") + n.channel);
        }
    }
    if (n.airport[0]) {
        lines.push_back(String("Serves ") + n.airport);
    }
    return lines;
}

void drawCard() {
    const Navaid &n = *g_card;
    std::vector<String> lines = cardLines(n);
    auto &canvas = screen::canvas();
    canvas.clearClipRect();
    canvas.setFont(&fonts::Font0);
    canvas.fillRoundRect(g_cardX, g_cardY, CHOICE_W, g_cardH, 10, colorBg);
    canvas.drawRoundRect(g_cardX, g_cardY, CHOICE_W, g_cardH, 10, colorRing);
    canvas.drawRoundRect(g_cardX + 1, g_cardY + 1, CHOICE_W - 2, g_cardH - 2, 9, colorRing);
    int x = g_cardX + CHOICE_PAD + 4;
    int y = g_cardY + CHOICE_PAD + 4;
    drawNavaidSymbol(x + NAVAID_R + NAVAID_TAB, y + 12, n.type, colorText);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextSize(3);
    canvas.setTextColor(colorText);
    int tx = x + 2 * (NAVAID_R + NAVAID_TAB) + 12;
    canvas.drawString(n.ident, tx, y);
    y += 40;
    // Names run to 31 characters, a little more than the card takes at size 2.
    canvas.setTextSize(2);
    canvas.setTextColor(colorMuted);
    if (canvas.textWidth(n.name) > CHOICE_W - 2 * (x - g_cardX)) {
        canvas.setTextSize(1);
    }
    canvas.drawString(n.name, x, y);
    canvas.setTextSize(2);
    y += CARD_LINE_H + 4;
    canvas.setTextColor(colorText);
    for (const String &line : lines) {
        canvas.drawString(line, x, y);
        y += CARD_LINE_H;
    }
    canvas.setTextColor(colorFaint);
    canvas.drawString("Tap anywhere to close", x, y + 6);
    screen::markDirty(g_cardX, g_cardY, CHOICE_W, g_cardH);
}

// Puts the card up for `n`, beside where it is on the plot.
void openCard(const Navaid *n, int x, int y) {
    g_card = n;
    soundReadNavaid(*n);  // if read aloud is on
    g_cardH = 2 * CHOICE_PAD + 8 + 40 + CARD_LINE_H + 4 + (int)cardLines(*n).size() * CARD_LINE_H + 6 + LABEL_H;
    placeBeside(x, y, g_cardH, g_cardX, g_cardY);
    // Over the footer or the range readout, it may be, and taken off again
    // before the radar draws another frame: they are drawn afresh then.
    g_footings.kept = false;
}

// --- an airport's card ----------------------------------------------------------
// INFO, in the zoom, puts up the card of the airport zoomed on: its name and
// codes, how high it is, and the frequencies to hear it on - its tower, the
// approach and the radar around it, the control centre over it and the rest,
// as far as OurAirports lists them - beside a radar on its mast, sending.
// Over the plot until the next tap anywhere, as a navaid's card is.
//
// The frequencies are the airport's and have no place of their own on the
// map, which is why they are on a card and not marked on the plot.

constexpr int INFO_W = 560, INFO_PAD = 16;
constexpr int MAST_W = 112, MAST_H = 116;  // the picture's box
constexpr int INFO_ROW_H = 28;
// A row is what the frequencies are for, then up to four of them.
constexpr int INFO_LABEL_W = 132, INFO_FREQ_W = 96;
// How much of a name fits a line beside the picture, at 12px a character.
constexpr int INFO_NAME_CHARS = (INFO_W - 3 * INFO_PAD - MAST_W) / 12;

const AirportInfo *g_airportCard = nullptr;
int g_airportCardX = 0, g_airportCardY = 0, g_airportCardH = 0;

bool overlayUp() { return g_choiceCount || g_card || g_airportCard; }

// A radar on a lattice mast - its dish tilted up, the feed out in front of
// it, and three arcs of what it sends going out, each fainter than the last
// - in a box MAST_W by MAST_H with its corner at (x, y).
void drawTransmitter(int x, int y) {
    auto &canvas = screen::canvas();
    auto line = [&](float x0, float y0, float x1, float y1, uint16_t color) {
        canvas.drawLine(x + (int)lroundf(x0), y + (int)lroundf(y0), x + (int)lroundf(x1), y + (int)lroundf(y1),
                        color);
    };

    // The mast: two legs closing in towards its top, braced straight across
    // and corner to corner, on a line of ground.
    constexpr float BASE_Y = 110, TOP_Y = 76, LEFT = 26, RIGHT = 62, LEAN = 12;
    auto leg = [&](float atY, bool right) {
        float in = (BASE_Y - atY) / (BASE_Y - TOP_Y) * LEAN;
        return right ? RIGHT - in : LEFT + in;
    };
    line(12, BASE_Y, 78, BASE_Y, colorFaint);
    line(LEFT, BASE_Y, LEFT + LEAN, TOP_Y, colorMuted);
    line(RIGHT, BASE_Y, RIGHT - LEAN, TOP_Y, colorMuted);
    constexpr float BARS[] = {BASE_Y, BASE_Y - 13, BASE_Y - 24, TOP_Y};
    for (int i = 0; i < 4; i++) {
        line(leg(BARS[i], false), BARS[i], leg(BARS[i], true), BARS[i], colorMuted);
        if (i < 3) {
            line(leg(BARS[i], false), BARS[i], leg(BARS[i + 1], true), BARS[i + 1], colorFaint);
            line(leg(BARS[i], true), BARS[i], leg(BARS[i + 1], false), BARS[i + 1], colorFaint);
        }
    }
    // The turning gear on top of it, and what holds the dish to that.
    constexpr float VX = 44, VY = 58;  // the middle of the dish
    canvas.fillRect(x + 40, y + 68, 9, 8, colorMuted);
    for (float footX : {40.0f, 44.0f, 48.0f}) {
        line(VX, VY, footX, 68, colorMuted);
    }

    // The dish, side on: a parabola 52px across, tilted 40 degrees up to the
    // right, drawn a few times over just behind itself for its thickness.
    constexpr float HALF = 26, DEPTH = 56;
    const float tilt = 40.0f * (float)M_PI / 180.0f;
    const float axisX = cosf(tilt), axisY = -sinf(tilt), acrossX = sinf(tilt), acrossY = cosf(tilt);
    auto dish = [&](float t, float back, float &outX, float &outY) {
        float along = t * t / DEPTH - back;
        outX = VX + along * axisX + t * acrossX;
        outY = VY + along * axisY + t * acrossY;
    };
    for (float back : {0.0f, 0.7f, 1.4f, 2.1f}) {
        float fromX, fromY;
        dish(-HALF, back, fromX, fromY);
        for (int i = 1; i <= 26; i++) {
            float toX, toY;
            dish(-HALF + 2 * HALF * i / 26, back, toX, toY);
            line(fromX, fromY, toX, toY, colorText);
            fromX = toX;
            fromY = toY;
        }
    }
    // Its feed, at the parabola's focus, on a strut from each rim.
    float focusX = VX + DEPTH / 4 * axisX, focusY = VY + DEPTH / 4 * axisY;
    for (float t : {-HALF, HALF}) {
        float rimX, rimY;
        dish(t, 0, rimX, rimY);
        line(rimX, rimY, focusX, focusY, colorMuted);
    }
    canvas.fillCircle(x + (int)lroundf(focusX), y + (int)lroundf(focusY), 2, colorText);

    // What it sends: arcs across the way it points, two pixels thick.
    const uint16_t shades[] = {colorBlip, colorMuted, colorFaint};
    for (int w = 0; w < 3; w++) {
        for (int thick = 0; thick < 2; thick++) {
            float r = 20 + 11 * w + thick;
            float fromX = 0, fromY = 0;
            for (int i = 0; i <= 16; i++) {
                float a = -tilt + (-30.0f + 60.0f * i / 16) * (float)M_PI / 180.0f;
                float toX = focusX + r * cosf(a), toY = focusY + r * sinf(a);
                if (i) {
                    line(fromX, fromY, toX, toY, shades[w]);
                }
                fromX = toX;
                fromY = toY;
            }
        }
    }
}

// How many rows of frequencies its card has: one for each use it has any for.
int airportCardRows(const AirportInfo &a) {
    const AirportFrequency *f = airportFrequencies(a);
    int rows = 0;
    for (int i = 0; i < a.frequencyCount; i++) {
        rows += i == 0 || f[i].use != f[i - 1].use;
    }
    return rows;
}

void drawAirportCard() {
    const AirportInfo &a = *g_airportCard;
    auto &canvas = screen::canvas();
    canvas.clearClipRect();
    canvas.setFont(&fonts::Font0);
    int left = g_airportCardX + INFO_PAD, top = g_airportCardY + INFO_PAD;
    canvas.fillRoundRect(g_airportCardX, g_airportCardY, INFO_W, g_airportCardH, 10, colorBg);
    canvas.drawRoundRect(g_airportCardX, g_airportCardY, INFO_W, g_airportCardH, 10, colorRing);
    canvas.drawRoundRect(g_airportCardX + 1, g_airportCardY + 1, INFO_W - 2, g_airportCardH - 2, 9, colorRing);
    drawTransmitter(left, top);

    // Beside the picture: its codes, its name in up to two lines, its height.
    int tx = left + MAST_W + INFO_PAD, ty = top + 6;
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextSize(3);
    canvas.setTextColor(colorText);
    canvas.drawString(a.code, tx, ty);
    int codeW = canvas.textWidth(a.code);
    canvas.setTextSize(2);
    if (a.icao[0]) {
        canvas.setTextColor(colorMuted);
        canvas.drawString(a.icao, tx + codeW + 16, ty + 8);  // on the larger one's baseline
    }
    ty += 36;
    canvas.setTextColor(colorMuted);
    String name(a.name);
    for (int lineNo = 0; lineNo < 2 && name.length(); lineNo++) {
        String part = name;
        if ((int)name.length() > INFO_NAME_CHARS) {
            // Broken at a space where the first line has one; the second is
            // cut short, for the few names two lines won't take.
            int space = lineNo == 0 ? name.lastIndexOf(' ', INFO_NAME_CHARS) : -1;
            part = space > 0 ? name.substring(0, space) : name.substring(0, INFO_NAME_CHARS - 2) + "..";
            name = space > 0 ? name.substring(space + 1) : String();
        } else {
            name = String();
        }
        canvas.drawString(part, tx, ty);
        ty += 22;
    }
    canvas.setTextColor(colorFaint);
    canvas.drawString("Elevation " + String(a.elevationFt) + " ft", tx, ty + 4);

    // Under it, a row for each use: TOWER 118.500 118.700 124.475.
    int y = top + MAST_H + 10;
    canvas.drawFastHLine(left, y, INFO_W - 2 * INFO_PAD, colorRing);
    y += 12;
    const AirportFrequency *f = airportFrequencies(a);
    for (int i = 0; i < a.frequencyCount;) {
        uint8_t use = f[i].use;
        canvas.setTextColor(colorMuted);
        canvas.drawString(atcUseName((AtcUse)use), left, y);
        canvas.setTextColor(colorText);
        for (int column = 0; i < a.frequencyCount && f[i].use == use; i++, column++) {
            char mhz[12];
            snprintf(mhz, sizeof(mhz), "%u.%03u", (unsigned)(f[i].khz / 1000), (unsigned)(f[i].khz % 1000));
            canvas.drawString(mhz, left + INFO_LABEL_W + column * INFO_FREQ_W, y);
        }
        y += INFO_ROW_H;
    }
    canvas.setTextColor(colorFaint);
    if (a.frequencyCount == 0) {
        canvas.drawString("No frequencies are listed for it", left, y);
        y += INFO_ROW_H;
        canvas.drawString("Tap anywhere to close", left, y + 6);
    } else {
        canvas.drawString("Frequencies in MHz - tap anywhere to close", left, y + 6);
    }
    screen::markDirty(g_airportCardX, g_airportCardY, INFO_W, g_airportCardH);
}

// Puts the card up for the airport with IATA `code`, in the middle of the
// plot - or leaves things as they are, for one the table hasn't got.
void openAirportCard(const String &code) {
    const AirportInfo *a = airportInfo(code.c_str());
    if (a == nullptr) {
        return;
    }
    g_airportCard = a;
    soundReadAirport(*a);  // if read aloud is on
    g_airportCardH = 2 * INFO_PAD + MAST_H + 10 + 12 + std::max(airportCardRows(*a), 1) * INFO_ROW_H + 6 + LABEL_H;
    g_airportCardX = CENTER_X - INFO_W / 2;
    g_airportCardY = std::min(std::max(CENTER_Y - g_airportCardH / 2, 8), 720 - 8 - g_airportCardH);
}

// --- free look -------------------------------------------------------------------

bool insideRing(int x, int y) {
    long dx = x - CENTER_X, dy = y - CENTER_Y;
    return dx * dx + dy * dy <= (long)RADIUS * RADIUS;
}

void startDrag(int x, int y, bool zoom) {
    g_drag = Drag();
    g_drag.down = true;
    g_drag.zoom = zoom;
    g_drag.startX = x;
    g_drag.startY = y;
    g_drag.view = zoom ? g_zoomView : g_radarView;
}

// The line a drag has stretched out from the middle of the plot, a ring on
// the end of it where the middle would go, and beside that how far it is and
// which way. Over the plot, once that has been drawn, at the range that was
// drawn at.
// What drawDragLine() last drew over: the line with the ring on its end, and
// the box beside it - in the plot's column, where both are kept.
LabelBox g_dragDrawn[2] = {};

LabelBox clippedToPlot(int x0, int y0, int x1, int y1) {
    x0 = std::max(x0, PLOT_L);
    y0 = std::max(y0, PLOT_T);
    x1 = std::min(x1, PLOT_R);
    y1 = std::min(y1, PLOT_B);
    return {(int16_t)x0, (int16_t)y0, (int16_t)std::max(0, x1 - x0), (int16_t)std::max(0, y1 - y0)};
}

void drawDragLine(float rangeNm) {
    auto &canvas = screen::canvas();
    canvas.clearClipRect();
    int dx = (int)lroundf(g_drag.eastNm / rangeNm * RADIUS), dy = -(int)lroundf(g_drag.northNm / rangeNm * RADIUS);
    int ex = CENTER_X + dx, ey = CENTER_Y + dy;
    // Two pixels wide, the second beside the first across the way it runs.
    bool steep = abs(dy) > abs(dx);
    canvas.drawLine(CENTER_X, CENTER_Y, ex, ey, colorHome);
    canvas.drawLine(CENTER_X + steep, CENTER_Y + !steep, ex + steep, ey + !steep, colorHome);
    canvas.fillCircle(ex, ey, 11, colorBg);
    canvas.drawCircle(ex, ey, 11, colorHome);
    canvas.drawCircle(ex, ey, 10, colorHome);
    canvas.drawLine(ex - 5, ey, ex + 5, ey, colorHome);
    canvas.drawLine(ex, ey - 5, ex, ey + 5, colorHome);

    // 0.35 in the zoom, 12.4 on the radar, 105 across the whole of it.
    char text[32];
    int bearing = ((int)lroundf(g_drag.bearing) % 360 + 360) % 360;
    snprintf(text, sizeof(text), "%.*f nm  BRG %03d", g_drag.nm < 1 ? 2 : g_drag.nm < 20 ? 1 : 0, g_drag.nm, bearing);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    int w = canvas.textWidth(text) + 2 * BOX_PAD_X;
    // Beside the ring, on the side with room for it, and inside the plot's
    // column, which is all of the screen a refresh pushes.
    int bx = ex + 18 + w <= PLOT_R ? ex + 18 : ex - 18 - w;
    bx = std::min(std::max(bx, PLOT_L), PLOT_R - w);
    int by = std::min(std::max(ey - BOX_H / 2, PLOT_T + 4), PLOT_B - 4 - BOX_H);
    canvas.fillRect(bx, by, w, BOX_H, colorBg);
    canvas.drawRect(bx, by, w, BOX_H, colorHome);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(colorText);
    canvas.drawString(text, bx + BOX_PAD_X, by + BOX_PAD_Y);
    g_dragDrawn[0] = clippedToPlot(std::min(CENTER_X, ex) - 13, std::min(CENTER_Y, ey) - 13, std::max(CENTER_X, ex) + 14,
                                   std::max(CENTER_Y, ey) + 14);
    g_dragDrawn[1] = clippedToPlot(bx, by, bx + w, by + BOX_H);
}

// The radar's plot as it was when a drag's line went out over it, for the
// line to be moved over it a finger's move at a time: the plot itself doesn't
// change while the line is out - see radarDragTo() - so where the line was
// is put back from this, the line drawn where it is now, and only those two
// pushed. Each move redrew and pushed the whole plot, 59ms of it at 60nm -
// 17 a second, behind the finger. In PSRAM: the plot's column, 1MB.
constexpr int PLOT_W = PLOT_R - PLOT_L, PLOT_H = PLOT_B - PLOT_T;
struct DragCache {
    uint16_t *pixels = nullptr;
    bool valid = false;
    bool zoom = false;  // the airport zoom's, whose drags work the same way
    float rangeNm = 0;
    double lat = 0, lon = 0;
};
DragCache g_dragCache;

void keepPlotForDrag(bool zoom, float rangeNm, double lat, double lon) {
    if (!g_dragCache.pixels) {
        g_dragCache.pixels = (uint16_t *)heap_caps_malloc((size_t)PLOT_W * PLOT_H * 2, MALLOC_CAP_SPIRAM);
        if (!g_dragCache.pixels) {
            return;  // the slow way, then
        }
    }
    const uint16_t *canvas = (const uint16_t *)screen::canvas().getBuffer();
    int stride = screen::canvas().width();
    for (int y = 0; y < PLOT_H; y++) {
        memcpy(g_dragCache.pixels + (size_t)y * PLOT_W, canvas + (size_t)(PLOT_T + y) * stride + PLOT_L, PLOT_W * 2);
    }
    g_dragCache = {g_dragCache.pixels, true, zoom, rangeNm, lat, lon};
}

// The plot without its traffic or names - map, rings, airports and navaids,
// as a full draw left them just before the names went on - so a refresh in
// which only the traffic has moved puts back what was under it, with the
// names, draws it where it is now and pushes those places, rather than
// drawing and pushing the whole plot: see radarPlotDraw(). The names are left
// out as where they go moves with the traffic. Another 1MB of PSRAM. Taken on the
// second full draw of the same view, so a tap's own redraw isn't slowed by the
// copy, and kept for as long as the view is the same. The plot is drawn whole
// all the same at least every NAMES_MAX_MS, for the names to be placed afresh,
// clear of where the labels have moved to since.
struct BackgroundKey {
    double lat, lon, homeLat, homeLon;
    float zoomNm;  // the zoom's range, which unlike the radar's comes in fractions
    int rangeNm, mapLevels;
    uint32_t runwaysInUse, runwaysRecent;  // the zoom's runway ends marked, a bit each
    bool zoom, coast, rings, navaids, airports, everyAirport, freeLook, markHome, mapOnly;
    char centreCode[8];
};
struct Background {
    uint16_t *pixels = nullptr;
    bool valid = false;
    bool pending = false;  // a full draw of `key` done: the next one takes the copy
    BackgroundKey key;
    uint32_t wholeMs = 0;  // when the plot was last drawn whole
};
Background g_background;
constexpr uint32_t NAMES_MAX_MS = 60000;
// Where the traffic was drawn - the last frame's, and this one's - to put
// back from the background, and to push.
std::vector<LabelBox> g_trafficWas;

bool sameBackground(const BackgroundKey &a, const BackgroundKey &b) { return memcmp(&a, &b, sizeof(a)) == 0; }

void keepBackground() {
    if (!g_background.pixels) {
        g_background.pixels = (uint16_t *)heap_caps_malloc((size_t)PLOT_W * PLOT_H * 2, MALLOC_CAP_SPIRAM);
        if (!g_background.pixels) {
            return;
        }
    }
    const uint16_t *canvas = (const uint16_t *)screen::canvas().getBuffer();
    int stride = screen::canvas().width();
    for (int y = 0; y < PLOT_H; y++) {
        memcpy(g_background.pixels + (size_t)y * PLOT_W, canvas + (size_t)(PLOT_T + y) * stride + PLOT_L, PLOT_W * 2);
    }
    g_background.valid = true;
}

// Whether putting back the last frame's boxes and pushing them and this
// frame's would beat drawing the plot whole, as measured: about 2us a row of
// a box's lines of cache to put back, and a push's 0.085ms and 53ns a pixel -
// a box where it was and where it is, a pixel or two along, being one push
// of up to twice its pixels. Drawn whole, a plot's background and push are
// some 39ms besides its traffic: past 32ms - fifty aircraft or so - a crowd
// is drawn whole, the background still kept.
bool trafficCheap() {
    float cost = 0;
    for (const LabelBox &b : g_trafficBoxes) {
        cost += b.h * (b.w / 32 + 2) * 0.0007f + 0.085f + 2 * b.w * b.h * 0.000053f;
    }
    return cost < 32.0f;
}

// A box of the plot put back as the background has it.
void restoreBackground(const LabelBox &b) {
    int x0 = std::max<int>(b.x, PLOT_L), x1 = std::min<int>(b.x + b.w, PLOT_R);
    int y0 = std::max<int>(b.y, PLOT_T), y1 = std::min<int>(b.y + b.h, PLOT_B);
    if (x1 <= x0 || y1 <= y0) {
        return;
    }
    uint16_t *canvas = (uint16_t *)screen::canvas().getBuffer();
    int stride = screen::canvas().width();
    for (int y = y0; y < y1; y++) {
        memcpy(canvas + (size_t)y * stride + x0, g_background.pixels + (size_t)(y - PLOT_T) * PLOT_W + (x0 - PLOT_L),
               (size_t)(x1 - x0) * 2);
    }
}

// Boxes run together where one push of the two costs less than two: a push
// is 0.085ms and 53ns a pixel, so where the box round both takes in fewer
// than 1600 pixels more than the two do apart - a contact's box where it was
// and where it is, say, but not the pieces of a slanting vector (see
// noteLine()), nor a crowd. Only boxes within NEAR of each other are weighed.
void mergeBoxes(std::vector<screen::Region> &r) {
    constexpr int NEAR = 32, EXTRA = 1600;
    bool merged = true;
    while (merged) {
        merged = false;
        for (size_t i = 0; i < r.size(); i++) {
            for (size_t j = i + 1; j < r.size();) {
                const screen::Region &a = r[i], &b = r[j];
                if (a.x - NEAR < b.x + b.w && b.x - NEAR < a.x + a.w && a.y - NEAR < b.y + b.h && b.y - NEAR < a.y + a.h) {
                    int x0 = std::min(a.x, b.x), y0 = std::min(a.y, b.y);
                    int x1 = std::max(a.x + a.w, b.x + b.w), y1 = std::max(a.y + a.h, b.y + b.h);
                    if ((x1 - x0) * (y1 - y0) - a.w * a.h - b.w * b.h < EXTRA) {
                        r[i] = {(int16_t)x0, (int16_t)y0, (int16_t)(x1 - x0), (int16_t)(y1 - y0)};
                        r[j] = r.back();
                        r.pop_back();
                        merged = true;
                        continue;
                    }
                }
                j++;
            }
        }
    }
}

// Whether a drag's line can be moved over the plot kept for it, rather than
// the plot drawn again: the same plot, nothing put up over it.
bool dragKept(bool zoom, float rangeNm, double lat, double lon) {
    return g_dragCache.valid && g_dragCache.zoom == zoom && g_dragCache.rangeNm == rangeNm &&
           g_dragCache.lat == lat && g_dragCache.lon == lon && !overlayUp();
}

void restoreUnderDrag();

// Moves the line over the kept plot, and pushes what that changed.
void moveDragLine(float rangeNm) {
    restoreUnderDrag();
    drawDragLine(rangeNm);
    for (const LabelBox &b : g_dragDrawn) {
        screen::markDirty(b.x, b.y, b.w, b.h);
    }
    screen::flush();
}

// Where the line last was, put back as the plot was under it, and marked for
// the push.
void restoreUnderDrag() {
    uint16_t *canvas = (uint16_t *)screen::canvas().getBuffer();
    int stride = screen::canvas().width();
    for (const LabelBox &b : g_dragDrawn) {
        if (b.w <= 0 || b.h <= 0) {
            continue;
        }
        for (int y = b.y; y < b.y + b.h; y++) {
            memcpy(canvas + (size_t)y * stride + b.x, g_dragCache.pixels + (size_t)(y - PLOT_T) * PLOT_W + (b.x - PLOT_L),
                   b.w * 2);
        }
        screen::markDirty(b.x, b.y, b.w, b.h);
    }
}

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
        if (c.kind == Choice::AIRPORT) {
            drawAirfield(iconX, midY, colorText);
        } else if (c.kind == Choice::NAVAID) {
            drawNavaidSymbol(iconX, midY, c.navaid->type, colorText);
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
    g_choiceH = 2 * CHOICE_PAD + CHOICE_TITLE_H + (g_choiceCount + 1) * (CHOICE_ROW_H + CHOICE_GAP) - CHOICE_GAP;
    placeBeside(x, y, g_choiceH, g_choiceX, g_choiceY);
    g_footings.kept = false;  // as a card's: see openCard()
    drawChooser();
    screen::flush();
}

// Everything within reach of a tap at (x, y) on the last plot drawn, into
// g_choices: the airports first, since an airport is what a crowd of its own
// traffic hides, then the navaids, then the contacts, each nearest first.
// The zoom has no airports to tap, but has its navaids. Returns how many.
int collectChoices(int x, int y, bool withAirports) {
    std::vector<std::pair<long, int>> airports, navaids, blips;
    for (int i = 0; i < g_navaidHitCount; i++) {
        const NavaidHit &h = g_navaidHits[i];
        long dx = x - h.x, dy = y - h.y;
        long d = dx * dx + dy * dy;
        const LabelBox &b = h.label;
        bool onLabel = b.w && x >= b.x - AIRPORT_LABEL_PAD && x < b.x + b.w + AIRPORT_LABEL_PAD &&
                       y >= b.y - AIRPORT_LABEL_PAD && y < b.y + b.h + AIRPORT_LABEL_PAD;
        if (onLabel || d <= (long)AIRPORT_TAP_R * AIRPORT_TAP_R) {
            navaids.push_back({d, i});
        }
    }
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
    std::sort(navaids.begin(), navaids.end());
    std::sort(blips.begin(), blips.end());
    int n = 0;
    for (const auto &a : airports) {
        if (n < MAX_CHOICES) {
            const AirportHit &h = g_airportHits[a.second];
            g_choices[n++] = {Choice::AIRPORT, String(h.code), String("Zoom in on ") + h.code, false, nullptr};
        }
    }
    for (const auto &v : navaids) {
        if (n < MAX_CHOICES) {
            const NavaidHit &h = g_navaidHits[v.second];
            g_choices[n++] = {Choice::NAVAID, String(h.navaid->ident),
                              String(h.navaid->ident) + " " + navaidTypeName(h.navaid->type), false, h.navaid};
        }
    }
    for (const auto &b : blips) {
        if (n < MAX_CHOICES) {
            const Blip &bl = g_blips[b.second];
            g_choices[n++] = {Choice::AIRCRAFT, bl.hex, blipLabel(bl), bl.ground, nullptr};
        }
    }
    return n;
}

enum class Target { NONE, AIRCRAFT, AIRPORT, DISMISSED };

// What a tap on the plot is for. With a list up, it picks from the list -
// or, anywhere else, dismisses it. Otherwise it is the one target within
// reach, or NONE: either nothing is, or several are and the list has gone up.
Target tapTarget(int x, int y, bool withAirports, String &outId) {
    if (g_card || g_airportCard) {
        g_card = nullptr;
        g_airportCard = nullptr;
        soundStopReading();  // its card closed
        return Target::DISMISSED;
    }
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
                if (c.kind == Choice::NAVAID) {
                    for (int k = 0; k < g_navaidHitCount; k++) {
                        if (g_navaidHits[k].navaid == c.navaid) {
                            openCard(c.navaid, g_navaidHits[k].x, g_navaidHits[k].y);
                            g_cardSurvivesRedraw = true;  // the list's redraw puts it up
                        }
                    }
                    return Target::DISMISSED;
                }
                outId = c.id;
                return c.kind == Choice::AIRPORT ? Target::AIRPORT : Target::AIRCRAFT;
            }
        }
        return Target::NONE;  // the list's title or frame
    }
    int n = collectChoices(x, y, withAirports);
    if (n == 1 && g_choices[0].kind == Choice::NAVAID) {
        const NavaidHit *h = nullptr;
        for (int k = 0; k < g_navaidHitCount; k++) {
            if (g_navaidHits[k].navaid == g_choices[0].navaid) {
                h = &g_navaidHits[k];
            }
        }
        if (h) {
            openCard(h->navaid, h->x, h->y);
            drawCard();
            screen::flush();
        }
        return Target::NONE;
    }
    if (n == 1) {
        outId = g_choices[0].id;
        return g_choices[0].kind == Choice::AIRPORT ? Target::AIRPORT : Target::AIRCRAFT;
    }
    if (n > 1) {
        g_choiceCount = n;
        openChooser(x, y);
    }
    return Target::NONE;
}

}  // namespace

void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool full) {
    ensureColors();
    if (full) {
        g_choiceCount = 0;
        if (!g_cardSurvivesRedraw) {
            g_card = nullptr;
        }
        g_cardSurvivesRedraw = false;
        g_airportCard = nullptr;
        screen::clear(colorBg);
        drawHeader();
        drawClock();
    }
    // With a list up, a refresh is drawn under it and pushed along with it -
    // as it is with a drag's line over it.
    bool following = (g_follow == FollowButton::ON);
    bool dragged = g_drag.showing && !g_drag.zoom;
    // Where the plot is centred left to itself, as radarPlotDraw() has it:
    // home, or the airport nearest it. Dragged off that, it is centred where
    // it was let go - unless its own centre has since changed under it.
    Airport airport;
    bool onAirport = !following && g_centre == RadarCentre::AIRPORT && nearestAirport(lat, lon, airport, 120.0f, false);
    double ownLat = onAirport ? airport.lat : lat, ownLon = onAirport ? airport.lon : lon;
    FreeLook &look = g_radarLook;
    if (look.active && (following || look.fromLat != lat || look.fromLon != lon)) {
        look.active = false;
    }
    g_radarView = {look.active ? look.lat : ownLat, look.active ? look.lon : ownLon, (float)RADIUS / rangeNm, ownLat,
                   ownLon, lat, lon};
    // A line dragged past the edge has the plot at a range that takes it,
    // with its map and airports and none of its traffic. Out on the map's own
    // ranges, the airports are left off as well - at 500nm there are
    // hundreds - unless ALL asks for every one.
    int shownNm = dragged ? (int)dragRangeNm((float)rangeNm) : rangeNm;
    bool mapOnly = shownNm != rangeNm || g_mapOnly;
    static const std::vector<Aircraft> noAircraft;
    static const std::vector<uint8_t> noneNew;
    RadarScene scene = {mapOnly ? noAircraft : aircraft, mapOnly ? noneNew : isNew, nullptr,
                        look.active ? look.lat : lat, look.active ? look.lon : lon, shownNm,
                        following || look.active ? RadarCentre::HOME : g_centre, g_airportsOn, true,
                        !overlayUp() && !dragged, following ? g_followHex : String(),
                        following ? g_followOrigin : String()};
    scene.freeLook = look.active;
    scene.everyAirport = (look.active || dragged) && (!g_mapOnly || g_airportsOn);
    scene.markHome = look.active && !onAirport;
    scene.homeLat = lat;
    scene.homeLon = lon;
    scene.mapOnly = mapOnly;
    scene.partial = !following && !dragged && !overlayUp();
    scene.autoRange = following && g_autoRange;
    if (!dragged || full) {
        g_dragCache.valid = false;
    }
    if (dragged && dragKept(false, (float)shownNm, scene.lat, scene.lon)) {
        moveDragLine((float)shownNm);
        return;
    }
    // With a card or a list up, the plot is drawn whole under it and pushed
    // with it, in pieces - the column, the footer, the readout in the left
    // margin and the card's part outside the column, each on its own: run
    // together they were the whole screen, 42ms of the PPA's for 28 or so.
    // All of it drawn before any is pushed, nothing shows half drawn.
    if (!full && overlayUp()) {
        screen::keepApart();
    }
    g_livePlot = true;  // the filter's funnel, over the plot's corner - a replay has none
    radarPlotDraw(scene, full);
    g_livePlot = false;
    if (overlayUp() || dragged) {
        g_footings.kept = false;  // drawn over, it may be, outside the column
    }
    if (following) {
        radarTelemetryDraw(LIVE_TEL_X, LIVE_TEL_Y, g_heights, millis(), INT32_MIN, g_lostS);
    }
    if (dragged) {
        if (!overlayUp()) {
            keepPlotForDrag(false, (float)shownNm, scene.lat, scene.lon);
        }
        drawDragLine((float)shownNm);
    }
    if (g_choiceCount) {
        drawChooser();
    }
    if (g_card) {
        drawCard();
    }
    if (g_airportCard) {
        drawAirportCard();
    }
    // Following, the telemetry goes in a push of its own: with the plot's,
    // flush() would send the margin between them too.
    if (full || overlayUp() || following || dragged) {
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

void radarScreenSetZoom(bool canIn, bool canOut, bool onScreen) {
    if (canIn == g_canZoomIn && canOut == g_canZoomOut) {
        return;
    }
    g_canZoomIn = canIn;
    g_canZoomOut = canOut;
    if (onScreen) {
        ensureColors();
        drawZoomButtons();
        screen::flush();
    }
}

void radarScreenSetControls(RadarCentre centre, bool airports, FollowButton follow, const String &followHex,
                            const String &followCallsign, const String &followOrigin, bool onScreen) {
    g_followOrigin = followOrigin;
    if (follow == FollowButton::ON) {
        g_radarLook.active = false;  // the aircraft is the centre now
    }
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
        drawTitle();
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
               String((uint32_t)(g_pushUs / g_plotFrames / 1000)) + "ms avg, map " +
               String((uint32_t)(g_coastUs / g_plotFrames)) + "us";
    g_backgroundUs = g_contactsUs = g_pushUs = g_coastUs = 0;
    g_plotFrames = 0;
    return s;
}

String radarTakeLayers() {
    String s;
    for (int i = 0; i < LAYER_COUNT; i++) {
        if (g_layerFrames[i] != 0) {
            s += String(s.length() ? ", " : "") + LAYER_LABELS[i] + " " +
                 String((uint32_t)(g_layerCycles[i] / g_layerFrames[i] / 1000)) + " (" + String(g_layerFrames[i]) +
                 "x)";
        }
        g_layerCycles[i] = 0;
        g_layerFrames[i] = 0;
    }
    return s.length() ? "kcycles a frame: " + s : String("-");
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

// Where a label `lw` by `lh` goes in `slot` - below and right, below and
// left, above and right, above and left - beside a contact at (px, py), and
// outside its ring if it is the one followed.
void labelSlot(int slot, int px, int py, int lw, int lh, bool followed, int &x, int &y) {
    int off = followed ? 21 : 9, below = followed ? 12 : 7, above = followed ? 12 + lh : 7 + lh;
    x = slot % 2 == 0 ? px + off : px - off - lw;
    y = slot < 2 ? py + below : py - above;
}

bool offPlotSide(int x, int lw) { return x < CENTER_X - RADIUS || x + lw > CENTER_X + RADIUS; }

// The followed aircraft's label, in one place beside its ring for as long as
// it can be, rather than wherever is clear at each frame - which, with the
// map moving under it and auto zoom changing the range, had it hop from side
// to side. On the radar, in the middle of the plot, its place is kept for it
// before anything else is placed - see reserveFollowLabel() - so it never has
// to move; in the zoom, where it moves over the runways, the side it had is
// tried first, and kept when nowhere is clear.
struct FollowLabel {
    String hex;
    int slot = 0;
    bool reserved = false;  // for this plot, at `box`
    LabelBox box = {0, 0, 0, 0};
};
FollowLabel g_followLabel;

// Keeps the followed aircraft's label its place on the radar, where it was
// last - or below and right of its ring, the first time - among the map's
// labels, which the airports', navaids' and other contacts' labels and the
// names all keep clear of. Projected as drawContacts() projects it.
void reserveFollowLabel(const std::vector<Aircraft> &aircraft, double lat, double lon, float pxPerNm,
                        const String &hex) {
    g_followLabel.reserved = false;
    double nmPerDegLon = 60.0 * cos(lat * M_PI / 180.0);
    for (const Aircraft &ac : aircraft) {
        if (ac.hex != hex || !ac.hasPos) {
            continue;
        }
        double dNorth = (ac.lat - lat) * 60.0;
        double dEast = (ac.lon - lon) * nmPerDegLon;
        float px = CENTER_X + (float)dEast * pxPerNm;
        float py = CENTER_Y - (float)dNorth * pxPerNm;
        int lw = Raster::textWidth(ac.callsign), lh = LABEL_H;
        int slot = g_followLabel.hex == hex ? g_followLabel.slot : 0, x, y;
        labelSlot(slot, (int)px, (int)py, lw, lh, true, x, y);
        if (offPlotSide(x, lw)) {
            slot ^= 1;  // the other side, on the plot
            labelSlot(slot, (int)px, (int)py, lw, lh, true, x, y);
        }
        LabelBox box = {(int16_t)x, (int16_t)y, (int16_t)lw, (int16_t)lh};
        g_followLabel = {hex, slot, true, box};
        if (g_mapLabelCount < MAX_MAP_LABELS) {
            g_mapLabels[g_mapLabelCount++] = box;
        }
        return;
    }
}

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
    g_contactLabels.clear();
    g_trafficBoxes.clear();
    g_blipCount = 0;
    g_fastestKt = 0;
    int32_t clipX, clipY, clipW, clipH;
    canvas.getClipRect(&clipX, &clipY, &clipW, &clipH);
    Raster out = {(uint16_t *)canvas.getBuffer(), canvas.width(), (int)clipX, (int)clipY,
                  (int)(clipX + clipW - 1), (int)(clipY + clipH - 1), 0, 0};

    // Trails first, so every blip sits on top of its own and anyone else's.
    if (trails != nullptr) {
        bool penDown = false;
        int lastX = 0, lastY = 0;
        out.use(colorTrail);
        for (const replay::TrailPoint &p : *trails) {
            if (isnan(p.lat)) {
                penDown = false;
                continue;
            }
            int x = CENTER_X + (int)lroundf((float)((p.lon - v.lon) * nmPerDegLon) * v.pxPerNm);
            int y = CENTER_Y - (int)lroundf((float)((p.lat - v.lat) * 60.0) * v.pxPerNm);
            if (penDown) {
                out.line(lastX, lastY, x, y);
                noteLine(lastX, lastY, x, y);
            } else {
                out.disc(x, y, 1);
                g_trafficBoxes.push_back({(int16_t)(x - 2), (int16_t)(y - 2), 5, 5});
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
        if (!ac.posStale) {
            g_fastestKt = std::max(g_fastestKt, ac.speedStr.toFloat());
        }

        bool isNewHere = (i < isNew.size()) && isNew[i];
        // A position the feed has lost, kept from the last it had, dimmed to
        // the trails' shade: where it was, as near as anyone can say.
        uint16_t color = ac.posStale                  ? colorTrail
                         : (ac.alert & ALERT_EMERGENCY) ? colorEmergency
                         : ac.alert                     ? colorAlert
                         : isNewHere                    ? colorNewBlip
                                                        : colorBlip;

        // What it covers - blip, chevrons and ring if followed, and apart
        // from them its vector - for the next frame to put back from the
        // background.
        int gx0 = (int)px - 6, gy0 = (int)py - 16, gx1 = (int)px + 7, gy1 = (int)py + 17;
        if (ac.hasTrack) {
            // How far it will fly in the lead time at its current groundspeed,
            // with a floor so a slow or speed-less contact still shows which
            // way it's pointing.
            float gs = ac.speedStr.toFloat();
            float len = gs * v.leadS / 3600.0f * v.pxPerNm;
            if (len < 14.0f) {
                len = 14.0f;
            }
            // And a ceiling, so one passing over at speed at a close range -
            // a jet across a zoom at half a mile - says which way it is going
            // without a line across the whole plot. Nothing at the radar's
            // usual ranges comes near it.
            len = std::min(len, RADIUS / 2.0f);
            float a = ac.track * (float)M_PI / 180.0f;
            out.use(color);
            int vx = (int)(px + sinf(a) * len), vy = (int)(py - cosf(a) * len);
            out.line((int)px, (int)py, vx, vy);
            noteLine((int)px, (int)py, vx, vy);
        }
        bool onGround = (ac.status == "GROUND" || ac.status == "TAXI");
        if (v.zoomed && onGround) {
            out.use(colorBg);
            out.disc((int)px, (int)py, 5);  // hollow, over its own vector
            out.use(color);
            out.ring((int)px, (int)py, 5);
            out.ring((int)px, (int)py, 4);
        } else {
            out.use(color);
            out.disc((int)px, (int)py, 5);
        }
        drawTrend(out, (int)px, (int)py, ac.status);
        bool followed = v.followHex.length() && ac.hex == v.followHex;
        if (followed) {
            // Clear of the trend chevrons, which reach 15px above and below.
            out.use(colorHome);
            out.ring((int)px, (int)py, 17);
            out.ring((int)px, (int)py, 18);
            gx0 = std::min(gx0, (int)px - 19);
            gy0 = std::min(gy0, (int)py - 19);
            gx1 = std::max(gx1, (int)px + 20);
            gy1 = std::max(gy1, (int)py + 20);
        }
        g_trafficBoxes.push_back({(int16_t)gx0, (int16_t)gy0, (int16_t)(gx1 - gx0), (int16_t)(gy1 - gy0)});
        if (g_blipCount < MAX_BLIPS) {
            Blip &b = g_blips[g_blipCount++];
            b.x = (int16_t)px;
            b.y = (int16_t)py;
            b.hex = ac.hex;
            b.callsign = ac.callsign;
            b.type = ac.type;
            b.altStr = ac.altStr;
            b.ground = onGround;
        }

        // The label flashes between red and an ordinary contact's colour. A
        // flashing one is drawn even where it would collide with another
        // label: it's the one being drawn attention to - as is the one being
        // followed.
        AlertFlash flash = flashing ? alertFlash(ac.hex) : AlertFlash::NONE;
        uint16_t labelColor = flash == AlertFlash::RED ? colorEmergency
                              : flash == AlertFlash::OFF ? (isNewHere ? colorNewBlip : colorBlip)
                              : ac.posStale              ? colorFaint
                              : color == colorBlip && isCargo(ac.callsign) ? colorCargo
                                                         : color;

        String sub;
        if (v.zoomed && ac.altStr != "?") {
            sub = onGround ? String("GND") : ac.altStr + "ft";
        }
        bool forced = followed || flash != AlertFlash::NONE;
        // In the zoom, one stopped on the ground - parked at a stand, as a
        // busy airport's dozens are - is its ring alone: their callsigns and
        // GNDs were a heap over Heathrow's runways, and what is moving is
        // what there is to read.
        if (v.zoomed && ac.status == "GROUND" && !forced) {
            continue;
        }
        int lw = std::max(Raster::textWidth(ac.callsign), sub.length() ? Raster::textWidth(sub) : 0);
        int lh = sub.length() ? 2 * LABEL_H + 2 : LABEL_H;
        // Below and to the right of it, or failing that the left, or above
        // either side - clear of the other labels, and of the map's: the
        // airports' and navaids' labels and symbols, the ring distances and
        // compass points, which a callsign used to be written over. The
        // followed one's goes outside its ring, and stays put - see
        // FollowLabel: kept its place on the radar, and in the zoom tried
        // first where it was.
        bool sticky = followed && g_followLabel.hex == ac.hex;
        int kept = sticky ? g_followLabel.slot : 0;
        int lx, ly;
        bool clear = false;
        if (sticky && g_followLabel.reserved) {
            lx = g_followLabel.box.x;
            ly = g_followLabel.box.y;
            clear = true;
        } else {
            // Where it goes if forced, with nowhere clear: where it was, if it
            // is the one followed, or else below - on the side on the plot.
            int chosen = kept;
            labelSlot(chosen, (int)px, (int)py, lw, lh, followed, lx, ly);
            if (offPlotSide(lx, lw)) {
                chosen ^= 1;
                labelSlot(chosen, (int)px, (int)py, lw, lh, followed, lx, ly);
            }
            for (int k = -1; k < 4 && !clear; k++) {
                int slot = k < 0 ? kept : k, x, y;
                if (k >= 0 && slot == kept) {
                    continue;
                }
                labelSlot(slot, (int)px, (int)py, lw, lh, followed, x, y);
                if (offPlotSide(x, lw)) {
                    continue;
                }
                if (!overlapsAny(taken, takenCount, x, y, lw, lh) &&
                    !overlapsAny(g_mapLabels, g_mapLabelCount, x, y, lw, lh, 2) &&
                    !overlapsAny(g_mapMarks.data(), (int)g_mapMarks.size(), x, y, lw, lh, 2)) {
                    lx = x;
                    ly = y;
                    clear = true;
                    chosen = slot;
                }
            }
            if (followed) {
                g_followLabel.hex = ac.hex;
                g_followLabel.slot = chosen;
            }
        }
        clear = clear || forced;
        if (clear && (takenCount < MAX_LABELS || forced)) {
            out.use(labelColor);
            out.text(ac.callsign, lx, ly);
            if (sub.length()) {
                // Dimmer than the callsign, unless the contact is alerted.
                out.use(color == colorBlip || color == colorNewBlip ? colorFaint : color);
                out.text(sub, lx, ly + LABEL_H + 2);
            }
            if (takenCount < MAX_LABELS) {
                taken[takenCount++] = {(int16_t)lx, (int16_t)ly, (int16_t)lw, (int16_t)lh};
            }
            g_contactLabels.push_back({(int16_t)lx, (int16_t)ly, (int16_t)lw, (int16_t)lh});
            g_trafficBoxes.push_back({(int16_t)lx, (int16_t)ly, (int16_t)(lw + 1), (int16_t)(lh + 1)});
            labelled++;
        }
    }
}

// Pushes a plot redrawn on a refresh: the plot's column, marked by its fill,
// the footer readouts to the bottom right, and the readouts from `leftTop`
// down in the bottom left - the range, and over it the zoom's runways in use.
// Without `footer` or `left`, those are as they were - only their parts in
// the column drawn again, which go with it.
void pushPlot(bool push, int leftTop, bool footer = true, bool left = true) {
    int leftH = RANGE_BOX_T + BOX_H - leftTop;
    if (!push) {
        // Marked rather than pushed: the plot was by its fill, the footer
        // wasn't, and whoever pushes next has to take both.
        screen::markDirty(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T);
        screen::markDirty(0, leftTop, PLOT_L, leftH);
        return;
    }
    // Two flushes rather than one: marked together, flush() would judge the
    // two regions close enough to send as their bounding box, which takes in
    // the empty margin between the plot and the right edge. The range
    // readout's end in the left margin goes with the footer, a strip too thin
    // beside it to be merged into a box with it.
    screen::flush();
    if (footer) {
        screen::markDirty(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T);
    }
    if (left) {
        screen::markDirty(0, leftTop, PLOT_L, leftH);
    }
    screen::flush();
}

}  // namespace

// The last frame's traffic taken off a plot whose background is kept - see
// Background - by putting back what was under it: the first half of a
// refresh of the traffic alone. With the radar's names, which the background
// is kept without, where the last full draw put them.
void restoreTraffic(bool names) {
    g_trafficWas.swap(g_trafficBoxes);
    for (const LabelBox &b : g_trafficWas) {
        restoreBackground(b);
    }
    if (names) {
        for (const LabelBox &b : g_trafficWas) {
            drawNameLetters(std::max<int>(b.x, PLOT_L), std::max<int>(b.y, PLOT_T), std::min<int>(b.x + b.w, PLOT_R),
                            std::min<int>(b.y + b.h, PLOT_B));
        }
    }
}

// And the second, once the traffic is drawn where it is now: the places it
// was and is pushed - or, if a crowd makes that the slower, the plot whole -
// and the footer with them if `footer` says it has changed. The left
// margin's readouts are the same for as long as the view is. Twenty aircraft
// are a dozen boxes, a fraction of the 26.7ms the whole plot takes to push.
void pushTraffic(bool push, bool footer) {
    // Where the traffic was and is, clipped to the plot and run together.
    static std::vector<screen::Region> boxes;
    boxes.clear();
    for (const std::vector<LabelBox> *list : {&g_trafficWas, &g_trafficBoxes}) {
        for (const LabelBox &b : *list) {
            int x0 = std::max<int>(b.x, PLOT_L), x1 = std::min<int>(b.x + b.w, PLOT_R);
            int y0 = std::max<int>(b.y, PLOT_T), y1 = std::min<int>(b.y + b.h, PLOT_B);
            if (x1 > x0 && y1 > y0) {
                boxes.push_back({(int16_t)x0, (int16_t)y0, (int16_t)(x1 - x0), (int16_t)(y1 - y0)});
            }
        }
    }
    mergeBoxes(boxes);
    // A push is 0.085ms and 53ns a pixel; the whole plot is 26.7ms.
    float cost = 0;
    for (const screen::Region &b : boxes) {
        cost += 0.085f + b.w * b.h * 0.000053f;
    }
    bool whole = cost > 24.0f;
    if (!push) {
        for (const screen::Region &b : boxes) {
            screen::markDirty(b.x, b.y, b.w, b.h);
        }
    } else if (whole) {
        screen::markDirty(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);
        screen::flush();
    } else {
        screen::pushRects(boxes.data(), (int)boxes.size());
    }
    if (footer) {
        screen::markDirty(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T);
    }
    if (push) {
        screen::flush();
    }
}

// Whether the traffic drawn this frame, or the last frame's put back from the
// background, meets box `b`: both inside the plot's column, which is all of
// either there is.
bool trafficMeets(const LabelBox &b) {
    int x0 = std::max<int>(b.x, PLOT_L), x1 = std::min<int>(b.x + b.w, PLOT_R);
    int y0 = std::max<int>(b.y, PLOT_T), y1 = std::min<int>(b.y + b.h, PLOT_B);
    if (x1 <= x0 || y1 <= y0) {
        return false;
    }
    return overlapsAny(g_trafficWas.data(), (int)g_trafficWas.size(), x0, y0, x1 - x0, y1 - y0, 0) ||
           overlapsAny(g_trafficBoxes.data(), (int)g_trafficBoxes.size(), x0, y0, x1 - x0, y1 - y0, 0);
}

// The centre's airport code drawn over the traffic - a contact on the ground
// there put its dot over the letters, the L of GLA - with a rim of the plot's
// own dark round them, so it reads over a dot as it does over the plot. Into
// the background too, as it is kept, so that a refresh of the traffic alone
// draws it again only where the traffic meets it - see centreCodeMet().
//
// What the canvas drew as nine passes of its text - the code at the eight
// steps round it in the dark, then in place in home's colour - in one pass
// through the raster: each of the code's scan lines as bits, a pixel a bit,
// grown by a pixel each way for the rim. A code too long for 64 bits, which
// no airport's is, goes round the nine passes, through the raster still.
void drawCentreCode(const String &code) {
    int n = code.length();
    if (n == 0) {
        return;
    }
    auto &canvas = screen::canvas();
    int32_t clipX, clipY, clipW, clipH;
    canvas.getClipRect(&clipX, &clipY, &clipW, &clipH);
    Raster out = {(uint16_t *)canvas.getBuffer(), canvas.width(), (int)clipX, (int)clipY,
                  (int)(clipX + clipW - 1), (int)(clipY + clipH - 1), 0, 0};
    int left = CENTER_X - (Raster::textWidth(code) >> 1);  // as the canvas centres it
    if (n > 5) {
        out.use(colorBg);
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx != 0 || dy != 0) {
                    out.text(code, left + dx, CENTRE_CODE_Y + dy);
                }
            }
        }
        out.use(colorHome);
        out.text(code, left, CENTRE_CODE_Y);
        return;
    }
    // ink[i] the scan line at CENTRE_CODE_Y - 1 + i, its bit k the pixel at
    // left - 1 + k: Font0 at size 2, as Raster::text() draws it, with a line
    // and a column to spare all round for the rim.
    uint64_t ink[18] = {};
    for (int k = 0; k < n; k++) {
        const uint8_t *glyph = glcd::font + (uint8_t)code[k] * 5;
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 8; row++) {
                if (glyph[col] >> row & 1) {
                    uint64_t bits = 3ull << (1 + k * 12 + col * 2);
                    ink[1 + row * 2] |= bits;
                    ink[2 + row * 2] |= bits;
                }
            }
        }
    }
    auto runs = [&](uint64_t bits, int y) {
        while (bits) {
            int from = __builtin_ctzll(bits);
            int len = __builtin_ctzll(~(bits >> from));
            out.hline(left - 1 + from, y, len);
            bits &= ~(((1ull << len) - 1) << from);
        }
    };
    for (int i = 0; i < 18; i++) {
        uint64_t near = ink[i] | (i > 0 ? ink[i - 1] : 0) | (i < 17 ? ink[i + 1] : 0);
        int y = CENTRE_CODE_Y - 1 + i;
        out.use(colorBg);
        runs((near | near << 1 | near >> 1) & ~ink[i], y);
        out.use(colorHome);
        runs(ink[i], y);
    }
}

// Whether the traffic drawn this frame, or the last frame's put back from the
// background, meets the centre's code - its letters and their rim, with a
// pixel to spare. Elsewhere the code is still there as the background has it,
// rim and all, and was nine passes of the canvas's text and a box put back
// and pushed every frame.
bool centreCodeMet(const String &code) {
    if (code.length() == 0) {
        return false;
    }
    int w = Raster::textWidth(code);
    return trafficMeets({(int16_t)(CENTER_X - w / 2 - 2), (int16_t)(CENTRE_CODE_Y - 2), (int16_t)(w + 4),
                         (int16_t)(LABEL_H + 4)});
}

// A refresh of the radar's traffic alone - see Background.
void refreshTraffic(const RadarScene &scene, double lat, double lon, int rangeNm, int step, uint32_t t0,
                    const String &centreCode) {
    auto &canvas = screen::canvas();
    restoreTraffic(true);
    canvas.setFont(&fonts::Font0);
    canvas.setClipRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);
    int plotted = 0, labelled = 0;
    uint32_t t1 = micros();
    drawContacts(scene.aircraft, scene.isNew, scene.trails, scene.flash,
                 {lat, lon, (float)RADIUS / rangeNm, 60.0f, false, scene.followHex}, plotted, labelled);
    if (centreCodeMet(centreCode)) {
        drawCentreCode(centreCode);
    }
    canvas.clearClipRect();
    // The footer's readouts, the same for as long as the view is but the
    // count: each drawn again over itself only where the traffic put back or
    // drawn meets it - the range readout reaches into the plot's column, and
    // a long count does - and the count and the legend over it erased and
    // drawn afresh, and pushed, when the count changes. Elsewhere they are
    // left as they are: drawn over themselves, some 70 letters a frame, they
    // changed nothing.
    String count = contactsText(plotted, labelled);
    bool recount = !scene.mapOnly && count != g_countShown;
    if (recount) {
        canvas.fillRect(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T, colorBg);
        g_footings.count = count;
    }
    String range = rangeText(rangeNm, step, scene.autoRange);
    if (trafficMeets(footerBoxAt(range, 16, FOOTER_Y, false))) {
        footerBox(range, 16, FOOTER_Y, false);
    }
    if (!scene.mapOnly) {
        static const String legend = "^ climb   v descent";
        if (recount || trafficMeets(footerBoxAt(legend, 1264, FOOTER_Y - LEGEND_OFFSET, true))) {
            footerBox(legend, 1264, FOOTER_Y - LEGEND_OFFSET, true);
        }
        if (recount || trafficMeets(footerBoxAt(count, 1264, FOOTER_Y, true))) {
            drawCount(count);
        }
    }
    uint32_t t2 = micros();
    g_backgroundUs += t1 - t0;
    g_contactsUs += t2 - t1;
    g_plotFrames++;
    pushTraffic(scene.push, recount);
    g_pushUs += micros() - t2;
}

void radarPlotDraw(const RadarScene &scene, bool full) {
    ensureColors();
    uint32_t t0 = micros();
    auto &canvas = screen::canvas();
    double lat = scene.lat, lon = scene.lon;
    int rangeNm = scene.rangeNm;
    RadarCentre centre = scene.centre;

    // Centred on the airport, the centre is its symbol and its code, so it
    // says where it is; centred on home, it is a bare cross. Airport mode with
    // nothing in range of the lookup falls back to home, since there is no
    // other point to centre on.
    Airport airport;
    String centreCode;
    bool following = scene.followHex.length() > 0;
    if (!following && !scene.freeLook && centre == RadarCentre::AIRPORT &&
        nearestAirport(lat, lon, airport, 120.0f, false)) {
        lat = airport.lat;
        lon = airport.lon;
        centreCode = airport.code;
    }
    int step = g_ringsOn ? ringStep(rangeNm) : 0;
    bool drawsAirports = (scene.airports && centreCode.length()) || following || scene.everyAirport;

    // What the plot is of, all but its traffic: if it is what the background
    // kept was, and only the traffic has moved, the refresh is of that alone.
    BackgroundKey key;
    memset(&key, 0, sizeof(key));  // padding and all, for the compare
    key.lat = lat;
    key.lon = lon;
    key.homeLat = scene.homeLat;
    key.homeLon = scene.homeLon;
    key.rangeNm = rangeNm;
    key.mapLevels = coastLevels();
    key.coast = g_coastOn;
    key.rings = step != 0;
    key.navaids = g_navaidsOn;
    key.airports = drawsAirports;
    key.everyAirport = scene.everyAirport;
    key.freeLook = scene.freeLook;
    key.markHome = scene.markHome;
    key.mapOnly = scene.mapOnly;
    strncpy(key.centreCode, centreCode.c_str(), sizeof(key.centreCode) - 1);
    // Kept for this view - which with a list up, say, it stays, though the
    // plot is drawn whole under it.
    bool canKeep = scene.partial && !following;
    bool keptFor = g_background.valid && sameBackground(key, g_background.key);
    if (!full && canKeep && keptFor && millis() - g_background.wholeMs < NAMES_MAX_MS && trafficCheap()) {
        refreshTraffic(scene, lat, lon, rangeNm, step, t0, centreCode);
        return;
    }

    uint32_t layerAt = cycleCount();
    if (!full) {
        screen::fillRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T, colorBg);
        layerAt = layerDone(LAYER_FILL, layerAt);
    }
    canvas.setFont(&fonts::Font0);

    // --- the map, then rings and bearings over it --------------------------
    drawCoast(lat, lon, (float)rangeNm);
    layerAt = layerDone(LAYER_COAST, layerAt);
    drawScope((float)rangeNm, (float)step);
    layerAt = layerDone(LAYER_SCOPE, layerAt);

    g_airportHitCount = 0;
    g_navaidHitCount = 0;
    g_mapLabelCount = 0;
    g_airportAtCentre = false;
    if (centreCode.length()) {
        canvas.setTextSize(2);
        canvas.setTextColor(colorHome);
        canvas.setTextDatum(TC_DATUM);
        canvas.drawString(airport.code, CENTER_X, CENTRE_CODE_Y);
        int w = canvas.textWidth(airport.code);
        LabelBox label = {(int16_t)(CENTER_X - w / 2), (int16_t)CENTRE_CODE_Y, (int16_t)w, (int16_t)LABEL_H};
        noteAirportHit(airport.code, CENTER_X, CENTER_Y, label);
        g_mapLabels[g_mapLabelCount++] = label;
        g_airfieldSpots.push_back({(float)CENTER_X, (float)CENTER_Y});  // the cross is its symbol
        // Its symbol, as every other airport has, in home's colour - and no
        // cross: the cross is home's own, for the plot centred on home.
        drawAirfield(CENTER_X, CENTER_Y, colorHome);
    }
    // Following, the centre is the aircraft, ringed where it is drawn.
    // Dragged off its own centre, the middle is nowhere in particular: a
    // smaller, dimmer cross says where it is, and home keeps its own where
    // home is, if that is still on the plot.
    if (scene.freeLook) {
        int hx = CENTER_X + (int)lround((scene.homeLon - lon) * 60.0 * cos(lat * M_PI / 180.0) / rangeNm * RADIUS);
        int hy = CENTER_Y - (int)lround((scene.homeLat - lat) * 60.0 / rangeNm * RADIUS);
        if (scene.markHome && insideScope(hx - 9, hy - 9, 18, 18, 0)) {
            canvas.drawLine(hx - 9, hy, hx + 9, hy, colorHome);
            canvas.drawLine(hx, hy - 9, hx, hy + 9, colorHome);
            noteMark(hx - 9, hy - 9, 19, 19);
        }
    } else if (!following && !centreCode.length()) {
        canvas.drawLine(CENTER_X - 9, CENTER_Y, CENTER_X + 9, CENTER_Y, colorHome);
        canvas.drawLine(CENTER_X, CENTER_Y - 9, CENTER_X, CENTER_Y + 9, colorHome);
    }
    int centreReach = following ? 18 : 9;  // the followed one's ring, or the cross
    noteMark(CENTER_X - centreReach, CENTER_Y - centreReach, 2 * centreReach + 1, 2 * centreReach + 1);

    // --- contacts ---------------------------------------------------------
    canvas.setClipRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);

    // Part of the airport view, so centred on home they are off, as they are
    // on a replay switched to HOME. Following, they are always on, whichever
    // view it was picked from: they say where it might be going - and
    // following a departure, where it came from.
    // Following, its label's place kept for it before the map's labels go on.
    if (following) {
        reserveFollowLabel(scene.aircraft, lat, lon, (float)RADIUS / rangeNm, scene.followHex);
    } else {
        g_followLabel.reserved = false;
    }
    layerAt = layerDone(LAYER_CENTRE, layerAt);
    if (drawsAirports) {
        drawAirports(lat, lon, rangeNm, centreCode, following ? scene.originCode : String());
    }
    layerAt = layerDone(LAYER_AIRPORTS, layerAt);
    // A free look's middle, a small dim cross - but not over an airport's
    // symbol there, as one opened from its zoom is centred on: the airport
    // says where the middle is, and a cross is home's.
    if (scene.freeLook && !g_airportAtCentre) {
        canvas.drawLine(CENTER_X - 5, CENTER_Y, CENTER_X + 5, CENTER_Y, colorMuted);
        canvas.drawLine(CENTER_X, CENTER_Y - 5, CENTER_X, CENTER_Y + 5, colorMuted);
    }
    // Past 150nm their labels would bury the traffic: the symbols alone,
    // still there to tap for a card. With none drawn, none to tap.
    constexpr int NAVAID_LABELS_MAX_NM = 150;
    if (g_navaidsOn && !scene.mapOnly) {
        drawNavaids(lat, lon, rangeNm, rangeNm <= NAVAID_LABELS_MAX_NM);
    } else {
        g_navaidHitCount = 0;
    }
    layerDone(LAYER_NAVAIDS, layerAt);

    // The background kept before the names go on, as they move with the
    // traffic - see drawNames() - and with the centre's code on it as it is
    // drawn over the traffic.
    if (canKeep && g_background.pending && sameBackground(key, g_background.key)) {
        drawCentreCode(centreCode);
        keepBackground();
        g_background.pending = false;
    } else if (!keptFor) {
        g_background.valid = false;
        g_background.pending = canKeep;
        g_background.key = key;
    }  // else a crowd, a list or the names' minute drawn whole over a background still kept
    g_background.wholeMs = millis();
    layerAt = cycleCount();
    drawNames(lat, lon, (float)rangeNm);
    layerDone(LAYER_NAMES, layerAt);

    int plotted = 0, labelled = 0;
    uint32_t t1 = micros();
    drawContacts(scene.aircraft, scene.isNew, scene.trails, scene.flash,
                 {lat, lon, (float)RADIUS / rangeNm, 60.0f, false, scene.followHex}, plotted, labelled);
    drawCentreCode(centreCode);

    canvas.clearClipRect();
    // Out over the plot's corner, so wiped with it: put back each time.
    if (g_livePlot) {
        drawFilterIcon();
    }

    // --- footings ---------------------------------------------------------
    // Stacked in the bottom right rather than run along the bottom left: the
    // plot is still 165px wide at the footer's height, and a legend appended
    // to the range line reached x=664, well inside it. Boxed from the right
    // edge, every one of them starts beyond the widest part of the circle at
    // the height it sits at.
    // The range readout reaches from the margin into the plot square, and a
    // long count does too: their parts in the column, filled afresh, drawn
    // again whatever. Outside it, on a refresh of the live radar with nothing
    // over it, each is left as it was unless its text has changed - following
    // changes the range as it goes, and the count - and only then erased,
    // drawn afresh and pushed: the footer and the range's start in the
    // margin, two pushes and some 70 letters a frame, mostly the same.
    String range = rangeText(rangeNm, step, scene.autoRange);
    String count = scene.mapOnly ? String() : contactsText(plotted, labelled);
    bool kept = !full && scene.push && g_livePlot && g_footings.kept;
    bool newRange = !kept || range != g_footings.range, newCount = !kept || count != g_footings.count;
    if (!full && newCount) {
        canvas.fillRect(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T, colorBg);
    }
    if (!full && newRange) {
        canvas.fillRect(0, RANGE_BOX_T, PLOT_L, BOX_H, colorBg);
    }
    if (newRange || inColumn(footerBoxAt(range, 16, FOOTER_Y, false))) {
        footerBox(range, 16, FOOTER_Y, false);
    }
    if (!scene.mapOnly) {
        static const String legend = "^ climb   v descent";
        if (newCount || inColumn(footerBoxAt(legend, 1264, FOOTER_Y - LEGEND_OFFSET, true))) {
            footerBox(legend, 1264, FOOTER_Y - LEGEND_OFFSET, true);
        }
        if (newCount || inColumn(footerBoxAt(count, 1264, FOOTER_Y, true))) {
            drawCount(count);
        }
    }
    g_footings = {range, count, g_livePlot};

    uint32_t t2 = micros();
    g_backgroundUs += t1 - t0;
    g_contactsUs += t2 - t1;
    g_plotFrames++;
    if (full) {
        return;  // the caller flushes, once it has drawn the rest of the screen
    }
    pushPlot(scene.push, RANGE_BOX_T, newCount, newRange);
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
// The live zoom's ZOOM IN and OUT step its range through these, from the
// frame's own: in to taxiway level, where a crowded apron's contacts come
// apart, and out to the frame's range - past which ZOOM OUT goes back to the radar.
constexpr float ZOOM_VIEW_STEPS_NM[] = {0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
// Between the end of a runway and its number, and its number and its
// extended centreline.
constexpr float RUNWAY_LABEL_GAP = 5.0f;
// The arrowhead on a runway end in use: how far in from the end it starts,
// where the centreline's dashes do, and its size, in pixels.
constexpr float RUNWAY_ARROW_INSET = 10.0f, RUNWAY_ARROW_LEN = 18.0f, RUNWAY_ARROW_HALF_W = 8.0f;

// An end is in use while something in the air is lined up on it going the way
// that end faces: below circuit height, pointing along it and near its
// extended centreline - on either side, since departures go the same way as
// arrivals. Only in the air: on the ground, a vehicle racing down a runway
// and back can light both its ends - most likely what had Heathrow's 09R
// and 27L both in use, with a runway inspection car, SQUID30, on it - and a
// taxi along one the wrong way could too.
constexpr float IN_USE_ALIGN_DEG = 15.0f;
constexpr int IN_USE_MAX_AGL_FT = 2500;
constexpr float IN_USE_AIR_OFFSET_NM = 0.5f;

struct ZoomFrame {
    String code;
    double lat = 0, lon = 0;  // the middle of its runways, or the airport where it has none
    float rangeNm = ZOOM_NO_RUNWAYS_NM;   // the range shown: framedNm, or closer in by ZOOM IN
    float framedNm = ZOOM_NO_RUNWAYS_NM;  // the range it is framed at, to fit its runways
    const Runway *runways = nullptr;
    int runwayCount = 0;  // no more than MAX_ZOOM_RUNWAYS
};
ZoomFrame g_zoom;

// The control tower of the airport last zoomed on, from the card: looked up
// as its zoom is first drawn, and again each time one is opened, for a card
// put in since.
struct ZoomTower {
    char code[5] = "";
    bool has = false;
    double lat = 0, lon = 0;
};
ZoomTower g_zoomTower;

// When each runway end of one airport was last in use, so a zoom in a lull
// still shows which way the airport is working: airports seldom turn round
// more than a few times a day. On the clock its zoom is drawn on - the live
// zoom's millis(), a replay's own time - and forgotten when that runs
// backwards, as a replay going back does, or the airport changes.
//
// A replay's starts again, each time, from what the live zoom remembered as
// the recording began - its `seed`, at the replay's 0.
struct InUseMemory {
    char code[5] = "";
    uint32_t lastDrawnMs = 0;
    uint32_t ms[MAX_ZOOM_RUNWAYS][2];
    bool seen[MAX_ZOOM_RUNWAYS][2];
    recorder::RunwaysInUse seed;

    // Notes the ends in use now, at `nowMs`, and says which others were
    // within RUNWAY_IN_USE_MEMORY_MS.
    void update(const ZoomFrame &f, const bool inUse[][2], uint32_t nowMs, bool recent[][2]) {
        if (strcmp(code, f.code.c_str()) != 0 || nowMs < lastDrawnMs) {
            strlcpy(code, f.code.c_str(), sizeof(code));
            memset(seen, 0, sizeof(seen));
            if (seed.code == f.code) {
                for (const recorder::RunwaysInUse::End &e : seed.ends) {
                    for (int r = 0; r < f.runwayCount; r++) {
                        for (int k = 0; k < 2; k++) {
                            if (strcmp(f.runways[r].end[k].ident, e.ident) == 0) {
                                seen[r][k] = true;
                                ms[r][k] = 0u - e.agoS * 1000u;  // before 0, which the subtraction below wraps back
                            }
                        }
                    }
                }
            }
        }
        lastDrawnMs = nowMs;
        for (int r = 0; r < f.runwayCount; r++) {
            for (int k = 0; k < 2; k++) {
                if (inUse[r][k]) {
                    seen[r][k] = true;
                    ms[r][k] = nowMs;
                }
                recent[r][k] = !inUse[r][k] && seen[r][k] && nowMs - ms[r][k] <= RUNWAY_IN_USE_MEMORY_MS;
            }
        }
    }
};
// The live one is read by a recording starting, from the poll task too, so
// it is only touched under this. Nothing in it allocates.
portMUX_TYPE g_inUseMux = portMUX_INITIALIZER_UNLOCKED;
InUseMemory g_liveInUse, g_replayInUse;

float g_zoomViewNm = ZOOM_NO_RUNWAYS_NM;  // the range the live zoom shows: see ZOOM_VIEW_STEPS_NM
bool g_zoomOnFollowed = false;            // its last draw was centred on the aircraft followed
String g_zoomFollowHex, g_zoomFollowCallsign;
bool g_zoomPicking = false;

// A point in a zoom's flat-earth frame, in nm east and north of its middle.
struct Vec {
    float x, y;
};

// The frame's nm to a degree of longitude - in double, a cos the P4's FPU,
// single precision, hasn't got: some 4,000 instructions, so worked out once
// for a frame's many points rather than for each.
double nmPerDegLonOf(const ZoomFrame &f) { return 60.0 * cos(f.lat * M_PI / 180.0); }

Vec toLocal(const ZoomFrame &f, double nmPerDegLon, float lat, float lon) {
    return {(float)((lon - f.lon) * nmPerDegLon), (float)((lat - f.lat) * 60.0)};
}

Vec toLocal(const ZoomFrame &f, float lat, float lon) { return toLocal(f, nmPerDegLonOf(f), lat, lon); }

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
    f.framedNm = f.rangeNm;
    return true;
}

// Which way each runway end is being used, from the contacts lined up on it.
// A contact counts for one end at most - the one whose centreline it is
// nearest - so a pair of close parallels doesn't light up together for the
// one aircraft.
void runwaysInUse(const ZoomFrame &f, const std::vector<Aircraft> &aircraft, bool inUse[][2]) {
    // Each runway's ends, and from each the way along it and its heading,
    // worked out once a frame rather than for every contact - every zoom
    // frame, the partial ones too. The same sums, so the same flags, which
    // the background is kept by.
    struct Way {
        Vec from;
        float ux, uy, heading;
    };
    Way ways[MAX_ZOOM_RUNWAYS][2];
    float lens[MAX_ZOOM_RUNWAYS];
    double nmPerDegLon = nmPerDegLonOf(f);
    for (int r = 0; r < f.runwayCount; r++) {
        inUse[r][0] = inUse[r][1] = false;
        const Runway &rw = f.runways[r];
        Vec ends[2] = {toLocal(f, nmPerDegLon, rw.end[0].lat, rw.end[0].lon),
                       toLocal(f, nmPerDegLon, rw.end[1].lat, rw.end[1].lon)};
        float len = hypotf(ends[1].x - ends[0].x, ends[1].y - ends[0].y);
        lens[r] = len;
        if (len <= 0) {
            continue;
        }
        for (int k = 0; k < 2; k++) {
            // From this end towards the other: the way an aircraft using it goes.
            const Vec &from = ends[k], &to = ends[1 - k];
            Way &w = ways[r][k];
            w.from = from;
            w.ux = (to.x - from.x) / len;
            w.uy = (to.y - from.y) / len;
            w.heading = atan2f(w.ux, w.uy) * 180.0f / (float)M_PI;
        }
    }
    for (const Aircraft &ac : aircraft) {
        if (!ac.hasPos || !ac.hasTrack || ac.altStr == "?" || ac.altStr == "GND" || ac.status == "GROUND" ||
            ac.status == "TAXI") {
            continue;
        }
        Vec p = toLocal(f, nmPerDegLon, ac.lat, ac.lon);
        int altFt = ac.altStr.toInt();
        int bestR = -1, bestK = 0;
        float bestOffset = 1e9f;
        for (int r = 0; r < f.runwayCount; r++) {
            const Runway &rw = f.runways[r];
            if (rw.closed || altFt - rw.elevationFt > IN_USE_MAX_AGL_FT || lens[r] <= 0) {
                continue;
            }
            for (int k = 0; k < 2; k++) {
                const Way &w = ways[r][k];
                float off = fabsf(fmodf(ac.track - w.heading + 540.0f, 360.0f) - 180.0f);
                if (off > IN_USE_ALIGN_DEG) {
                    continue;
                }
                float rx = p.x - w.from.x, ry = p.y - w.from.y;
                float offset = fabsf(rx * w.uy - ry * w.ux);
                if (offset <= IN_USE_AIR_OFFSET_NM && offset < bestOffset) {
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
// `recent`: the ends not in use now that were lately, which keep a dimmer
// arrow - see InUseMemory.
void drawRunways(const ZoomFrame &f, const bool inUse[][2], const bool recent[][2]) {
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
    double nmPerDegLon = nmPerDegLonOf(f);
    for (int r = 0; r < f.runwayCount; r++) {
        const Runway &rw = f.runways[r];
        Vec a = toLocal(f, nmPerDegLon, rw.end[0].lat, rw.end[0].lon),
            b = toLocal(f, nmPerDegLon, rw.end[1].lat, rw.end[1].lon);
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
    // The numbers are the map's labels here, for the navaids' to keep off.
    for (int i = 0; i < placedCount && g_mapLabelCount < MAX_MAP_LABELS; i++) {
        g_mapLabels[g_mapLabelCount++] = placed[i];
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
            g_runwayBars.push_back({a.x, a.y, b.x, b.y, std::max(halfW, closed ? 6.0f : RUNWAY_ARROW_HALF_W)});
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
            if (!inUse[r][k] && !recent[r][k]) {
                continue;
            }
            // An end in use is the one traffic lands over and takes off from,
            // heading for the other: an arrowhead on the runway just inside
            // it, pointing that way, says which way it is being used at a
            // glance - and at an airport using a runway both ways, shows both.
            float ix = -e.ox, iy = -e.oy;  // into the runway, from this end
            float baseX = e.x + ix * RUNWAY_ARROW_INSET, baseY = e.y + iy * RUNWAY_ARROW_INSET;
            float tipX = baseX + ix * RUNWAY_ARROW_LEN, tipY = baseY + iy * RUNWAY_ARROW_LEN;
            float wx = -iy * RUNWAY_ARROW_HALF_W, wy = ix * RUNWAY_ARROW_HALF_W;
            canvas.fillTriangle((int)lroundf(tipX), (int)lroundf(tipY), (int)lroundf(baseX + wx),
                                (int)lroundf(baseY + wy), (int)lroundf(baseX - wx), (int)lroundf(baseY - wy),
                                inUse[r][k] ? colorText : colorMuted);
        }
    }
}

// The airport's control tower, where OpenStreetMap has one - from the card's
// airfields.bin, which the 3D view's airfields come from too: a chart's
// small picture of one whatever the range, its cab lit over its shaft, in the
// navaids' shade - and marked for their labels to keep off.
void drawTower(const ZoomFrame &f) {
    if (strcmp(g_zoomTower.code, f.code.c_str()) != 0) {
        strlcpy(g_zoomTower.code, f.code.c_str(), sizeof g_zoomTower.code);
        g_zoomTower.has = airfield::towerOf(f.code.c_str(), g_zoomTower.lat, g_zoomTower.lon);
    }
    if (!g_zoomTower.has) {
        return;
    }
    float pxPerNm = RADIUS / f.rangeNm;
    Vec v = toLocal(f, g_zoomTower.lat, g_zoomTower.lon);
    int x = CENTER_X + (int)lroundf(v.x * pxPerNm), y = CENTER_Y - (int)lroundf(v.y * pxPerNm);
    constexpr int HALF_W = 7, TOP = 12, BOTTOM = 8;  // its reach from (x, y), the middle of its box
    if (!insideScope(x - HALF_W, y - TOP, 2 * HALF_W + 1, TOP + BOTTOM + 1, 2)) {
        return;
    }
    auto &canvas = screen::canvas();
    canvas.drawFastVLine(x, y - 12, 3, colorRingText);      // its aerial
    canvas.drawFastHLine(x - 7, y - 9, 15, colorRingText);  // the roof, over the glass
    canvas.fillTriangle(x - 6, y - 8, x + 6, y - 8, x + 4, y - 4, colorMuted);  // the cab's glass, leaning
    canvas.fillTriangle(x - 6, y - 8, x + 4, y - 4, x - 4, y - 4, colorMuted);  // out, and its frames
    canvas.drawFastVLine(x - 2, y - 8, 5, colorBg);
    canvas.drawFastVLine(x + 2, y - 8, 5, colorBg);
    canvas.drawFastHLine(x - 4, y - 3, 9, colorRingText);   // its floor
    canvas.fillRect(x - 1, y - 2, 3, 10, colorRingText);     // the shaft
    canvas.drawFastHLine(x - 4, y + 8, 9, colorRingText);   // on the ground
    noteMark(x - HALF_W, y - TOP, 2 * HALF_W + 1, TOP + BOTTOM + 1);
}

bool zoomCanIn() { return g_zoomViewNm > ZOOM_VIEW_STEPS_NM[0] + 0.01f; }

// Where the radar has its ZOOM buttons, working the same way. ZOOM OUT always
// goes somewhere: at the frame's range, back to the radar.
void drawZoomViewButtons() {
    drawZoomButton(ZOOM_IN_Y, true, zoomCanIn() ? Look::PLAIN : Look::GREYED);
    drawZoomButton(ZOOM_OUT_Y, false, Look::PLAIN);
}

void drawZoomHeader() {
    auto &canvas = screen::canvas();
    canvas.fillRect(0, 0, PLOT_L, TITLE_BLOCK_H, colorBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(colorText);
    canvas.drawString(g_zoom.code, 16, TITLE_Y);
    canvas.setTextColor(colorMuted);
    canvas.setTextSize(2);
    int closed = 0;
    for (int r = 0; r < g_zoom.runwayCount; r++) {
        closed += g_zoom.runways[r].closed ? 1 : 0;
    }
    int open = g_zoom.runwayCount - closed;
    String what = g_zoom.runwayCount == 0 ? String("no runway data")
                                          : String(open) + (open == 1 ? " runway" : " runways");
    if (closed) {
        what += ", " + String(closed) + " closed";
    }
    canvas.drawString(what, 16, TITLE_LINE2_Y);
    drawMuteIcon();
    // FOLLOW where the radar has it, working the same way: a tap, then a
    // contact - one at the gate, say, to follow it out - with what it is
    // doing said under the title.
    if (g_zoomFollowHex.length()) {
        const char *following = "following ";
        canvas.drawString(following, 16, TITLE_LINE3_Y);
        canvas.setTextColor(colorHome);
        canvas.drawString(g_zoomFollowCallsign, 16 + canvas.textWidth(following), TITLE_LINE3_Y);
    } else if (g_zoomPicking) {
        canvas.drawString("tap a plane", 16, TITLE_LINE3_Y);
    }
    drawButton(FOLLOW_Y, g_zoomPicking ? "TAP PLANE" : "FOLLOW",
               g_zoomFollowHex.length() || g_zoomPicking ? Look::LIT : Look::PLAIN);
    draw3dButton(g_zoomFollowHex.length() > 0);
    drawRecButton();
    drawReplaysButton();
    // Back to the table, as the radar's Back is - the radar coming back to
    // this zoom - where ZOOM OUT, from the zoom's own framing, is to the radar.
    drawButton(BACK_Y, "Back", Look::PLAIN);
    // Where the radar has HOME, the first of the buttons it has no use for
    // here: the airport's card - see drawAirportCard().
    drawButton(HOME_Y, "INFO", Look::PLAIN);
    // And under it, where the radar has AIRPORT: back to the middle of the
    // runways, once the plot has been dragged off it.
    drawButton(AIRPORT_Y, "CENTRE", g_zoomLook.active ? Look::PLAIN : Look::GREYED);
    drawZoomViewButtons();
}

}  // namespace

bool radarZoomOnFollowed() { return g_zoomOnFollowed; }

float radarZoomViewNm() { return g_zoomViewNm < g_zoom.framedNm - 0.01f ? g_zoomViewNm : 0.0f; }

void radarZoomRefreshButtons() {
    ensureColors();
    drawZoomViewButtons();
}

void radarZoomSetViewNm(float nm) {
    g_zoomViewNm = nm > 0 ? std::min(nm, g_zoom.framedNm) : g_zoom.framedNm;
}

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
    g_zoomTower.code[0] = 0;  // looked up afresh
    g_zoomLook.active = false;
    g_zoomViewNm = g_zoom.rangeNm;
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

// Set round a draw of the live zoom at a range a drag's line has it at, with
// none of its traffic: the readouts that count the traffic are left off too.
bool g_zoomMapOnly = false;

// The zoom's plot and footer readouts, for frame `f`: what the live zoom and
// a replay of one both draw. As radarPlotDraw(): with `full` the caller has
// cleared the screen and flushes; without it, only the plot and the footer
// are erased and redrawn, and pushed as `push` says - or with `partial`,
// where only the traffic has moved since the last, only that: see Background.
//
// Zoomed in past its framing with an aircraft followed, it is the aircraft
// that stays in the middle - touching down, rolling out and taxiing to its
// stand at a range where the runways' middle would have lost it - with the
// runways passing under it, as the radar keeps one in the air there. True
// where it was.
bool zoomPlot(const ZoomFrame &framed, const std::vector<Aircraft> &aircraft,
              const std::vector<replay::TrailPoint> *trails, bool flash, const String &followHex, bool full,
              bool push, bool partial, uint32_t nowMs, InUseMemory &memory) {
    auto &canvas = screen::canvas();
    uint32_t t0 = micros();
    g_footings.kept = false;  // its own readouts where the radar's go
    ZoomFrame f = framed;
    bool onFollowed = false;
    if (followHex.length() && f.rangeNm < f.framedNm - 0.01f) {
        for (const Aircraft &a : aircraft) {
            if (a.hex == followHex && a.hasPos) {
                f.lat = a.lat;
                f.lon = a.lon;
                onFollowed = true;
                break;
            }
        }
    }
    float range = f.rangeNm;
    float step = range > 3.0f ? 1.0f : range > 1.0f ? 0.5f : 0.25f;
    bool inUse[MAX_ZOOM_RUNWAYS][2], recent[MAX_ZOOM_RUNWAYS][2];
    runwaysInUse(f, aircraft, inUse);
    portENTER_CRITICAL(&g_inUseMux);
    memory.update(f, inUse, nowMs, recent);
    portEXIT_CRITICAL(&g_inUseMux);
    static const std::vector<uint8_t> noneNew;  // the radar marks new contacts; here they are all just traffic
    g_followLabel.reserved = false;  // the radar's place for the followed one's label isn't the zoom's
    // The lead in proportion to the range, so closer in the vectors are no
    // longer on the plot than at the framing.
    float leadS = ZOOM_LEAD_S * std::min(1.0f, range / f.framedNm);
    ContactView view = {f.lat, f.lon, RADIUS / range, leadS, true, followHex};

    // As the radar's, with the runways in use over the range when there are
    // any - the ends in use now, or in a lull those lately.
    auto footers = [&](const String &count) {
        footerBox(nmText(range) + " nm range   rings every " + nmText(step) + " nm", 16, FOOTER_Y, false);
        String used, lately;
        for (int r = 0; r < f.runwayCount; r++) {
            for (int k = 0; k < 2; k++) {
                if (inUse[r][k]) {
                    used += String(" ") + f.runways[r].end[k].ident;
                } else if (recent[r][k]) {
                    lately += String(" ") + f.runways[r].end[k].ident;
                }
            }
        }
        if (used.length()) {
            footerBox("in use:" + used, 16, FOOTER_Y - LEGEND_OFFSET, false);
        } else if (lately.length()) {
            footerBox("last in use:" + lately, 16, FOOTER_Y - LEGEND_OFFSET, false);
        }
        // Not while a drag's line has it drawn without its traffic.
        if (!g_zoomMapOnly) {
            footerBox("o ground   ^ climb   v descent", 1264, FOOTER_Y - LEGEND_OFFSET, true);
            drawCount(count);
        }
    };

    // What the plot is of, all but its traffic, as the radar has it: the
    // runway ends marked in use go with it, as they are drawn under it.
    BackgroundKey key;
    memset(&key, 0, sizeof(key));  // padding and all, for the compare
    key.zoom = true;
    key.lat = f.lat;
    key.lon = f.lon;
    key.zoomNm = range;
    key.navaids = g_navaidsOn;
    key.mapOnly = g_zoomMapOnly;
    strncpy(key.centreCode, f.code.c_str(), sizeof(key.centreCode) - 1);
    for (int r = 0; r < f.runwayCount; r++) {
        for (int k = 0; k < 2; k++) {
            key.runwaysInUse |= (uint32_t)inUse[r][k] << (2 * r + k);
            key.runwaysRecent |= (uint32_t)recent[r][k] << (2 * r + k);
        }
    }
    // Not centred on the aircraft followed: the runways move under it.
    bool canKeep = partial && !onFollowed;
    bool keptFor = g_background.valid && sameBackground(key, g_background.key);
    if (!full && canKeep && keptFor && trafficCheap()) {
        restoreTraffic(false);
        canvas.setFont(&fonts::Font0);
        canvas.setClipRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);
        int plotted = 0, labelled = 0;
        uint32_t t1 = micros();
        drawContacts(aircraft, noneNew, trails, flash, view, plotted, labelled);
        uint32_t t2 = micros();
        canvas.clearClipRect();
        // As the radar's refresh has its footer: see refreshTraffic().
        String count = contactsText(plotted, labelled);
        bool recount = !g_zoomMapOnly && count != g_countShown;
        if (recount) {
            canvas.fillRect(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T, colorBg);
        }
        footers(count);
        g_backgroundUs += t1 - t0;
        g_contactsUs += t2 - t1;
        g_plotFrames++;
        pushTraffic(push, recount);
        g_pushUs += micros() - t2;
        return onFollowed;
    }

    uint32_t layerAt = cycleCount();
    if (!full) {
        screen::fillRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T, colorBg);
        canvas.fillRect(FOOT_L, FOOT_T, FOOT_R - FOOT_L, FOOT_B - FOOT_T, colorBg);
        layerAt = layerDone(LAYER_FILL, layerAt);
    }
    canvas.setFont(&fonts::Font0);
    drawScope(range, step);
    layerAt = layerDone(LAYER_SCOPE, layerAt);
    g_mapLabelCount = 0;

    canvas.setClipRect(PLOT_L, PLOT_T, PLOT_R - PLOT_L, PLOT_B - PLOT_T);
    drawRunways(f, inUse, recent);
    layerAt = layerDone(LAYER_RUNWAYS, layerAt);
    drawTower(f);
    layerAt = layerDone(LAYER_TOWER, layerAt);
    // Over the runways, which an airport's own often sits beside.
    g_navaidHitCount = 0;
    if (g_navaidsOn) {
        drawNavaids(f.lat, f.lon, range);
    }
    layerDone(LAYER_NAVAIDS, layerAt);
    if (canKeep && g_background.pending && sameBackground(key, g_background.key)) {
        keepBackground();
        g_background.pending = false;
    } else if (!keptFor) {
        g_background.valid = false;
        g_background.pending = canKeep;
        g_background.key = key;
    }
    g_background.wholeMs = millis();
    int plotted = 0, labelled = 0;
    uint32_t t1 = micros();
    drawContacts(aircraft, noneNew, trails, flash, view, plotted, labelled);
    uint32_t t2 = micros();
    g_backgroundUs += t1 - t0;
    g_contactsUs += t2 - t1;
    g_plotFrames++;
    canvas.clearClipRect();

    if (!full) {
        canvas.fillRect(0, IN_USE_BOX_T, PLOT_L, RANGE_BOX_T + BOX_H - IN_USE_BOX_T, colorBg);
    }
    footers(contactsText(plotted, labelled));

    if (!full) {
        uint32_t t3 = micros();
        pushPlot(push, IN_USE_BOX_T);
        g_pushUs += micros() - t3;
    }
    return onFollowed;
}

// The frame a replay last drew its zoom in, kept apart from the live zoom's.
ZoomFrame g_replayZoom;

}  // namespace

void radarZoomDraw(const std::vector<Aircraft> &aircraft, bool full) {
    ensureColors();
    if (full) {
        g_choiceCount = 0;
        if (!g_cardSurvivesRedraw) {
            g_card = nullptr;
        }
        g_cardSurvivesRedraw = false;
        g_airportCard = nullptr;
        screen::clear(colorBg);
        drawZoomHeader();
        drawClock();
    }
    // With a list or card up, a refresh is drawn under it and pushed along
    // with it - as it is with a drag's line over it.
    bool following = g_zoomFollowHex.length() > 0;
    bool dragged = g_drag.showing && g_drag.zoom;
    ZoomFrame view = g_zoom;
    view.rangeNm = g_zoomViewNm;
    // Dragged off the middle of its runways, it is centred where it was let
    // go - until something is followed, which has a centre of its own.
    if (following) {
        g_zoomLook.active = false;
    }
    if (g_zoomLook.active) {
        view.lat = g_zoomLook.lat;
        view.lon = g_zoomLook.lon;
    }
    g_zoomView = {view.lat, view.lon, RADIUS / view.rangeNm, g_zoom.lat, g_zoom.lon, g_zoom.lat, g_zoom.lon};
    // A line dragged past the edge has the plot at a range that takes it,
    // with its runways and none of its traffic.
    if (dragged) {
        view.rangeNm = dragRangeNm(view.rangeNm);
    }
    bool mapOnly = view.rangeNm != g_zoomViewNm;
    if (!dragged || full) {
        g_dragCache.valid = false;
    }
    if (dragged && !following && dragKept(true, view.rangeNm, view.lat, view.lon)) {
        moveDragLine(view.rangeNm);
        return;
    }
    static const std::vector<Aircraft> noAircraft;
    if (!full && overlayUp()) {
        screen::keepApart();  // as the radar's, with its INFO card up
    }
    g_zoomMapOnly = mapOnly;
    g_zoomOnFollowed = zoomPlot(view, mapOnly ? noAircraft : aircraft, nullptr, true, g_zoomFollowHex, full,
                                !overlayUp() && !dragged, !overlayUp() && !dragged && !mapOnly, millis(),
                                g_liveInUse);
    g_zoomMapOnly = false;
    // Following into the zoom, its height against the field's.
    if (following) {
        radarTelemetryDraw(LIVE_TEL_X, LIVE_TEL_Y, g_heights, millis(),
                           g_zoom.runwayCount ? g_zoom.runways[0].elevationFt : INT32_MIN, g_lostS);
    }
    if (g_choiceCount) {
        drawChooser();
    }
    if (g_card) {
        drawCard();
    }
    if (g_airportCard) {
        drawAirportCard();
    }
    if (dragged) {
        if (!overlayUp() && !following) {
            keepPlotForDrag(true, view.rangeNm, view.lat, view.lon);
        }
        drawDragLine(view.rangeNm);
    }
    if (full || overlayUp() || following || dragged) {
        screen::flush();
    }
}

bool radarZoomPlotDraw(const RadarScene &scene, const String &code, bool full) {
    ensureColors();
    if (g_replayZoom.code != code && !frameZoom(code, g_replayZoom)) {
        return false;
    }
    ZoomFrame view = g_replayZoom;
    if (scene.zoomRangeNm > 0) {
        view.rangeNm = std::min(scene.zoomRangeNm, view.framedNm);
    }
    zoomPlot(view, scene.aircraft, scene.trails, scene.flash, scene.followHex, full, scene.push, false, scene.nowMs,
             g_replayInUse);
    return true;
}

void radarScreenSetNavaids(bool on) { g_navaidsOn = on; }

void radarScreenSetCoast(bool on) { g_coastOn = on; }

void radarScreenSetRings(bool on) { g_ringsOn = on; }

void radarScreenSetMuted(bool muted, bool onScreen) {
    g_soundMuted = muted;
    if (onScreen) {
        ensureColors();
        drawMuteIcon();
        screen::flush();
    }
}

void radarScreenSetClock(ClockZone zone) {
    if (zone != g_clockZone) {
        g_clockZone = zone;
        g_clockMinute = -1;  // drawn again at the next tick
    }
}

void radarScreenSetClockAt(bool on, double lat, double lon) {
    int zone = on ? timeZoneAt(lat, lon) : -1;
    if (zone != g_clockAt) {
        g_clockAt = zone;
        g_clockMinute = -1;  // drawn again at the next tick
    }
}

void radarScreenTickClock() {
    if (g_clockZone == ClockZone::OFF) {
        return;
    }
    time_t now = time(nullptr);
    struct tm utc;
    gmtime_r(&now, &utc);
    if (now > 1600000000 && utc.tm_hour * 60 + utc.tm_min != g_clockMinute) {
        ensureColors();
        drawClock();
        screen::flush();
    }
}

bool radarScreenHitMute(int x, int y) { return abs(x - MUTE_CX) <= MUTE_HIT && abs(y - MUTE_CY) <= MUTE_HIT; }

bool radarScreenHitFilter(int x, int y) {
    return abs(x - FILTER_CX) <= FILTER_HIT && abs(y - FILTER_CY) <= FILTER_HIT && !radarScreenHitMute(x, y);
}

void radarLiveRunwaysInUse(recorder::RunwaysInUse &out) {
    out = recorder::RunwaysInUse();
    char code[5];
    uint32_t ms[MAX_ZOOM_RUNWAYS][2];
    bool seen[MAX_ZOOM_RUNWAYS][2];
    portENTER_CRITICAL(&g_inUseMux);
    memcpy(code, g_liveInUse.code, sizeof(code));
    memcpy(ms, g_liveInUse.ms, sizeof(ms));
    memcpy(seen, g_liveInUse.seen, sizeof(seen));
    portEXIT_CRITICAL(&g_inUseMux);
    int count = 0;
    const Runway *runways = code[0] ? runwaysAt(code, count) : nullptr;
    uint32_t now = millis();
    for (int r = 0; r < std::min(count, MAX_ZOOM_RUNWAYS); r++) {
        for (int k = 0; k < 2; k++) {
            if (seen[r][k] && now - ms[r][k] <= RUNWAY_IN_USE_MEMORY_MS) {
                recorder::RunwaysInUse::End e = {};
                strlcpy(e.ident, runways[r].end[k].ident, sizeof(e.ident));
                e.agoS = (now - ms[r][k]) / 1000;
                out.ends.push_back(e);
            }
        }
    }
    if (!out.ends.empty()) {
        out.code = code;
    }
}

void radarReplayRunwaysInUse(const recorder::RunwaysInUse &seed) {
    portENTER_CRITICAL(&g_inUseMux);
    g_replayInUse.code[0] = 0;  // so its next draw starts afresh, from this
    portEXIT_CRITICAL(&g_inUseMux);
    g_replayInUse.seed = seed;
}

ZoomAction radarZoomHandleTouch(int x, int y, String &outHex) {
    if (!overlayUp() && hit3dButton(x, y)) {
        return ZoomAction::OPEN_3D;
    }
    if (!overlayUp()) {
        int b = buttonAt(x, y);
        if (b == BTN_BACK) {
            return ZoomAction::BACK;
        }
        if (b == BTN_FOLLOW) {
            return ZoomAction::TOGGLE_FOLLOW;
        }
        if (b == BTN_REC) {
            return ZoomAction::TOGGLE_RECORD;
        }
        if (b == BTN_REPLAYS) {
            return ZoomAction::OPEN_RECORDINGS;
        }
        if (b == BTN_AIRPORT) {
            // CENTRE, which has somewhere to go back to once it is dragged.
            if (!g_zoomLook.active) {
                return ZoomAction::NONE;
            }
            g_zoomLook.active = false;
            return ZoomAction::REDRAW;
        }
        if (b == BTN_HOME) {
            // INFO: the card goes up over the plot as it stands, and the
            // plot's refreshes are drawn under it until a tap takes it off.
            openAirportCard(g_zoom.code);
            if (g_airportCard) {
                drawAirportCard();
                screen::flush();
            }
            return ZoomAction::NONE;
        }
        if (b == BTN_ZOOM_IN && zoomCanIn()) {
            float next = ZOOM_VIEW_STEPS_NM[0];
            for (float r : ZOOM_VIEW_STEPS_NM) {
                if (r < g_zoomViewNm - 0.01f) {
                    next = r;
                }
            }
            g_zoomViewNm = next;
            return ZoomAction::RANGE;
        }
        if (b == BTN_ZOOM_OUT) {
            if (g_zoomViewNm >= g_zoom.framedNm - 0.01f) {
                return ZoomAction::UNZOOM;
            }
            float next = g_zoom.framedNm;
            for (float r : ZOOM_VIEW_STEPS_NM) {
                if (r > g_zoomViewNm + 0.01f) {
                    next = std::min(r, g_zoom.framedNm);
                    break;
                }
            }
            g_zoomViewNm = next;
            return ZoomAction::RANGE;
        }
    }
    switch (tapTarget(x, y, false, outHex)) {
    case Target::AIRCRAFT:
        return ZoomAction::SELECT;
    case Target::DISMISSED:
        return ZoomAction::DISMISS;
    default:
        break;
    }
    // On the plot with nothing under it, and nothing put up by it: it may be
    // going to drag - see radarDragTo().
    if (!overlayUp() && !g_zoomFollowHex.length() && insideRing(x, y)) {
        startDrag(x, y, true);
        return ZoomAction::DRAG;
    }
    return ZoomAction::NONE;
}

RadarAction radarScreenHandleTouch(int x, int y, String &outHex, String &outAirport) {
    // With the list up, every tap is the list's: outside it is a dismissal.
    if (!overlayUp() && hit3dButton(x, y)) {
        return RadarAction::OPEN_3D;
    }
    if (!overlayUp()) {
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
        case BTN_ZOOM_IN:
            return g_canZoomIn ? RadarAction::ZOOM_IN : RadarAction::NONE;
        case BTN_ZOOM_OUT:
            return g_canZoomOut ? RadarAction::ZOOM_OUT : RadarAction::NONE;
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
    // On the plot with nothing under it, and nothing put up by it: it may be
    // going to drag - see radarDragTo().
    if (!overlayUp() && g_follow != FollowButton::ON && insideRing(x, y)) {
        startDrag(x, y, false);
        return RadarAction::DRAG;
    }
    return RadarAction::NONE;
}

bool radarDragTo(int x, int y) {
    Drag &d = g_drag;
    if (!d.down) {
        return false;
    }
    int dx = x - d.startX, dy = y - d.startY;
    if (!d.showing && dx * dx + dy * dy < DRAG_MIN_PX * DRAG_MIN_PX) {
        return false;  // still a tap that found nothing, or a finger settling
    }
    // The middle goes the way the finger has, and as far at the scale the
    // plot had when it landed: where that is, on the plot's own flat earth.
    const PlotView &v = d.view;
    double nmPerDegLon = 60.0 * std::max(cos(v.lat * M_PI / 180.0), 0.01);
    double lat = v.lat - dy / v.pxPerNm / 60.0;
    double lon = v.lon + dx / v.pxPerNm / nmPerDegLon;
    if (d.zoom) {
        // The zoom's is no further from the middle of its runways than its
        // framing reaches, which is as far as its traffic is fetched.
        double ownNmPerDegLon = 60.0 * std::max(cos(v.ownLat * M_PI / 180.0), 0.01);
        double east = (lon - v.ownLon) * ownNmPerDegLon, north = (lat - v.ownLat) * 60.0;
        double out = hypot(east, north), reach = g_zoom.framedNm;
        if (out > reach) {
            lat = v.ownLat + north * reach / out / 60.0;
            lon = v.ownLon + east * reach / out / ownNmPerDegLon;
        }
    }
    // The radar's goes where it likes - short of the poles, and round the
    // date line rather than off the end of it.
    lat = std::min(std::max(lat, -85.0), 85.0);
    float eastNm = (float)((lon - v.lon) * nmPerDegLon), northNm = (float)((lat - v.lat) * 60.0);
    if (d.showing && fabsf(eastNm - d.eastNm) * v.pxPerNm < 2 && fabsf(northNm - d.northNm) * v.pxPerNm < 2) {
        return false;  // nowhere new enough to draw again
    }
    d.showing = true;
    d.eastNm = eastNm;
    d.northNm = northNm;
    d.lat = lat;
    d.lon = lon > 180.0 ? lon - 360.0 : lon < -180.0 ? lon + 360.0 : lon;
    d.nm = hypotf(eastNm, northNm);
    d.bearing = atan2f(eastNm, northNm) * 180.0f / (float)M_PI;
    return true;
}

bool radarDragEnd() {
    Drag d = g_drag;
    g_drag = Drag();
    if (!d.showing) {
        return false;
    }
    // Let go back where it started, it has gone nowhere - but the line has to
    // come off either way.
    if (d.nm * d.view.pxPerNm >= DRAG_MIN_PX) {
        FreeLook &look = d.zoom ? g_zoomLook : g_radarLook;
        look.active = true;
        look.lat = d.lat;
        look.lon = d.lon;
        look.fromLat = d.zoom ? d.view.ownLat : d.view.homeLat;
        look.fromLon = d.zoom ? d.view.ownLon : d.view.homeLon;
    }
    return true;
}

bool radarScreenFreeLook(double &lat, double &lon) {
    lat = g_radarLook.lat;
    lon = g_radarLook.lon;
    return g_radarLook.active;
}

void radarScreenSetMapOnly(bool on) { g_mapOnly = on; }

void radarScreenRedrawTitle() {
    ensureColors();
    drawTitle();
    screen::flush();  // on its own, so it isn't taken into a box with the plot after it
}
void radarScreenSetFiltered(bool on) { g_filtered = on; }

void radarScreenSetAutoRange(bool on) { g_autoRange = on; }

void radarDrawFunnel(int cx, int cy, int r, bool lit) {
    ensureColors();
    drawFunnel(cx, cy, r, lit);
}

float radarFastestKt() { return g_fastestKt; }

void radarScreenLookAt(double lat, double lon, double homeLat, double homeLon) {
    g_radarLook.active = true;
    g_radarLook.lat = lat;
    g_radarLook.lon = lon;
    g_radarLook.fromLat = homeLat;
    g_radarLook.fromLon = homeLon;
}

bool radarScreenEndFreeLook() {
    bool was = g_radarLook.active;
    g_radarLook.active = false;
    return was;
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

// What the panel was last drawn from, for it to be drawn again only when
// that changes - or once anything else has been drawn over it - rather than
// every frame, where it was 4-6ms of a following frame on the radar: where
// it is, the field's height and how long the aircraft has been lost; the
// chart's left edge, on its own grid of pixels; which reports are in the
// chart and up to now; and what those say.
struct TelemetryKey {
    int x, y;
    int32_t fieldFt;
    uint32_t lostS;
    int64_t edge;
    uint32_t first, count, sum;
    bool operator==(const TelemetryKey &o) const {
        return x == o.x && y == o.y && fieldFt == o.fieldFt && lostS == o.lostS && edge == o.edge &&
               first == o.first && count == o.count && sum == o.sum;
    }
};
TelemetryKey g_telemetryKey;
bool g_telemetryKept = false;
// How often it was drawn and how often left as it was, and the time the
// drawing took, for the [perf] line.
uint32_t g_telemetryDrawn = 0, g_telemetryLeft = 0;
uint64_t g_telemetryUs = 0;

}  // namespace

void radarSetHeights(const std::vector<FollowSample> &samples, uint32_t lostS) {
    g_heights = samples;
    g_lostS = lostS;
}

void radarTelemetryDraw(int x, int y, const std::vector<FollowSample> &samples, uint32_t nowMs, int32_t fieldFt,
                        uint32_t lostS) {
    uint32_t t0 = micros();
    ensureColors();
    auto &canvas = screen::canvas();

    // The latest report by now, and the window of them the chart covers -
    // whose left edge is taken on the chart's own grid, a pixel to every
    // TELEMETRY_WINDOW_MS / (TELEMETRY_W - 3), 2.2s: the points then step
    // left together, a pixel at a time, rather than each at its own moment,
    // and the chart stays as it is in between. A point is at most a pixel
    // right of where the window's own edge would put it.
    uint32_t from = nowMs > TELEMETRY_WINDOW_MS ? nowMs - TELEMETRY_WINDOW_MS : 0;
    int64_t edge = (int64_t)(TELEMETRY_W - 3) * from / TELEMETRY_WINDOW_MS;
    const FollowSample *latest = nullptr;
    int32_t highest = fieldFt == INT32_MIN ? 0 : fieldFt;
    uint32_t count = 0, first = UINT32_MAX;
    for (const FollowSample &s : samples) {
        if (s.ms > nowMs) {
            break;
        }
        latest = &s;
        if (s.ms >= from && first == UINT32_MAX) {
            first = count;
        }
        count++;
        if (s.ms >= from && s.hasAlt) {
            highest = std::max(highest, s.altFt);
        }
    }
    // What those reports say - the chart's, and the latest's if it is older
    // than the chart - summed, FNV-1a, for the key.
    uint32_t sum = 2166136261u;
    for (uint32_t i = std::min(first, count ? count - 1 : 0); i < count; i++) {
        const FollowSample &s = samples[i];
        for (uint32_t v : {s.ms, (uint32_t)s.altFt, (uint32_t)s.vsFpm, (uint32_t)s.gsKt,
                           (uint32_t)(s.hasAlt | s.ground << 1 | s.hasVs << 2 | s.hasGs << 3)}) {
            sum = (sum ^ v) * 16777619u;
        }
    }
    TelemetryKey key = {x, y, fieldFt, lostS, edge, first, count, sum};
    if (g_telemetryKept && key == g_telemetryKey && !screen::touched()) {
        // Left as a redraw leaves it, for whatever is drawn next.
        canvas.clearClipRect();
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(2);
        canvas.setTextColor(colorFaint);
        canvas.setTextDatum(TR_DATUM);
        g_telemetryLeft++;
        return;
    }
    g_telemetryKey = key;
    g_telemetryKept = true;
    // By the PPA, which fills at 355MB/s where the CPU writes 67 - marked
    // dirty as it goes.
    screen::fillRect(x, y, TELEMETRY_W, TELEMETRY_H, colorBg);
    canvas.setFont(&fonts::Font0);

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
    auto px = [&](uint32_t ms) { return cx + 1 + (int)((int64_t)(cw - 3) * ms / TELEMETRY_WINDOW_MS - edge); };
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
    // Watched from here, now its own drawing is marked.
    screen::watch(x, y, TELEMETRY_W, TELEMETRY_H);
    g_telemetryDrawn++;
    g_telemetryUs += micros() - t0;
}

String radarTakeTelemetry() {
    String s = "-";
    if (g_telemetryDrawn + g_telemetryLeft != 0) {
        s = String(g_telemetryDrawn) + "x drawn";
        if (g_telemetryDrawn != 0) {
            s += " avg " + String(g_telemetryUs / 1000.0f / g_telemetryDrawn, 1) + "ms";
        }
        s += ", " + String(g_telemetryLeft) + "x left";
    }
    g_telemetryDrawn = g_telemetryLeft = 0;
    g_telemetryUs = 0;
    return s;
}



