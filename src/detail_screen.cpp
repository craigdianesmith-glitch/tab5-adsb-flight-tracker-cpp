#include "detail_screen.h"

#include "route.h"

#include <M5Unified.h>
#include <math.h>

#include "aircraft_db.h"
#include "radar_screen.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int FOLLOW_W = 124, FOLLOW_X = BACK_X - 12 - FOLLOW_W;
constexpr int WATCH_W = 124, WATCH_X = FOLLOW_X - 12 - WATCH_W;
// SAY, left of the leftmost of those shown - see sayX().
constexpr int SAY_W = 100;
// The airline's banner, under the buttons.
constexpr int BANNER_Y = BACK_Y + BACK_H + 4, BANNER_H = 32;
// Label column widened to fit the bigger label font (e.g. "ALTITUDE (BARO)"
// at size 3); value width capped so long values (e.g. "Cessna Citation CJ2")
// can't run into the next column - field() shrinks the font if it doesn't fit.
constexpr int COL1_LABEL_X = 40, COL1_VALUE_X = 360, COL1_VALUE_MAX_W = 320;
constexpr int COL2_LABEL_X = 700, COL2_VALUE_X = 1020, COL2_VALUE_MAX_W = 234;
constexpr int ROW0_Y = 120, ROW_H = 80;
// The route's two airports, as buttons - each opens its zoom - either side
// of a ">", in its row's value column.
constexpr int ROUTE_Y = ROW0_Y + 3 * ROW_H;
constexpr int ROUTE_BTN_Y = ROUTE_Y - 12, ROUTE_BTN_H = 48, ROUTE_BTN_PAD = 22, ROUTE_ARROW_W = 48;

Aircraft g_ac;
bool g_hasAc = false;
DetailFollow g_follow = DetailFollow::NONE;
DetailWatch g_watch = DetailWatch::NONE;
// The route's airports as drawn, where they are buttons: an airport the
// zoom hasn't got is written as it is, and is no button.
String g_fromCode, g_toCode;
int g_fromX = 0, g_fromW = 0, g_toX = 0, g_toW = 0;

uint16_t colorBg, colorWhite, colorGrey, colorBtnBg, colorClimb, colorDescend, colorLevel;

String statusValue(const String &status) {
    if (status == "CLIMB") return "Climbing";
    if (status == "DESCEND") return "Descending";
    if (status == "LEVEL") return "Level";
    if (status == "TAXI") return "Taxiing";
    if (status == "GROUND") return "On ground";
    return status;
}

uint16_t statusColor(const String &status) {
    if (status == "CLIMB") return colorClimb;
    if (status == "DESCEND") return colorDescend;
    return colorLevel;
}

void field(int labelX, int valueX, int maxValueW, int y, const char *label, const String &value,
           uint16_t valueColor) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextColor(colorGrey);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(label, labelX, y);

    // Same fixed-width bitmap font as the label rather than a proportional
    // TrueType-ish font - exact width math (6px * size), so the shrink-to-fit
    // below is reliable instead of guessing at proportional glyph widths.
    canvas.setTextColor(valueColor);
    int size = 3;
    canvas.setTextSize(size);
    while (size > 1 && canvas.textWidth(value) > maxValueW) {
        size--;
        canvas.setTextSize(size);
    }
    canvas.drawString(value, valueX, y);
}

// Where SAY goes: beside the leftmost button shown - a replay's details have
// neither FOLLOW nor WATCH.
int sayX() {
    int left = g_watch != DetailWatch::NONE ? WATCH_X : g_follow != DetailFollow::NONE ? FOLLOW_X : BACK_X;
    return left - 12 - SAY_W;
}

// A route's airport at x: a button if the zoom has it, as text if not.
// Returns how wide it is.
int routeAirport(const String &code, int x) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    double lat, lon;
    float range;
    bool zoomable = radarZoomFrame(code, lat, lon, range);
    int w = canvas.textWidth(code) + (zoomable ? 2 * ROUTE_BTN_PAD : 0);
    if (zoomable) {
        canvas.fillRoundRect(x, ROUTE_BTN_Y, w, ROUTE_BTN_H, 6, colorBtnBg);
        canvas.drawRoundRect(x, ROUTE_BTN_Y, w, ROUTE_BTN_H, 6, colorGrey);
    }
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(ML_DATUM);
    canvas.drawString(code, x + (zoomable ? ROUTE_BTN_PAD : 0), ROUTE_BTN_Y + ROUTE_BTN_H / 2);
    return zoomable ? w : 0;
}

}  // namespace

void detailScreenSet(const Aircraft &ac, DetailFollow follow, DetailWatch watch) {
    g_ac = ac;
    g_hasAc = true;
    g_follow = follow;
    g_watch = watch;
}

void detailScreenDraw() {
    auto &canvas = screen::canvas();
    if (!colorsReady) {
        colorBg = M5.Display.color565(0x10, 0x14, 0x18);
        colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
        colorGrey = M5.Display.color565(0x88, 0x88, 0x88);
        colorBtnBg = M5.Display.color565(0x2C, 0x3E, 0x50);
        colorClimb = M5.Display.color565(0x2E, 0xCC, 0x71);
        colorDescend = M5.Display.color565(0xF3, 0x9C, 0x12);
        colorLevel = M5.Display.color565(0xFF, 0xFF, 0xFF);
        colorsReady = true;
    }
    if (!g_hasAc) {
        return;
    }

    // Whole-screen repaint, so hand the clear to the PPA and let the flush
    // below push the lot in one transfer.
    screen::clear(colorBg);
    canvas.setFont(&fonts::Font0);

    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    // Callsign alone: the registration has a field of its own below, and
    // repeating it up here read as noise - worst of all for the aircraft
    // whose callsign *is* the registration, where the title came out as
    // "GSGTS  (G-SGTS)".
    canvas.drawString(g_ac.callsign, 16, 16);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    // Follow it from here as from the radar: no need to go back and find it
    // among the rest.
    if (g_follow != DetailFollow::NONE) {
        canvas.fillRoundRect(FOLLOW_X, BACK_Y, FOLLOW_W, BACK_H, 6, colorBtnBg);
        canvas.drawString(g_follow == DetailFollow::FOLLOW    ? "FOLLOW"
                          : g_follow == DetailFollow::VIEW_3D ? "3D"
                                                              : "UNFOLLOW",
                          FOLLOW_X + FOLLOW_W / 2, BACK_Y + BACK_H / 2);
    }
    if (g_watch != DetailWatch::NONE) {
        canvas.fillRoundRect(WATCH_X, BACK_Y, WATCH_W, BACK_H, 6, colorBtnBg);
        canvas.drawString(g_watch == DetailWatch::WATCH ? "WATCH" : "UNWATCH", WATCH_X + WATCH_W / 2,
                          BACK_Y + BACK_H / 2);
    }
    // Its details said aloud, now, whether or not read aloud is on.
    canvas.fillRoundRect(sayX(), BACK_Y, SAY_W, BACK_H, 6, colorBtnBg);
    canvas.drawString("SAY", sayX() + SAY_W / 2, BACK_Y + BACK_H / 2);

    // Livery banner: no real logo/image assets available, so this is the
    // airline's actual brand colour as a drawn band, not a bitmap. Clear of
    // the buttons above it, whose bottom 4px it covered from y=48.
    AirlineInfo airline;
    if (lookupAirline(g_ac.callsign, airline)) {
        uint16_t bandColor = M5.Display.color565((airline.color >> 16) & 0xFF, (airline.color >> 8) & 0xFF,
                                                   airline.color & 0xFF);
        canvas.fillRect(16, BANNER_Y, 1248, BANNER_H, bandColor);
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(3);
        canvas.setTextColor(colorWhite);
        canvas.setTextDatum(ML_DATUM);
        canvas.drawString(airline.name, 26, BANNER_Y + BANNER_H / 2);
        if (airline.cargo) {
            // A tag beside the name, outlined in the banner's own white.
            int x = 26 + canvas.textWidth(airline.name) + 18;
            canvas.setTextSize(2);
            int w = canvas.textWidth("CARGO") + 16, h = BANNER_H - 8;
            canvas.drawRoundRect(x, BANNER_Y + 4, w, h, 4, colorWhite);
            canvas.drawRoundRect(x + 1, BANNER_Y + 5, w - 2, h - 2, 3, colorWhite);
            canvas.setTextDatum(MC_DATUM);
            canvas.drawString("CARGO", x + w / 2, BANNER_Y + BANNER_H / 2);
        }
    }

    canvas.drawFastHLine(16, BANNER_Y + BANNER_H + 8, 1248, colorGrey);

    String altBaro = (g_ac.altStr == "GND")  ? g_ac.altStr
                     : (g_ac.altStr == "?") ? String("unknown")
                                            : (g_ac.altStr + " ft");
    String speed = (g_ac.speedStr == "?") ? String("unknown") : (g_ac.speedStr + " kt");
    String altGeom = g_ac.hasAltGeom ? (String(g_ac.altGeom) + " ft") : "unknown";
    String heading = g_ac.hasTrack ? (String((int)g_ac.track) + " deg") : "unknown";
    String vrate = g_ac.hasVertRate ? (String(g_ac.vertRate >= 0 ? "+" : "") + String((int)g_ac.vertRate) + " ft/min")
                                     : "unknown";
    String dist = g_ac.hasDist ? (String((int)lroundf(g_ac.distNm)) + " nm") : "unknown";
    String type = g_ac.type.length() ? lookupAircraftType(g_ac.type, g_ac.military) : "unknown";
    // Where it is from and going to, where adsbdb knows: in its row the
    // airports' codes, and in the footer their towns. In place of the size
    // category the transponder sends, which said little to anyone.
    Route route;
    RouteState routeState = routeFor(g_ac.callsign, route);
    String routeText = routeState == RouteState::FOUND     ? route.fromCode + " > " + route.toCode
                       : routeState == RouteState::LOOKING ? String("looking up...")
                                                           : String("unknown");
    String squawk = g_ac.squawk.length() ? g_ac.squawk : "unknown";
    String reg = g_ac.reg.length() ? g_ac.reg : "unknown";

    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 0 * ROW_H, "CALLSIGN", g_ac.callsign, colorWhite);
    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 1 * ROW_H, "REGISTRATION", reg, colorWhite);
    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 2 * ROW_H, "TYPE", type, colorWhite);
    g_fromCode = g_toCode = "";
    g_fromW = g_toW = 0;
    if (routeState == RouteState::FOUND) {
        field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROUTE_Y, "ROUTE", "", colorWhite);
        g_fromCode = route.fromCode;
        g_toCode = route.toCode;
        g_fromX = COL1_VALUE_X;
        g_fromW = routeAirport(g_fromCode, g_fromX);
        int arrowX = g_fromX + (g_fromW ? g_fromW : canvas.textWidth(g_fromCode));
        canvas.setTextColor(colorGrey);
        canvas.setTextDatum(MC_DATUM);
        canvas.drawString(">", arrowX + ROUTE_ARROW_W / 2, ROUTE_BTN_Y + ROUTE_BTN_H / 2);
        g_toX = arrowX + ROUTE_ARROW_W;
        g_toW = routeAirport(g_toCode, g_toX);
    } else {
        field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROUTE_Y, "ROUTE", routeText, colorGrey);
    }
    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 4 * ROW_H, "SQUAWK", squawk, colorWhite);
    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 5 * ROW_H, "STATUS", statusValue(g_ac.status),
          statusColor(g_ac.status));

    field(COL2_LABEL_X, COL2_VALUE_X, COL2_VALUE_MAX_W, ROW0_Y + 0 * ROW_H, "ALTITUDE (BARO)", altBaro, colorWhite);
    field(COL2_LABEL_X, COL2_VALUE_X, COL2_VALUE_MAX_W, ROW0_Y + 1 * ROW_H, "ALTITUDE (GEOM)", altGeom, colorWhite);
    field(COL2_LABEL_X, COL2_VALUE_X, COL2_VALUE_MAX_W, ROW0_Y + 2 * ROW_H, "GROUND SPEED", speed, colorWhite);
    field(COL2_LABEL_X, COL2_VALUE_X, COL2_VALUE_MAX_W, ROW0_Y + 3 * ROW_H, "HEADING", heading, colorWhite);
    field(COL2_LABEL_X, COL2_VALUE_X, COL2_VALUE_MAX_W, ROW0_Y + 4 * ROW_H, "VERTICAL RATE", vrate, colorWhite);
    field(COL2_LABEL_X, COL2_VALUE_X, COL2_VALUE_MAX_W, ROW0_Y + 5 * ROW_H, "DISTANCE", dist, colorWhite);

    canvas.drawFastHLine(16, ROW0_Y + 6 * ROW_H - 20, 1248, colorGrey);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextColor(colorGrey);
    canvas.setTextDatum(TL_DATUM);
    String footer = "ICAO " + g_ac.hex;
    if (routeState == RouteState::FOUND && route.fromCity.length() && route.toCity.length()) {
        footer += "    " + route.fromCity + " to " + route.toCity;
    }
    if (g_ac.hasPos) {
        footer += "    Position " + String(g_ac.lat, 4) + ", " + String(g_ac.lon, 4);
    }
    canvas.drawString(footer, 16, ROW0_Y + 6 * ROW_H);

    screen::flush();
}

DetailAction detailScreenHandleTouch(int x, int y) {
    if (y >= ROUTE_BTN_Y - 8 && y < ROUTE_BTN_Y + ROUTE_BTN_H + 8) {
        if (g_fromW && x >= g_fromX - 8 && x < g_fromX + g_fromW + 8) {
            return DetailAction::OPEN_FROM;
        }
        if (g_toW && x >= g_toX - 8 && x < g_toX + g_toW + 8) {
            return DetailAction::OPEN_TO;
        }
    }
    if (y < BACK_Y || y >= BACK_Y + BACK_H) {
        return DetailAction::NONE;
    }
    if (x >= sayX() && x < sayX() + SAY_W) {
        return DetailAction::SAY;
    }
    if (x >= BACK_X && x < BACK_X + BACK_W) {
        return DetailAction::BACK;
    }
    if (x >= FOLLOW_X && x < FOLLOW_X + FOLLOW_W && g_follow != DetailFollow::NONE) {
        return g_follow == DetailFollow::FOLLOW    ? DetailAction::FOLLOW
               : g_follow == DetailFollow::VIEW_3D ? DetailAction::VIEW_3D
                                                   : DetailAction::UNFOLLOW;
    }
    if (x >= WATCH_X && x < WATCH_X + WATCH_W && g_watch != DetailWatch::NONE) {
        return DetailAction::WATCH;
    }
    return DetailAction::NONE;
}

String detailScreenRouteCode(bool from) { return from ? g_fromCode : g_toCode; }
