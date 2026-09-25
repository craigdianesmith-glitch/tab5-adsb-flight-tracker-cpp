#include "detail_screen.h"

#include <M5Unified.h>
#include <math.h>

#include "aircraft_db.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
// Label column widened to fit the bigger label font (e.g. "ALTITUDE (BARO)"
// at size 3); value width capped so long values (e.g. "Cessna Citation CJ2")
// can't run into the next column - field() shrinks the font if it doesn't fit.
constexpr int COL1_LABEL_X = 40, COL1_VALUE_X = 360, COL1_VALUE_MAX_W = 320;
constexpr int COL2_LABEL_X = 700, COL2_VALUE_X = 1020, COL2_VALUE_MAX_W = 234;
constexpr int ROW0_Y = 120, ROW_H = 80;

Aircraft g_ac;
bool g_hasAc = false;

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

}  // namespace

void detailScreenSet(const Aircraft &ac) {
    g_ac = ac;
    g_hasAc = true;
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
    String title = g_ac.callsign;
    if (g_ac.reg.length()) {
        title += "  (" + g_ac.reg + ")";
    }
    canvas.drawString(title, 16, 16);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    // Livery banner: no real logo/image assets available, so this is the
    // airline's actual brand colour as a drawn band, not a bitmap.
    AirlineInfo airline;
    if (lookupAirline(g_ac.callsign, airline)) {
        uint16_t bandColor = M5.Display.color565((airline.color >> 16) & 0xFF, (airline.color >> 8) & 0xFF,
                                                   airline.color & 0xFF);
        canvas.fillRect(16, 48, 1248, 32, bandColor);
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(3);
        canvas.setTextColor(colorWhite);
        canvas.setTextDatum(ML_DATUM);
        canvas.drawString(airline.name, 26, 64);
    }

    canvas.drawFastHLine(16, 92, 1248, colorGrey);

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
    String category = g_ac.category.length() ? g_ac.category : "unknown";
    String squawk = g_ac.squawk.length() ? g_ac.squawk : "unknown";
    String reg = g_ac.reg.length() ? g_ac.reg : "unknown";

    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 0 * ROW_H, "CALLSIGN", g_ac.callsign, colorWhite);
    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 1 * ROW_H, "REGISTRATION", reg, colorWhite);
    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 2 * ROW_H, "TYPE", type, colorWhite);
    field(COL1_LABEL_X, COL1_VALUE_X, COL1_VALUE_MAX_W, ROW0_Y + 3 * ROW_H, "CATEGORY", category, colorWhite);
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
    if (g_ac.hasPos) {
        footer += "    Position " + String(g_ac.lat, 4) + ", " + String(g_ac.lon, 4);
    }
    canvas.drawString(footer, 16, ROW0_Y + 6 * ROW_H);

    screen::flush();
}

bool detailScreenHandleTouch(int x, int y) {
    return x >= BACK_X && x < BACK_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H;
}
