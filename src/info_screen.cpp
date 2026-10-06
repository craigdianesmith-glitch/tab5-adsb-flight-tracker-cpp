#include "info_screen.h"

#include <M5Unified.h>

#include <vector>

#include "config.h"
#include "screen.h"

namespace {

// --- what they say -----------------------------------------------------------
// A block a line: "# " starts a heading, "- " a bullet, anything else is a
// paragraph. Wrapped and split into pages to fit when the screen opens, so
// the words can change without anything here being counted.

const char *const HELP[] = {
    "# The table",
    "The aircraft around your location, nearest first: callsign, type, altitude, speed, distance, heading and "
    "what each is doing. A new one is green for a moment; one an alert is for stays amber while it is about, or red "
    "for an emergency.",
    "- Tap a row for that aircraft's details. There, FOLLOW follows it on the radar and WATCH puts its callsign on "
    "your watchlist.",
    "- Hold a row to put its callsign on the watchlist, or take it off.",
    "- The icons at the top right open the radar, switch the sound off or on, and open Settings.",

    "# The radar",
    "Everything in range, around the centre: each aircraft with its callsign, a line to where it will be in a "
    "minute, and ^ or v while it climbs or descends.",
    "- HOME centres on your location, AIRPORT on the airport nearest it, and ALL is AIRPORT with every other "
    "airport around marked too.",
    "- ZOOM IN and ZOOM OUT change the range. ZOOM IN from the closest opens the airport zoom; ZOOM OUT past your "
    "range widens it.",
    "- Tap an aircraft for its details, an airport to zoom in on it, or a navaid - the small hexagons and boxes - "
    "for its frequency. Where a tap reaches several, a list asks which.",
    "- Back goes to the table. The radar keeps its view for next time.",

    "# The airport zoom",
    "One airport close up, its runways drawn to scale. A runway in use - something lined up on it, landing or "
    "taking off - has an arrow on it and is listed at the bottom left, and each take-off and landing is said as a "
    "controller would clear it.",
    "- ZOOM IN to see the aircraft on the taxiways and at the stands. ZOOM OUT from the widest goes to the radar.",
    "- Back goes to the table, and the radar opened from there comes back to this zoom.",

    "# Following an aircraft",
    "Tap FOLLOW, then an aircraft. The radar centres on it and moves with it, closes in as it comes down to an "
    "airport, and hands it to that airport's zoom to watch it land - or, taking off, back to the radar as it climbs "
    "away. Its height, climb and speed are charted on the left. FOLLOW again, or HOME, AIRPORT or ALL, stops it.",

    "# Alerts",
    "Settings > Alerts picks what raises one: emergencies (squawk 7700, 7600 or 7500), military aircraft, rare "
    "types, and your watchlist. An alert puts a banner across the top - tap it for the aircraft, or X to close it - "
    "chimes, and says what it is and its callsign. Auto-record records while one is about; auto-follow follows the "
    "aircraft it is for.",
    "The watchlist takes callsigns (EZY12AB), airline prefixes (EZY), registrations (G-ABCD), type codes (A388) "
    "and ICAO hex codes, separated by spaces.",

    "# Recording, replays and sharing",
    "With an SD card in, REC on the radar or the zoom records what is on show until it is tapped again. Replays "
    "lists the recordings: play one at 1x, 4x, 16x or 64x, step a minute either way or tap along its bar, and "
    "Export it as a video. Share, on the Replays screen, puts the videos and recordings on your WiFi for a phone or "
    "computer to download - open the address it shows in a browser.",

    "# Settings",
    "Civilian or military traffic, how far around you to look, how often to refresh, your location, the WiFi, the "
    "data source, and navaids. They take effect as you leave with Back.",
};

const char *const ABOUT[] = {
    "Overhead v" FIRMWARE_VERSION " - a live flight tracker for the M5Stack Tab5, by Craig Smith. Free software "
    "under the MIT licence: github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-cpp",

    "# Disclaimer",
    "For interest and entertainment only. Not for navigation, air traffic control, flight planning, or any "
    "purpose where safety depends on it.",
    "The aircraft come from volunteer-run networks of receivers on the ground. What they report can be "
    "incomplete, late or wrong: an aircraft can be missing, or shown where it isn't. Positions near the ground are "
    "partly estimated, take-offs and landings are simulated, and the spoken clearances are made up from them - "
    "they are not real air traffic control.",
    "Provided as is, without warranty of any kind - see the licence. Please respect other people's privacy, and "
    "the law where you use it.",

    "# Credits",
    "- Live aircraft: adsb.lol, whose data is made available under the Open Database License 1.0, and adsb.fi "
    "(adsb.fi) - both run by volunteers, and free to use. Please keep the refresh interval generous.",
    "- Airports, runways and navaids: OurAirports (ourairports.com), public domain.",
    "- Place search: Open-Meteo's geocoding API (open-meteo.com), CC BY 4.0.",
    "- Voice: Piper text-to-speech, its en_GB-alba-medium voice, built from the Alba data of the CSTR, University "
    "of Edinburgh, CC BY 4.0.",
    "- Built on M5Unified, M5GFX and ArduinoJson (MIT), the ESP32 Arduino core (LGPL 2.1) and ESP-IDF "
    "(Apache 2.0).",
    "- Airline names identify the operator, and imply no endorsement by it.",
};

// --- how they are laid out -----------------------------------------------------

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int NEXT_X = BACK_X - 12 - BACK_W, PREV_X = NEXT_X - 12 - BACK_W;

constexpr int BODY_X = 40, BODY_W = 1200, BODY_TOP = 76, BODY_BOTTOM = 708;
constexpr int CHAR_W = 12;  // Font0 at size 2: fixed width, so wrapping is a count
constexpr int BULLET_INDENT = 28;
constexpr int LINE_H = 22, PARA_GAP = 10, HEADING_GAP = 14;

enum class Kind : uint8_t { HEADING, TEXT, BULLET, BULLET_MORE, GAP };
struct Line {
    Kind kind;
    String text;
};

InfoPage g_page = InfoPage::HELP;
std::vector<Line> g_lines;
std::vector<size_t> g_pageStarts;  // index into g_lines of each page's first line
size_t g_pageIdx = 0;

uint16_t colorBg, colorWhite, colorDim, colorBtnBg, colorHeading, colorDisabled;
bool colorsReady = false;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorDim = M5.Display.color565(0x88, 0x91, 0x9B);
    colorBtnBg = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorHeading = M5.Display.color565(0x6F, 0xCF, 0x97);
    colorDisabled = M5.Display.color565(0x44, 0x4C, 0x55);
    colorsReady = true;
}

int heightOf(Kind k) { return k == Kind::GAP ? PARA_GAP : k == Kind::HEADING ? LINE_H + HEADING_GAP : LINE_H; }

// `text` broken at spaces into lines of at most `chars` characters - or
// mid-word, for a word longer than a whole line.
std::vector<String> wrap(const String &text, int chars) {
    std::vector<String> out;
    int start = 0, len = text.length();
    while (start < len) {
        if (len - start <= chars) {
            out.push_back(text.substring(start));
            break;
        }
        int cut = start + chars;
        int space = text.lastIndexOf(' ', cut);
        if (space <= start) {
            out.push_back(text.substring(start, cut));
            start = cut;
        } else {
            out.push_back(text.substring(start, space));
            start = space + 1;
        }
    }
    return out;
}

void layOut(const char *const *blocks, size_t count) {
    // Each paragraph or bullet is a unit the pages break between, never
    // through - a heading together with the one after it.
    struct Unit {
        size_t first, end;
    };
    std::vector<Unit> units;
    g_lines.clear();
    bool afterHeading = false;
    for (size_t i = 0; i < count; i++) {
        String b(blocks[i]);
        size_t first = g_lines.size();
        if (b.startsWith("# ")) {
            g_lines.push_back({Kind::HEADING, b.substring(2)});
            units.push_back({first, g_lines.size()});
            afterHeading = true;
            continue;
        }
        bool bullet = b.startsWith("- ");
        int chars = (BODY_W - (bullet ? BULLET_INDENT : 0)) / CHAR_W;
        std::vector<String> wrapped = wrap(bullet ? b.substring(2) : b, chars);
        for (size_t j = 0; j < wrapped.size(); j++) {
            g_lines.push_back({bullet ? (j == 0 ? Kind::BULLET : Kind::BULLET_MORE) : Kind::TEXT, wrapped[j]});
        }
        g_lines.push_back({Kind::GAP, String()});
        if (afterHeading) {
            units.back().end = g_lines.size();
        } else {
            units.push_back({first, g_lines.size()});
        }
        afterHeading = false;
    }

    // Into pages: as many units as fit - the gap after the last needn't, and
    // a heading at the top of a page goes without the space above it.
    g_pageStarts.clear();
    int y = BODY_TOP;
    for (const Unit &u : units) {
        int h = 0;
        for (size_t i = u.first; i < u.end; i++) {
            h += heightOf(g_lines[i].kind);
        }
        int trailing = g_lines[u.end - 1].kind == Kind::GAP ? PARA_GAP : 0;
        bool top = g_pageStarts.empty() || y == BODY_TOP;
        int lead = top && g_lines[u.first].kind == Kind::HEADING ? HEADING_GAP : 0;
        if (g_pageStarts.empty() || (!top && y + h - lead - trailing > BODY_BOTTOM)) {
            g_pageStarts.push_back(u.first);
            y = BODY_TOP;
            lead = g_lines[u.first].kind == Kind::HEADING ? HEADING_GAP : 0;
        }
        y += h - lead;
    }
    if (g_pageStarts.empty()) {
        g_pageStarts.push_back(0);
    }
}

void button(int x, const char *label, bool enabled) {
    auto &canvas = screen::canvas();
    if (enabled) {
        canvas.fillRoundRect(x, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    } else {
        canvas.drawRoundRect(x, BACK_Y, BACK_W, BACK_H, 6, colorDisabled);
    }
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(enabled ? colorWhite : colorDisabled);
    canvas.drawString(label, x + BACK_W / 2, BACK_Y + BACK_H / 2);
}

}  // namespace

void infoScreenOpen(InfoPage page) {
    g_page = page;
    if (page == InfoPage::HELP) {
        layOut(HELP, sizeof(HELP) / sizeof(HELP[0]));
    } else {
        layOut(ABOUT, sizeof(ABOUT) / sizeof(ABOUT[0]));
    }
    g_pageIdx = 0;
    infoScreenDraw();
}

void infoScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);
    canvas.setFont(&fonts::Font0);

    const char *title = g_page == InfoPage::HELP ? "How to use it" : "About";
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(title, 16, 12);
    size_t pages = g_pageStarts.size();
    if (pages > 1) {
        canvas.setTextSize(2);
        canvas.setTextColor(colorDim);
        canvas.drawString(String(g_pageIdx + 1) + " of " + String(pages), 16 + 18 * strlen(title) + 16, 20);
        button(PREV_X, "< Prev", g_pageIdx > 0);
        button(NEXT_X, "Next >", g_pageIdx + 1 < pages);
    }
    button(BACK_X, "Back", true);

    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    size_t end = g_pageIdx + 1 < pages ? g_pageStarts[g_pageIdx + 1] : g_lines.size();
    int y = BODY_TOP;
    for (size_t i = g_pageStarts[g_pageIdx]; i < end; i++) {
        const Line &l = g_lines[i];
        switch (l.kind) {
        case Kind::GAP:
            if (y > BODY_TOP) {
                y += PARA_GAP;
            }
            continue;
        case Kind::HEADING:
            if (y > BODY_TOP) {
                y += HEADING_GAP;
            }
            canvas.setTextColor(colorHeading);
            canvas.drawString(l.text, BODY_X, y);
            break;
        case Kind::BULLET:
            canvas.setTextColor(colorWhite);
            canvas.fillCircle(BODY_X + 8, y + 7, 3, colorDim);
            canvas.drawString(l.text, BODY_X + BULLET_INDENT, y);
            break;
        case Kind::BULLET_MORE:
            canvas.setTextColor(colorWhite);
            canvas.drawString(l.text, BODY_X + BULLET_INDENT, y);
            break;
        case Kind::TEXT:
            canvas.setTextColor(colorWhite);
            canvas.drawString(l.text, BODY_X, y);
            break;
        }
        y += LINE_H;
    }
    screen::flush();
}

InfoAction infoScreenHandleTouch(int x, int y) {
    if (y < BACK_Y || y >= BACK_Y + BACK_H) {
        return InfoAction::NONE;
    }
    if (x >= BACK_X && x < BACK_X + BACK_W) {
        return InfoAction::BACK;
    }
    size_t pages = g_pageStarts.size();
    if (pages > 1 && x >= NEXT_X && x < NEXT_X + BACK_W && g_pageIdx + 1 < pages) {
        g_pageIdx++;
        infoScreenDraw();
    } else if (pages > 1 && x >= PREV_X && x < PREV_X + BACK_W && g_pageIdx > 0) {
        g_pageIdx--;
        infoScreenDraw();
    }
    return InfoAction::NONE;
}
