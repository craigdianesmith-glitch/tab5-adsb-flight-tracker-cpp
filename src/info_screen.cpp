#include "info_screen.h"

#include <M5Unified.h>

#include <vector>

#include "airfield_pack.h"
#include "config.h"
#include "ground_pack.h"
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
    "for an emergency. A cargo flight's callsign - FedEx, UPS, DHL - is sky blue.",
    "- Tap a row for that aircraft's details. There, FOLLOW follows it on the radar, WATCH puts its callsign on "
    "your watchlist, and SAY reads them out. Tap an airport of its route to open that airport's zoom.",
    "- Hold a row to put its callsign on the watchlist, or take it off. With more aircraft than rows, drag the "
    "table up or down for the rest. Tap a heading to sort by it, again to reverse.",
    "- The icons at the top right: the filter, Replays, the radar, the sound off or on, and Settings.",
    "- The filter (the funnel, here and on the radar) shows only the aircraft in a band of height and of speed, civil, cargo or military, and of the airlines "
    "and types typed in (EZY RYR, A32 B738), leaving out the types under Hide types (GLID, the gliders) - "
    "on the table and the radar, and in a replay as it was recorded. The "
    "funnel is lit while it is on. The aircraft followed, the zoom and alerts are never filtered, and a recording "
    "keeps every aircraft.",

    "# The radar",
    "Everything in range, around the centre: each aircraft with its callsign, a line to where it will be in a "
    "minute, and ^ or v while it climbs or descends. Under it is the map: coasts and lakes, with borders dashed "
    "and state lines dotted, and faintly the names of the countries, regions, seas and lakes.",
    "- HOME centres on your location, AIRPORT on the airport nearest it, and ALL is AIRPORT with every other "
    "airport around marked too.",
    "- ZOOM IN and ZOOM OUT change the range. ZOOM IN from the closest opens the airport zoom; ZOOM OUT past your "
    "range widens it, and past the widest goes on over the map alone, to 2000nm. Drag empty sky to look around - "
    "in the zoom too. HOME or AIRPORT comes back; ALL marks every airport where you have got to.",
    "- Tap an aircraft for its details, an airport to zoom in on it, or a navaid - the small hexagons and boxes - "
    "for its frequency. Where a tap reaches several, a list asks which.",
    "- Back goes to the table. The radar keeps its view for next time.",

    "# The airport zoom",
    "One airport close up, its runways drawn to scale. A runway in use - something lined up on it, landing or "
    "taking off - has an arrow on it and is listed at the bottom left, and each take-off and landing is said as a "
    "controller would clear it.",
    "- ZOOM IN shows the aircraft on the taxiways and stands; ZOOM OUT from the widest, the radar.",
    "- INFO puts up the airport's card: its name, and its tower, approach, radar and other frequencies.",
    "- Back goes to the table, and the radar opened from there comes back to this zoom.",

    "# Following an aircraft",
    "Tap FOLLOW, then an aircraft. The radar centres on it and moves with it, closes in as it comes down to an "
    "airport, and hands it to that airport's zoom to watch it land - or, taking off, back to the radar as it climbs "
    "away. ZOOM IN and ZOOM OUT change the range around it until the next landing or take-off; with Auto zoom on, "
    "in Settings, it widens while the sky around it is quiet and closes in as it fills, and in the airport's zoom "
    "follows it in to the runway. Its height, "
    "climb and speed are charted on the left. FOLLOW again, or HOME, AIRPORT or ALL, stops it.",

    "# The 3D view",
    "Following, 3D beside FOLLOW shows the aircraft in 3D, in its airline's colours: at an airport, on its real "
    "runways, taxiways and stands among its buildings; high up, over the coasts, towns and rivers - with the "
    "traffic about it. Its take-offs and landings are simulated, as on the zoom, and each one at the airport is said "
    "as a controller would clear it.",
    "- CINEMA cuts between shots as a director would: the approach from beside the runway, the touchdown, the "
    "roll-out; a helicopter from a camera on the ground where it sets down. CHASE is behind it, ORBIT goes round "
    "it - drag the picture to turn it - and TOWER watches from the airfield's tower.",
    "- FLY PAST sets a camera down ahead of it, beside its way, for it to go by - then ahead again for the next "
    "pass, the other side. ZOOM IN and ZOOM OUT bring the camera nearer or further off.",
    "- GROUND: the land at its height, its hills and mountains, from the ground pack - press it to see them. Off, "
    "as it starts, the ground is flat, its hills along the skyline, and the pictures come twice as quick. Greyed "
    "with no pack, or away from the land it covers.",
    "- SOUND: its engines, as they sound from the camera - loud close by, muffled far off, the pitch dropping as it "
    "passes one standing still. For now an A320's, A318 to A321neo, and a Eurofighter Typhoon's; other aircraft are "
    "silent. The speaker's mute silences them too.",
    "- Tap another aircraft for its details. Back goes to the radar.",
    "- An airport's taxiways, stands, buildings and tower come from the airfield pack, airfields.bin, in the SD "
    "card's overhead folder - as does the tower on the zoom. Without it, or at an airport it hasn't got, the view "
    "is a basic one, the runways alone, and says so under the picture.",
    "- The land's fields, woods, moor and towns, and its hills, come from the ground pack, ground.bin, in the same "
    "folder. The one on each release covers the UK and Ireland; for anywhere else, make one on a computer with "
    "tools/make_ground_pack.py - the manual says how. Without it, or away from the land it covers, the land is flat "
    "and one colour, and the line under the picture says why.",
    "- Settings > About shows which packs the card has, and whether each can be used.",

    "# Alerts",
    "Settings > Alerts picks what raises one: emergencies (squawk 7700, 7600 or 7500), military aircraft, rare "
    "types, and your watchlist. An alert puts a banner across the top - tap it for the aircraft, or X to close it - "
    "chimes, and says what it is and its callsign. While you are following an aircraft it only chimes, unless it "
    "is an emergency. Auto-record records while one is about; auto-follow follows the aircraft it is for.",
    "Words are said in the voice of where they are about: Scottish for Scotland, English for England, American for "
    "the United States, and the one picked under Voice for everywhere else. Read aloud says an aircraft's details "
    "as they open, a navaid's card, and one of each of an airport's frequencies.",
    "The watchlist takes callsigns (EZY12AB), airline prefixes (EZY), registrations (G-ABCD), type codes (A388) "
    "and ICAO hex codes, separated by spaces.",

    "# Recording, replays and sharing",
    "With an SD card in, REC on the radar or the zoom records what is on show until it is tapped again. Replays "
    "lists the recordings: play one at 1x, 4x, 16x or 64x, step a minute either way or tap along its bar, and "
    "Export it as a video. Share, on the Replays screen, puts the videos and recordings on your WiFi for a phone or "
    "computer to download - open the address it shows in a browser.",

    "# Settings",
    "How far to look, how often to refresh, your location, the WiFi, the data source, and the radar's navaids, "
    "map, rings and auto zoom for following. Civil, cargo or military is the filter's. They take effect as you "
    "leave with Back. Display has the "
    "brightness, dimming when untouched, the radar's clock - UTC, or the local time of the UK, central Europe or "
    "the US - and whether replays draw each aircraft's track. The speaker at the end "
    "of the radar's title "
    "mutes the sound, as the table's does.",
};

const char *const ABOUT[] = {
    "Overhead v" FIRMWARE_VERSION " - a live flight tracker for the M5Stack Tab5, by Craig Smith. Its source is at "
    "github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-cpp: free software under the MIT licence, but for "
    "its sound and terrain engines, free for noncommercial use under PolyForm Noncommercial 1.0.0.",

    "# Disclaimer",
    "For interest and entertainment only. Not for navigation, air traffic control, flight planning, or any "
    "purpose where safety depends on it.",
    "The aircraft come from volunteer-run networks of receivers on the ground. What they report can be "
    "incomplete, late or wrong: an aircraft can be missing, or shown where it isn't. Positions near the ground are "
    "partly estimated, take-offs and landings are simulated, and the spoken clearances are made up from them - "
    "they are not real air traffic control.",
    "Provided as is, without warranty of any kind - see the licences. Please respect other people's privacy, and "
    "the law where you use it.",

    "# Credits",
    "- Flight routes: adsbdb.com, free and crowd-sourced.",
    "- Live aircraft: adsb.lol, whose data is made available under the Open Database License 1.0, and adsb.fi "
    "(adsb.fi) - both run by volunteers, and free to use. Please keep the refresh interval generous.",
    "- Airports, their runways and frequencies, and navaids: OurAirports (ourairports.com). The map's coasts, "
    "lakes, borders and place names, and the 3D view's towns and rivers: Natural Earth (naturalearthdata.com). "
    "The height of the sea over the GPS ellipsoid: the NGA's EGM96 geoid model. All public domain.",
    "- Place search: Open-Meteo's geocoding API (open-meteo.com), CC BY 4.0.",
    "- Time zones, for the clock while following: timezone-boundary-builder, from OpenStreetMap data, "
    "(c) OpenStreetMap contributors, under the Open Database License 1.0, as is the table made from it; and the "
    "IANA time zone database, public domain.",
    "- Airfields in the 3D view - taxiways, aprons, stands, buildings and towers, from the card's airfields.bin: "
    "(c) OpenStreetMap contributors (openstreetmap.org/copyright), under the Open Database License 1.0, as is "
    "the file made from them by tools/gen_airfields.py.",
    "- The land in the 3D view - what covers it, and its hills - from the card's ground.bin: ESA WorldCover 2021, "
    "(c) ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA "
    "WorldCover consortium, CC BY 4.0; and the Copernicus DEM GLO-30 and GLO-90, (c) DLR e.V. 2010-2014 and (c) Airbus "
    "Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights "
    "reserved.",
    "- Voices: Piper text-to-speech. Scottish, en_GB-alba-medium, from the Alba data of the CSTR, University of "
    "Edinburgh, CC BY 4.0; English, en_GB-cori-medium, from LibriVox recordings, public domain; American, "
    "en_US-amy-medium, from Mycroft AI's Mimic 3 voices, CC BY-SA 4.0.",
    "- Built on M5Unified, M5GFX and ArduinoJson (MIT), the ESP32 Arduino core (LGPL 2.1) and ESP-IDF "
    "(Apache 2.0).",
    "- Airline names identify the operator, and imply no endorsement by it.",
};

// --- what the card has -------------------------------------------------------
// For About, under its first paragraph: the airfield and ground packs on the
// SD card, or why the 3D view goes without each - looked at afresh each time
// it opens, a card having gone in since, or the 3D view having read the pack.

// A count with its thousands marked: 2,068.
String grouped(uint32_t n) {
    String digits(n), out;
    int len = digits.length();
    for (int i = 0; i < len; i++) {
        if (i > 0 && (len - i) % 3 == 0) {
            out += ',';
        }
        out += digits[i];
    }
    return out;
}

String megabytes(uint32_t bytes) { return String(bytes / 1048576.0f, 1) + "MB"; }

// An edge of a pack's grid, `tiles` fifths of a degree from the equator or
// from Greenwich: "49.8N", "10.8W".
String edge(int tiles, char plus, char minus) {
    return String(abs(tiles) / (float)groundpack::TILES_PER_DEG, 1) + (tiles < 0 ? minus : plus);
}

String airfieldPackLine() {
    uint16_t airfields = 0;
    uint32_t bytes = 0;
    airfield::Missing why;
    if (airfield::packInfo(airfields, bytes, why)) {
        return "- Airfield pack: airfields.bin, " + grouped(airfields) + " airfields (" + megabytes(bytes) + ").";
    }
    String said = why == airfield::Missing::NO_CARD   ? "no SD card in"
                  : why == airfield::Missing::NO_PACK ? "no airfields.bin in the card's overhead folder"
                                                      : "the card's airfields.bin is for another version";
    return "- Airfield pack: " + said + ", so the 3D view's airports are the basic view, their runways alone.";
}

String groundPackLine() {
    using S = groundpack::Status;
    groundpack::PackCheck c;
    switch (groundpack::packInfo(c)) {
    case S::READY:
        return "- Ground pack: ground.bin, " + grouped(c.count) + " tiles (" + megabytes(c.fileSize) + "), between " +
               edge(c.south, 'N', 'S') + " and " + edge(c.south + c.rows, 'N', 'S') + ", " + edge(c.west, 'E', 'W') +
               " and " + edge(c.west + c.cols, 'E', 'W') + ".";
    case S::LOADING:
        return "- Ground pack: ground.bin, being read in for the 3D view.";
    case S::NO_CARD:
        return "- Ground pack: no SD card in, so the 3D view's land is flat and one colour.";
    case S::NO_PACK:
        return "- Ground pack: no ground.bin in the card's overhead folder, so the 3D view's land is flat and one "
               "colour.";
    case S::OTHER_VERSION:
        return "- Ground pack: the card's ground.bin is for another version, so the 3D view's land is flat. Put the "
               "one from this version's release on the card, or make it again.";
    case S::TOO_BIG:
        return "- Ground pack: the card's ground.bin is too big for the Tab5 - " + String(c.why) +
               " - so the 3D view's land is flat. Make a smaller one: the manual says how.";
    case S::DAMAGED:
        return "- Ground pack: the card's ground.bin is damaged, so the 3D view's land is flat. Copy it to the card "
               "again.";
    default:
        return "- Ground pack: there wasn't the memory free for the card's ground.bin, so the 3D view's land is flat. "
               "Restart the Tab5.";
    }
}

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
        // Its first paragraph, then what the card has, then the rest.
        const String card[] = {
            "# On the SD card", airfieldPackLine(), groundPackLine(),
            "The ground pack on each release covers the UK and Ireland. One for anywhere else can be made on a "
            "computer with tools/make_ground_pack.py - the manual says how."};
        std::vector<const char *> blocks = {ABOUT[0]};
        for (const String &block : card) {
            blocks.push_back(block.c_str());
        }
        blocks.insert(blocks.end(), ABOUT + 1, ABOUT + sizeof(ABOUT) / sizeof(ABOUT[0]));
        layOut(blocks.data(), blocks.size());
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
