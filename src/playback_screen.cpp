#include "playback_screen.h"

#include <M5Unified.h>
#include <SD.h>
#include <algorithm>
#include <time.h>
#include <vector>

#include "follow.h"
#include "telemetry.h"
#include "radar_screen.h"
#include "replay.h"
#include "runways.h"
#include "screen.h"
#include "video_writer.h"

namespace {

// Back and the centre toggle where the live radar has them, and Export under
// Back: the plot now runs the full height, and a third button in the row
// reached into it.
constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int CENTRE_W = 124;
constexpr int CENTRE_X = BACK_X - 12 - CENTRE_W;
constexpr int EXPORT_W = 124;
constexpr int EXPORT_X = BACK_X, EXPORT_Y = BACK_Y + BACK_H + 8;

// The controls take the margin left of the plot, where the live radar puts a
// followed aircraft's telemetry: the plot itself is drawn exactly as it is
// live. A replay of one has its telemetry on the right instead, under the
// buttons.
constexpr int PANEL_X = 16, PANEL_W = 274;
constexpr int CLOCK_LABEL_Y = 76, CLOCK_Y = 100;
constexpr int ELAPSED_Y = 146;
constexpr int NOTE_Y = 176;
constexpr int PLAY_Y = 214, PLAY_H = 72;
constexpr int STEP_Y = 298, STEP_H = 60, STEP_GAP = 8;
constexpr int STEP_W = (PANEL_W - 2 * STEP_GAP) / 3;
constexpr int BAR_Y = 378, BAR_H = 40;
constexpr int BAR_TOUCH_PAD = 14;  // a finger is wider than the bar is tall
constexpr int FRAME_Y = 436;
constexpr int TEL_X = 1264 - TELEMETRY_W, TEL_Y = EXPORT_Y + BACK_H + 12;
// What a tick repaints: the clock and elapsed lines, and the bar with its
// caption. The buttons only change when they are pressed.
constexpr int DYN1_Y = CLOCK_Y - 4, DYN1_H = NOTE_Y - DYN1_Y - 6;
constexpr int DYN2_Y = BAR_Y - 4, DYN2_H = FRAME_Y + 22 - DYN2_Y;

constexpr uint32_t SKIP_MS = 60000;
constexpr int SPEEDS[] = {1, 4, 16, 64};
constexpr int SPEED_COUNT = sizeof(SPEEDS) / sizeof(SPEEDS[0]);
// How often a playing recording is redrawn. At 1x the blips creep, and
// redrawing the plot four times as often would show nothing new; faster,
// they move enough between frames to want the smoothness.
constexpr uint32_t FRAME_MS_SLOW = 400, FRAME_MS_FAST = 100;

// --- video export ---
// Ten frames a second: the blips glide between polls, but slowly, so more
// would mostly lengthen the export - every frame is drawn and encoded in turn.
constexpr int VIDEO_FPS = 10;
// How often the frame just encoded is shown, with the progress over it.
constexpr uint32_t PROGRESS_MS = 500;
// The progress takes the place of the Play and step buttons, which have no
// use while it runs - and which the video doesn't show.
constexpr int CTRL_Y = PLAY_Y, CTRL_H = STEP_Y + STEP_H - PLAY_Y;
// How the last export went, shown under the panel until the next.
constexpr int RESULT_Y = FRAME_Y + 40;
String g_resultLine, g_resultDetail;
String g_path;  // the recording open, for naming its video
// Whether the recording has already been exported at the speed chosen now.
// Export is greyed out while it has: doing it again would only make the same
// file over. A different speed is a different video, so changing the speed
// can bring it back.
bool g_videoExists = false;

uint32_t g_t = 0;  // ms into the recording
bool g_playing = false;
int g_speedIdx = 1;
uint32_t g_lastTickMs = 0;
uint32_t g_lastDrawMs = 0;
bool g_dirty = false;
RadarCentre g_centre = DEFAULT_RADAR_CENTRE;
bool g_airports = false;

std::vector<Aircraft> g_scene;
std::vector<replay::TrailPoint> g_trails;
const std::vector<uint8_t> g_noneNew;  // nothing is "new" in a replay
// For a recording that follows an aircraft, the view last drawn: the airport
// zoom's code, or "" for the radar centred on the aircraft. A change between
// them is a full redraw, since their footers and the note above differ.
replay::View g_view;  // the view last drawn
int g_followRange = FOLLOW_RANGE_NM;  // the radar view's, as last worked out
// How long the followed aircraft had been lost at g_t, as the live screen
// counted it, or 0: set by keepFollowedInScene() for the telemetry.
uint32_t g_lostS = 0;

// Whether the recording has polls of the radar around home, for the HOME and
// AIRPORT centre to apply to.
bool hasHome() { return replay::hasHomeView(); }

String clockText(uint32_t ms) {
    uint32_t s = ms / 1000;
    char buf[16];
    if (s >= 3600) {
        snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60),
                 (unsigned long)(s % 60));
    } else {
        snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
    }
    return String(buf);
}

void button(int x, int y, int w, int h, const String &label, uint16_t bg, int size) {
    auto &canvas = screen::canvas();
    const RadarPalette &p = radarPalette();
    canvas.fillRect(x, y, w, h, p.bg);
    canvas.fillRoundRect(x, y, w, h, 6, bg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(size);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(p.text);
    canvas.drawString(label, x + w / 2, y + h / 2);
    screen::markDirty(x, y, w, h);
}

void drawPlayButton() {
    const RadarPalette &p = radarPalette();
    bool atEnd = g_t >= replay::durationMs();
    button(PANEL_X, PLAY_Y, PANEL_W, PLAY_H, g_playing ? "Pause" : (atEnd ? "Replay" : "Play"),
           g_playing ? p.btnBg : p.ring, 3);
}

void drawStepButtons() {
    const RadarPalette &p = radarPalette();
    button(PANEL_X, STEP_Y, STEP_W, STEP_H, "-1m", p.btnBg, 2);
    button(PANEL_X + STEP_W + STEP_GAP, STEP_Y, STEP_W, STEP_H, String(SPEEDS[g_speedIdx]) + "x", p.btnBg, 2);
    button(PANEL_X + 2 * (STEP_W + STEP_GAP), STEP_Y, STEP_W, STEP_H, "+1m", p.btnBg, 2);
}

void drawDynamic() {
    auto &canvas = screen::canvas();
    const RadarPalette &p = radarPalette();
    uint32_t dur = replay::durationMs();

    canvas.fillRect(PANEL_X, DYN1_Y, PANEL_W, DYN1_H, p.bg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(TL_DATUM);
    uint32_t epoch = replay::epochAt(g_t);
    if (epoch) {
        time_t t = epoch;
        struct tm tm;
        gmtime_r(&t, &tm);
        char buf[16];
        strftime(buf, sizeof(buf), "%H:%M:%S", &tm);
        canvas.setTextSize(4);
        canvas.setTextColor(p.text);
        canvas.drawString(buf, PANEL_X, CLOCK_Y);
        canvas.setTextSize(2);
        canvas.setTextColor(p.muted);
        canvas.drawString("UTC", PANEL_X + canvas.textWidth("00:00:00") * 2 + 10, CLOCK_Y + 14);
    } else {
        // No wall clock was recorded, so the time into the recording takes
        // its place.
        canvas.setTextSize(4);
        canvas.setTextColor(p.text);
        canvas.drawString(clockText(g_t), PANEL_X, CLOCK_Y);
    }
    canvas.setTextSize(2);
    canvas.setTextColor(p.muted);
    canvas.drawString(clockText(g_t) + " / " + clockText(dur), PANEL_X, ELAPSED_Y);
    screen::markDirty(PANEL_X, DYN1_Y, PANEL_W, DYN1_H);

    canvas.fillRect(PANEL_X, DYN2_Y, PANEL_W, DYN2_H, p.bg);
    canvas.fillRoundRect(PANEL_X, BAR_Y, PANEL_W, BAR_H, 6, p.btnBg);
    int filled = dur ? (int)((uint64_t)PANEL_W * std::min(g_t, dur) / dur) : 0;
    if (filled > 0) {
        canvas.fillRoundRect(PANEL_X, BAR_Y, std::max(filled, 12), BAR_H, 6, p.ring);
    }
    canvas.fillRect(PANEL_X + std::min(filled, PANEL_W - 4), BAR_Y - 4, 4, BAR_H + 8, p.text);
    canvas.setTextSize(2);
    canvas.setTextColor(p.faint);
    canvas.setTextDatum(TL_DATUM);
    size_t frames = replay::frameCount();
    canvas.drawString("Poll " + String(frames ? replay::frameIndexAt(g_t) + 1 : 0) + " of " + String(frames),
                      PANEL_X, FRAME_Y);
    screen::markDirty(PANEL_X, DYN2_Y, PANEL_W, DYN2_H);
}

// The video the recording open makes at the speed chosen now:
// 20261002-143155.rec at 4x is /videos/20261002-143155-4x.mp4.
String videoPath() {
    String name = g_path.substring(g_path.lastIndexOf('/') + 1);
    if (name.endsWith(".rec")) {
        name.remove(name.length() - 4);
    }
    return String(VIDEO_DIR) + "/" + name + "-" + String(SPEEDS[g_speedIdx]) + "x.mp4";
}

void checkVideo() {
    recorder::CardLock lock;
    g_videoExists = recorder::mounted() && SD.exists(videoPath());
}

void drawExportButton() {
    auto &canvas = screen::canvas();
    const RadarPalette &p = radarPalette();
    canvas.fillRect(EXPORT_X, EXPORT_Y, EXPORT_W, BACK_H, p.bg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    if (g_videoExists) {
        // Outlined and grey, as the radar draws a button with nothing behind it.
        canvas.drawRoundRect(EXPORT_X, EXPORT_Y, EXPORT_W, BACK_H, 6, p.disabled);
        canvas.setTextColor(p.disabled);
        canvas.drawString("Exported", EXPORT_X + EXPORT_W / 2, EXPORT_Y + BACK_H / 2);
    } else {
        canvas.fillRoundRect(EXPORT_X, EXPORT_Y, EXPORT_W, BACK_H, 6, p.btnBg);
        canvas.setTextColor(p.text);
        canvas.drawString("Export", EXPORT_X + EXPORT_W / 2, EXPORT_Y + BACK_H / 2);
    }
    screen::markDirty(EXPORT_X, EXPORT_Y, EXPORT_W, BACK_H);
}

// For a video frame, the buttons give way to the speed it plays at: nobody
// can press them, and a 16x video should say it is one.
void drawHeader(bool forVideo) {
    auto &canvas = screen::canvas();
    const RadarPalette &p = radarPalette();
    const recorder::Header &h = replay::header();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(p.text);
    // In the corner left of the plot, as the live radar's title is: the
    // date on a line of its own under it.
    const char *title = "Replay";
    canvas.drawString(title, 16, 24);
    canvas.setTextColor(p.muted);
    canvas.drawString(h.military ? " - MIL -" : " - CIV -", 16 + canvas.textWidth(title), 24);
    if (h.startEpoch) {
        time_t t = h.startEpoch;
        struct tm tm;
        gmtime_r(&t, &tm);
        char buf[32];
        strftime(buf, sizeof(buf), "%d %b %Y %H:%M UTC", &tm);
        canvas.setTextSize(2);
        canvas.drawString(buf, 16, 56);
        canvas.setTextSize(3);
    }

    if (forVideo) {
        canvas.setTextDatum(MR_DATUM);
        canvas.drawString(String(SPEEDS[g_speedIdx]) + "x speed", 1264, 24);
        return;
    }
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(p.text);
    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, p.btnBg);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    // A followed aircraft's polls were fetched around it, not home, so there
    // is no other centre to switch to.
    if (hasHome()) {
        canvas.fillRoundRect(CENTRE_X, BACK_Y, CENTRE_W, BACK_H, 6, p.btnBg);
        canvas.drawString(g_centre == RadarCentre::HOME ? "HOME" : "AIRPORT", CENTRE_X + CENTRE_W / 2,
                          BACK_Y + BACK_H / 2);
    }
    drawExportButton();
}

void drawStaticPanel(bool forVideo) {
    auto &canvas = screen::canvas();
    const RadarPalette &p = radarPalette();
    const recorder::Header &h = replay::header();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(p.faint);
    canvas.drawString(replay::epochAt(0) ? "TIME" : "ELAPSED", PANEL_X, CLOCK_LABEL_Y);

    String note = h.trigger == recorder::Trigger::AUTO     ? "Auto: " + h.note
                  : h.trigger == recorder::Trigger::FOLLOW ? "Follow: " + h.note
                  : h.note.length()                        ? "Manual: " + h.note
                                                           : String("Manual recording");
    if (h.part > 1) {
        note += " (part " + String(h.part) + ")";
    }
    if (g_view.kind == replay::View::ZOOM) {
        note += String(" at ") + g_view.code;
    } else if (g_view.kind == replay::View::FOLLOW && h.followHex != g_view.hex) {
        note += String(" - following ") + g_view.hex;
    }
    // Allowed the margin between the panel and the plot as well: a typical
    // auto note ("Auto: EZY13MG WATCHLIST") overran the panel by two pixels
    // and fell to size 1, which is hard to read - in a video especially. No
    // further than the plot's edge, which its refresh erases up to.
    constexpr int NOTE_MAX_W = PANEL_W + 2;
    int size = 2;
    canvas.setTextSize(size);
    while (size > 1 && canvas.textWidth(note) > NOTE_MAX_W) {
        canvas.setTextSize(--size);
    }
    canvas.setTextColor(p.muted);
    canvas.drawString(note, PANEL_X, NOTE_Y);

    if (forVideo) {
        return;
    }
    drawPlayButton();
    drawStepButtons();

    if (g_resultLine.length()) {
        canvas.setTextDatum(TL_DATUM);  // the buttons above leave it centred
        canvas.setTextSize(2);
        canvas.setTextColor(p.text);
        canvas.drawString(g_resultLine, PANEL_X, RESULT_Y);
        canvas.setTextSize(1);  // a path is longer than the panel is wide at 2
        canvas.setTextColor(p.muted);
        canvas.drawString(g_resultDetail, PANEL_X, RESULT_Y + 24);
    }
}

String durationText(uint32_t s) {
    if (s < 60) {
        return String(s) + "s";
    }
    if (s < 3600) {
        return String(s / 60) + "m " + String(s % 60) + "s";
    }
    return String(s / 3600) + "h " + String(s / 60 % 60) + "m";
}

void drawExportProgress(uint32_t done, uint32_t total, uint32_t elapsedMs) {
    auto &canvas = screen::canvas();
    const RadarPalette &p = radarPalette();
    canvas.fillRect(PANEL_X, CTRL_Y, PANEL_W, CTRL_H, p.bg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextSize(2);
    canvas.setTextColor(p.text);
    canvas.drawString("Exporting video", PANEL_X, CTRL_Y);

    int pct = (int)((uint64_t)done * 100 / total);
    constexpr int BAR_H2 = 16;
    int barY = CTRL_Y + 26;
    canvas.fillRoundRect(PANEL_X, barY, PANEL_W, BAR_H2, BAR_H2 / 2, p.btnBg);
    int filled = PANEL_W * pct / 100;
    if (filled > 0) {
        canvas.fillRoundRect(PANEL_X, barY, std::max(filled, BAR_H2), BAR_H2, BAR_H2 / 2, p.text);
    }
    // The first couple of seconds are too few frames to say anything by.
    String eta = elapsedMs < 2000 ? String("starting")
                                  : durationText((uint32_t)((uint64_t)elapsedMs * (total - done) / done / 1000)) +
                                        " left";
    canvas.setTextColor(p.muted);
    canvas.drawString(String(pct) + "%   " + eta, PANEL_X, barY + BAR_H2 + 10);
    screen::markDirty(PANEL_X, CTRL_Y, PANEL_W, CTRL_H);
    button(PANEL_X, STEP_Y, PANEL_W, STEP_H, "Cancel", p.rec, 2);
}

// Stops until the finger that pressed Cancel has lifted, and takes the lift
// with it - or it would arrive afterwards as a tap on whatever the replay
// screen puts there.
void waitForRelease() {
    do {
        M5.update();
        delay(10);
    } while (M5.Touch.getCount() && M5.Touch.getDetail(0).isPressed());
    M5.update();
}

// The plot at g_t, from the scene already taken from the recording, drawn
// the way that poll was seen live: the radar around home, centred as chosen
// here; the radar centred on a followed aircraft; or an airport's zoom.
void drawViewPlot(RadarScene &scene, bool full);

void drawPlot(bool full, bool push) {
    const recorder::Header &h = replay::header();
    RadarScene scene{g_scene, g_noneNew, &g_trails, h.lat, h.lon, h.radiusNm, g_centre, g_airports};
    scene.push = push;
    if (g_view.kind == replay::View::HOME) {
        radarPlotDraw(scene, full);
        return;
    }
    drawViewPlot(scene, full);
    if (!g_view.hex[0]) {
        return;  // a zoom with nobody followed in it
    }
    // The followed aircraft's height to the moment shown, against the field's
    // in a zoom. Pushed apart from the plot, which flush() would otherwise
    // send the margin between them with.
    int32_t fieldFt = INT32_MIN;
    if (g_view.kind == replay::View::ZOOM) {
        int count = 0;
        const Runway *runways = runwaysAt(g_view.code, count);
        if (count) {
            fieldFt = runways[0].elevationFt;
        }
    }
    radarTelemetryDraw(TEL_X, TEL_Y, replay::followHeights(g_view.hex), g_t, fieldFt, g_lostS);
    if (!full && push) {
        screen::flush();
    }
}

// The followed aircraft, lost by the feed at g_t - missing from the poll, or
// there with only its last position - put where it is estimated to be from
// its reports, dimmed, as the live plot puts it: for up to as long
// as following would have waited for it.
void keepFollowedInScene() {
    const String hex = g_view.hex;
    g_lostS = 0;
    for (const Aircraft &a : g_scene) {
        if (a.hex == hex && a.hasPos && !a.posStale) {
            return;
        }
    }
    FollowEstimate e;
    uint32_t lostMs;
    if (!replay::estimateAt(hex, g_t, e, lostMs)) {
        return;
    }
    g_lostS = lostMs / 1000 + 1;
    Aircraft ghost;
    for (const Aircraft &a : g_scene) {
        if (a.hex == hex) {
            ghost = a;  // its callsign, type and the rest, as the poll had them
        }
    }
    g_scene.erase(std::remove_if(g_scene.begin(), g_scene.end(), [&](const Aircraft &a) { return a.hex == hex; }),
                  g_scene.end());
    ghost.hex = hex;
    if (!ghost.callsign.length()) {
        ghost.callsign = replay::callsignOf(hex);
    }
    ghost.hasPos = true;
    ghost.posStale = true;
    ghost.lat = e.lat;
    ghost.lon = e.lon;
    ghost.hasTrack = true;
    ghost.track = e.track;
    ghost.altStr = e.ground ? String("GND") : String(e.altFt);
    ghost.speedStr = String(e.gsKt);
    ghost.status = e.ground                           ? (e.gsKt > 2 ? "TAXI" : "GROUND")
                   : e.vsFpm >= CLIMB_THRESHOLD_FPM   ? "CLIMB"
                   : e.vsFpm <= DESCEND_THRESHOLD_FPM ? "DESCEND"
                                                      : "LEVEL";
    g_scene.push_back(ghost);
}

void drawViewPlot(RadarScene &scene, bool full) {
    scene.followHex = g_view.hex;
    if (g_view.hex[0]) {
        keepFollowedInScene();
    } else {
        g_lostS = 0;
    }
    if (g_view.kind == replay::View::ZOOM && radarZoomPlotDraw(scene, g_view.code, full)) {
        return;
    }
    // Where it is now, glided between polls as the rest are - or, missing
    // from this one, where it was last reported.
    bool found = false;
    for (const Aircraft &a : g_scene) {
        if (a.hex == g_view.hex && a.hasPos) {
            scene.lat = a.lat;
            scene.lon = a.lon;
            found = true;
            break;
        }
    }
    float lat, lon;
    if (!found && replay::lastFixOf(g_view.hex, g_t, lat, lon)) {
        scene.lat = lat;
        scene.lon = lon;
    }
    // At the range it was shown at live, worked out the same way from the same
    // poll - and from a poll missing it, the range it last had, as live.
    Aircraft followed;
    if (replay::followedAt(g_t, followed)) {
        g_followRange = followRangeFor(followed);
    }
    scene.rangeNm = g_followRange;
    scene.centre = RadarCentre::HOME;
    scene.originCode = replay::departedFrom(g_t);  // a departure's airport, marked as it climbs away
    radarPlotDraw(scene, full);
}

// Whether the view at g_t is a different one from the last drawn, and so
// wants a full redraw - its header, panel note and footers differ. Takes it
// as the one shown.
bool viewChanges() {
    replay::View view = replay::viewAt(g_t);
    if (view.sameAs(g_view)) {
        return false;
    }
    g_view = view;
    return true;
}

void drawWhole();

void drawScene() {
    replay::sceneAt(g_t, g_scene, g_trails);
    if (viewChanges()) {
        drawWhole();
        return;
    }
    drawPlot(false, true);
    drawDynamic();
    screen::flush();
}

void seek(uint32_t t) {
    g_t = std::min(t, replay::durationMs());
    g_dirty = true;
}

}  // namespace

bool playbackScreenOpen(const String &path, replay::LoadProgress progress) {
    if (!replay::load(path, progress)) {
        return false;
    }
    g_t = 0;
    g_playing = false;
    g_dirty = false;
    g_view = replay::View();
    g_followRange = FOLLOW_RANGE_NM;
    g_path = path;
    g_resultLine = g_resultDetail = "";
    checkVideo();
    return true;
}

void playbackScreenClose() {
    g_playing = false;
    g_scene.clear();
    g_trails.clear();
    replay::unload();
}

void playbackScreenSetCentre(RadarCentre centre) { g_centre = centre; }
void playbackScreenSetAirports(bool airports) { g_airports = airports; }

void playbackScreenDraw() {
    replay::sceneAt(g_t, g_scene, g_trails);
    viewChanges();
    drawWhole();
    g_lastTickMs = millis();
}

namespace {

// The whole screen, from the scene already taken.
void drawWhole() {
    const RadarPalette &p = radarPalette();
    screen::clear(p.bg);
    drawHeader(false);
    drawStaticPanel(false);
    drawPlot(true, true);
    drawDynamic();
    screen::flush();
    g_lastDrawMs = millis();
    g_dirty = false;
}

}  // namespace

void playbackScreenTick() {
    uint32_t now = millis();
    if (g_playing) {
        g_t += (now - g_lastTickMs) * SPEEDS[g_speedIdx];
        if (g_t >= replay::durationMs()) {
            g_t = replay::durationMs();
            g_playing = false;
            drawPlayButton();
        }
        g_dirty = true;
    }
    g_lastTickMs = now;

    uint32_t frameMs = (SPEEDS[g_speedIdx] == 1) ? FRAME_MS_SLOW : FRAME_MS_FAST;
    if (g_dirty && (!g_playing || now - g_lastDrawMs >= frameMs)) {
        drawScene();
        g_lastDrawMs = now;
        g_dirty = false;
    }
}

PlaybackAction playbackScreenHandleTouch(int x, int y, Aircraft &outAircraft) {
    if (y >= BACK_Y && y < BACK_Y + BACK_H) {
        if (x >= BACK_X && x < BACK_X + BACK_W) {
            return PlaybackAction::BACK;
        }
        if (x >= CENTRE_X && x < CENTRE_X + CENTRE_W && hasHome()) {
            return PlaybackAction::TOGGLE_CENTRE;
        }
    }
    if (y >= EXPORT_Y && y < EXPORT_Y + BACK_H && x >= EXPORT_X && x < EXPORT_X + EXPORT_W && !g_videoExists) {
        return PlaybackAction::EXPORT;
    }

    if (x >= PANEL_X && x < PANEL_X + PANEL_W) {
        if (y >= PLAY_Y && y < PLAY_Y + PLAY_H) {
            if (!g_playing && g_t >= replay::durationMs()) {
                seek(0);  // at the end, Play starts it over
            }
            g_playing = !g_playing;
            g_lastTickMs = millis();
            drawPlayButton();
            screen::flush();
            return PlaybackAction::NONE;
        }
        if (y >= STEP_Y && y < STEP_Y + STEP_H) {
            int col = (x - PANEL_X) / (STEP_W + STEP_GAP);
            if (col == 0) {
                seek(g_t > SKIP_MS ? g_t - SKIP_MS : 0);
            } else if (col == 1) {
                g_speedIdx = (g_speedIdx + 1) % SPEED_COUNT;
                drawStepButtons();
                checkVideo();  // a video at one speed says nothing about another
                drawExportButton();
                screen::flush();
            } else {
                seek(g_t + SKIP_MS);
            }
            drawPlayButton();  // a skip can land on, or leave, the end
            screen::flush();
            return PlaybackAction::NONE;
        }
        if (y >= BAR_Y - BAR_TOUCH_PAD && y < BAR_Y + BAR_H + BAR_TOUCH_PAD) {
            seek((uint32_t)((uint64_t)replay::durationMs() * (x - PANEL_X) / PANEL_W));
            drawPlayButton();
            screen::flush();
            return PlaybackAction::NONE;
        }
        return PlaybackAction::NONE;
    }

    String hex;
    if (radarPlotHit(x, y, hex)) {
        for (const Aircraft &a : g_scene) {
            if (a.hex == hex) {
                // Paused, so the moment is still there to come back to.
                if (g_playing) {
                    g_playing = false;
                }
                outAircraft = a;
                return PlaybackAction::SELECT;
            }
        }
    }
    return PlaybackAction::NONE;
}

void playbackScreenExport() {
    const RadarPalette &p = radarPalette();
    int speed = SPEEDS[g_speedIdx];
    uint32_t dur = replay::durationMs();
    // How far into the recording each frame of video moves on.
    uint32_t stepMs = (uint32_t)speed * 1000 / VIDEO_FPS;
    uint32_t total = dur / stepMs + 1;

    String path = videoPath();

    g_playing = false;
    uint32_t savedT = g_t;

    VideoWriter video;
    if (!video.open(path, VIDEO_FPS)) {
        g_resultLine = "Export failed";
        g_resultDetail = "No card, or the encoder wouldn't start";
        playbackScreenDraw();
        return;
    }
    Serial.printf("[video] exporting %s at %dx: %u frames\n", path.c_str(), speed, (unsigned)total);

    auto &canvas = screen::canvas();
    uint32_t start = millis(), lastShown = 0;
    uint64_t renderUs = 0;
    bool overlay = false, cancelled = false, failed = false;
    for (uint32_t k = 0; k < total; k++) {
        uint32_t r0 = micros();
        g_t = std::min(k * stepMs, dur);
        replay::sceneAt(g_t, g_scene, g_trails);
        bool newView = viewChanges();
        if (k == 0 || newView) {
            // The first frame is the whole picture, as is the first of each
            // change of view; after it only the plot and the panel's moving
            // parts change.
            screen::clear(p.bg);
            drawHeader(true);
            drawStaticPanel(true);
            drawPlot(true, false);
            overlay = false;
        } else {
            if (overlay) {
                canvas.fillRect(PANEL_X, CTRL_Y, PANEL_W, CTRL_H, p.bg);  // the progress is no part of the video
                screen::markDirty(PANEL_X, CTRL_Y, PANEL_W, CTRL_H);
                overlay = false;
            }
            drawPlot(false, false);
        }
        drawDynamic();
        renderUs += micros() - r0;

        if (!video.addCanvasFrame()) {
            failed = true;
            break;
        }

        // Now and then the frame just encoded goes to the panel too, with the
        // progress drawn over it - after the encode, so never into the video.
        uint32_t now = millis();
        if (k == 0 || now - lastShown >= PROGRESS_MS) {
            lastShown = now;
            drawExportProgress(k + 1, total, now - start);
            overlay = true;
            screen::flush();
        }

        // Read here rather than by loop(), which isn't running: a press is
        // enough, since a tap could start and end between two frames.
        M5.update();
        if (M5.Touch.getCount()) {
            auto t = M5.Touch.getDetail(0);
            if (t.isPressed() && t.x >= PANEL_X && t.x < PANEL_X + PANEL_W && t.y >= STEP_Y && t.y < STEP_Y + STEP_H) {
                cancelled = true;
                break;
            }
        }
        vTaskDelay(1);  // the idle task on this core needs a look in, or its watchdog complains
    }

    if (cancelled || failed) {
        video.abort();
        g_resultLine = cancelled ? "Export cancelled" : "Export failed";
        g_resultDetail = cancelled ? "" : "Card full or removed?";
        if (cancelled) {
            waitForRelease();
        }
    } else if (video.close()) {
        g_resultLine = "Video saved, " + String(video.bytes() / (1024.0f * 1024.0f), 1) + " MB";
        g_resultDetail = path;
    } else {
        g_resultLine = "Export failed";
        g_resultDetail = "Couldn't finish the file";
    }
    // Read once it is finished: until then the last frames may still be with
    // the encoder.
    uint32_t frames = video.frames();
    uint32_t secs = (millis() - start) / 1000;
    if (frames > 0) {
        Serial.printf("[video] %s: %s, %u frames in %us - per frame: render %u, convert %u, encode %u, write %u ms\n",
                      path.c_str(), g_resultLine.c_str(), (unsigned)frames, (unsigned)secs,
                      (unsigned)(renderUs / 1000 / frames), (unsigned)(video.convertMs() / frames),
                      (unsigned)(video.encodeMs() / frames), (unsigned)(video.writeMs() / frames));
    }

    g_t = savedT;
    checkVideo();  // made now, or - cancelled or failed - still not
    playbackScreenDraw();
}
