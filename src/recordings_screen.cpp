#include "recordings_screen.h"

#include <M5Unified.h>
#include <algorithm>
#include <time.h>
#include <vector>

#include "recorder.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int SHARE_W = 124;
constexpr int SHARE_X = BACK_X - 12 - SHARE_W;
constexpr int STATUS_Y = 70;
constexpr int LIST_X = 16, LIST_Y = 104, LIST_W = 1248, ROW_H = 74, ROW_GAP = 8;
constexpr int ROWS_PER_PAGE = 7;
constexpr int DEL_W = 140, DEL_H = 50;
constexpr int DEL_X = LIST_X + LIST_W - DEL_W - 12;
constexpr int PAGER_Y = 650, PAGER_W = 180, PAGER_H = 60;
constexpr int PREV_X = 16, NEXT_X = 1264 - PAGER_W;

struct Row {
    bool ok = false;
    recorder::Header header;
    uint32_t bytes = 0;
    std::vector<String> videos;  // exported from it, one per speed
};

std::vector<String> g_paths;
std::vector<String> g_videos;  // every video on the card, read with the list
Row g_rows[ROWS_PER_PAGE];
int g_page = 0;
// A delete takes two taps: the first arms the row's button, the second acts.
// Anything else tapped in between disarms it.
int g_confirmRow = -1;
bool g_mounted = false;
String g_activePath;

// The row being opened, while its recording loads. Loading reads the whole
// file to index it - a few seconds for a full-sized part - and the screen
// would otherwise sit there looking as if the tap had been missed.
int g_openingRow = -1;
uint32_t g_openingSinceMs = 0;
int g_shownPct = -1;
// A short recording is open before a bar could be read, and one flashed up
// and straight off reads as a glitch - so only a load still going after this
// long gets one.
constexpr uint32_t PROGRESS_AFTER_MS = 150;

uint16_t colorBg, colorWhite, colorGrey, colorDim, colorBtnBg, colorRowBg, colorBorder, colorDelete, colorRec;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorGrey = M5.Display.color565(0xBB, 0xBB, 0xBB);
    colorDim = M5.Display.color565(0x88, 0x91, 0x9B);
    colorBtnBg = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorRowBg = M5.Display.color565(0x1C, 0x22, 0x29);
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorDelete = M5.Display.color565(0xC0, 0x39, 0x2B);
    colorRec = M5.Display.color565(0xFF, 0x4D, 0x4D);
    colorsReady = true;
}

int pageCount() { return g_paths.empty() ? 1 : (int)((g_paths.size() + ROWS_PER_PAGE - 1) / ROWS_PER_PAGE); }

void loadPage() {
    for (int i = 0; i < ROWS_PER_PAGE; i++) {
        size_t idx = (size_t)g_page * ROWS_PER_PAGE + i;
        g_rows[i] = Row();
        if (idx < g_paths.size()) {
            g_rows[i].ok = recorder::readHeader(g_paths[idx], g_rows[i].header, g_rows[i].bytes);
            g_rows[i].videos = recorder::videosOf(g_paths[idx], g_videos);
        }
    }
}

void reload() {
    g_mounted = recorder::mount();
    g_paths = g_mounted ? recorder::list() : std::vector<String>();
    g_videos = g_mounted ? recorder::videos() : std::vector<String>();
    g_activePath = recorder::activePath();
    if (g_page >= pageCount()) {
        g_page = pageCount() - 1;
    }
    g_confirmRow = -1;
    g_openingRow = -1;
    loadPage();
}

String baseName(const String &path) {
    int slash = path.lastIndexOf('/');
    return slash >= 0 ? path.substring(slash + 1) : path;
}

String sizeText(uint32_t bytes) {
    if (bytes >= 1024 * 1024) {
        return String(bytes / (1024.0f * 1024.0f), 1) + " MB";
    }
    return String((bytes + 1023) / 1024) + " KB";
}

void drawRow(int i) {
    auto &canvas = screen::canvas();
    int y = LIST_Y + i * ROW_H;
    int h = ROW_H - ROW_GAP;
    canvas.fillRect(LIST_X, y, LIST_W, ROW_H, colorBg);
    size_t idx = (size_t)g_page * ROWS_PER_PAGE + i;
    if (idx >= g_paths.size()) {
        screen::markDirty(LIST_X, y, LIST_W, ROW_H);
        return;
    }
    const String &path = g_paths[idx];
    const Row &row = g_rows[i];
    bool active = (path == g_activePath);

    canvas.fillRoundRect(LIST_X, y, LIST_W, h, 8, colorRowBg);
    canvas.drawRoundRect(LIST_X, y, LIST_W, h, 8, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(ML_DATUM);

    String title = baseName(path);
    if (row.ok && row.header.startEpoch) {
        time_t t = row.header.startEpoch;
        struct tm tm;
        gmtime_r(&t, &tm);
        char buf[40];
        strftime(buf, sizeof(buf), "%a %d %b %Y  %H:%M UTC", &tm);
        title = buf;
    }
    canvas.setTextSize(3);
    canvas.setTextColor(colorWhite);
    canvas.drawString(title, LIST_X + 20, y + 22);

    String detail;
    uint16_t detailColor = colorDim;
    if (active) {
        detail = "Recording now";
        detailColor = colorRec;
    } else if (!row.ok) {
        detail = "Not a recording this version can read";
    } else {
        detail = row.header.trigger == recorder::Trigger::AUTO
                     ? "Auto" + (row.header.note.length() ? " - " + row.header.note : String(""))
                     : String("Manual");
        if (row.header.part > 1) {
            detail += ", part " + String(row.header.part);  // carries on from the one below it
        }
        detail += "    " + String(row.header.radiusNm) + " nm " + (row.header.military ? "military" : "civil");
        detail += "    " + sizeText(row.bytes);
    }
    bool armed = (g_confirmRow == i);
    if (armed && !row.videos.empty()) {
        // Its videos go with it, which is worth saying before it happens.
        detail = row.videos.size() == 1 ? "Tap Sure? again to delete this and its video"
                                         : "Tap Sure? again to delete this and its " + String(row.videos.size()) +
                                               " videos";
        detailColor = colorRec;
    }
    canvas.setTextSize(2);
    canvas.setTextColor(detailColor);
    canvas.drawString(detail, LIST_X + 20, y + 50);

    // Which speeds it has been exported at, after the rest and in white, so a
    // recording with a video can be picked out down the list.
    if (!armed && !row.videos.empty()) {
        std::vector<int> speeds;
        for (const String &path : row.videos) {
            speeds.push_back(path.substring(path.lastIndexOf('-') + 1).toInt());  // "...-16x.mp4" -> 16
        }
        std::sort(speeds.begin(), speeds.end());
        String tag = "    video";
        for (size_t v = 0; v < speeds.size(); v++) {
            tag += String(v ? ", " : " ") + String(speeds[v]) + "x";
        }
        canvas.setTextColor(colorWhite);
        canvas.drawString(tag, LIST_X + 20 + canvas.textWidth(detail), y + 50);
    }

    if (!active) {
        int by = y + (h - DEL_H) / 2;
        canvas.fillRoundRect(DEL_X, by, DEL_W, DEL_H, 6, armed ? colorDelete : colorBtnBg);
        canvas.setTextColor(colorWhite);
        canvas.setTextDatum(MC_DATUM);
        canvas.drawString(armed ? "Sure?" : "Delete", DEL_X + DEL_W / 2, by + DEL_H / 2);
        if (row.ok) {
            canvas.setTextSize(3);
            canvas.setTextColor(colorGrey);
            canvas.setTextDatum(MR_DATUM);
            canvas.drawString(">", DEL_X - 24, y + h / 2);
        }
    }
    screen::markDirty(LIST_X, y, LIST_W, ROW_H);
}

void drawPager() {
    auto &canvas = screen::canvas();
    canvas.fillRect(0, PAGER_Y, 1280, PAGER_H, colorBg);
    if (pageCount() > 1) {
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(2);
        canvas.setTextDatum(MC_DATUM);
        if (g_page > 0) {
            canvas.fillRoundRect(PREV_X, PAGER_Y, PAGER_W, PAGER_H, 6, colorBtnBg);
            canvas.setTextColor(colorWhite);
            canvas.drawString("< Newer", PREV_X + PAGER_W / 2, PAGER_Y + PAGER_H / 2);
        }
        if (g_page + 1 < pageCount()) {
            canvas.fillRoundRect(NEXT_X, PAGER_Y, PAGER_W, PAGER_H, 6, colorBtnBg);
            canvas.setTextColor(colorWhite);
            canvas.drawString("Older >", NEXT_X + PAGER_W / 2, PAGER_Y + PAGER_H / 2);
        }
        canvas.setTextColor(colorDim);
        canvas.drawString("Page " + String(g_page + 1) + " of " + String(pageCount()), 640, PAGER_Y + PAGER_H / 2);
    }
    screen::markDirty(0, PAGER_Y, 1280, PAGER_H);
}

void drawList() {
    for (int i = 0; i < ROWS_PER_PAGE; i++) {
        drawRow(i);
    }
    drawPager();
}

}  // namespace

void recordingsScreenEnter() {
    g_page = 0;
    reload();
    recordingsScreenDraw();
}

void recordingsScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Recordings", 16, 12);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    if (g_mounted) {
        canvas.fillRoundRect(SHARE_X, BACK_Y, SHARE_W, BACK_H, 6, colorBtnBg);
        canvas.drawString("Share", SHARE_X + SHARE_W / 2, BACK_Y + BACK_H / 2);
    }

    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(colorDim);
    String status;
    if (!g_mounted) {
        status = "No SD card found. Insert a FAT32 card to record and replay.";
    } else if (g_paths.empty()) {
        status = "No recordings yet. Tap REC on the radar, or turn on auto-record in Settings > Alerts.";
    } else {
        status = String(g_paths.size()) + (g_paths.size() == 1 ? " recording" : " recordings") +
                 ", newest first. Tap one to play it.";
    }
    canvas.drawString(status, 16, STATUS_Y);

    drawList();
    screen::flush();
}

RecordingsAction recordingsScreenHandleTouch(int x, int y, String &outPath) {
    if (x >= BACK_X && x < BACK_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return RecordingsAction::BACK;
    }
    if (g_mounted && x >= SHARE_X && x < SHARE_X + SHARE_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return RecordingsAction::SHARE;
    }

    if (y >= PAGER_Y && y < PAGER_Y + PAGER_H) {
        int page = g_page;
        if (x >= PREV_X && x < PREV_X + PAGER_W && g_page > 0) {
            page--;
        } else if (x >= NEXT_X && x < NEXT_X + PAGER_W && g_page + 1 < pageCount()) {
            page++;
        }
        if (page != g_page) {
            g_page = page;
            g_confirmRow = -1;
            loadPage();
            drawList();
            screen::flush();
        }
        return RecordingsAction::NONE;
    }

    if (x >= LIST_X && x < LIST_X + LIST_W && y >= LIST_Y && y < LIST_Y + ROWS_PER_PAGE * ROW_H) {
        int i = (y - LIST_Y) / ROW_H;
        size_t idx = (size_t)g_page * ROWS_PER_PAGE + i;
        if (idx >= g_paths.size()) {
            return RecordingsAction::NONE;
        }
        bool active = (g_paths[idx] == g_activePath);
        int prevConfirm = g_confirmRow;
        if (!active && x >= DEL_X && x < DEL_X + DEL_W) {
            if (g_confirmRow == i) {
                for (const String &video : g_rows[i].videos) {
                    recorder::removeVideo(video);
                }
                recorder::remove(g_paths[idx]);
                reload();
                recordingsScreenDraw();
                return RecordingsAction::NONE;
            }
            g_confirmRow = i;
            if (prevConfirm >= 0) {
                drawRow(prevConfirm);
            }
            drawRow(i);
            screen::flush();
            return RecordingsAction::NONE;
        }
        if (prevConfirm >= 0) {
            g_confirmRow = -1;
            drawRow(prevConfirm);
            screen::flush();
        }
        if (active || !g_rows[i].ok) {
            return RecordingsAction::NONE;
        }
        outPath = g_paths[idx];
        g_openingRow = i;
        g_openingSinceMs = millis();
        g_shownPct = -1;
        return RecordingsAction::PLAY;
    }
    return RecordingsAction::NONE;
}

void recordingsScreenLoadProgress(uint32_t done, uint32_t total) {
    if (g_openingRow < 0 || millis() - g_openingSinceMs < PROGRESS_AFTER_MS) {
        return;
    }
    int pct = total ? (int)((uint64_t)std::min(done, total) * 100 / total) : 100;
    if (pct == g_shownPct) {
        return;
    }
    g_shownPct = pct;

    // Over the row's detail line, which is the least needed while it loads -
    // the title still says which recording it is.
    auto &canvas = screen::canvas();
    int y = LIST_Y + g_openingRow * ROW_H;
    int lineY = y + 50;
    int left = LIST_X + 2, right = DEL_X - 60;
    canvas.fillRect(left, lineY - 12, right - left, 24, colorRowBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(colorWhite);
    canvas.drawString("Loading " + String(pct) + "%", LIST_X + 20, lineY);

    constexpr int BAR_H = 12;
    int barX = LIST_X + 20 + 160;
    int barW = right - 20 - barX;
    canvas.fillRoundRect(barX, lineY - BAR_H / 2, barW, BAR_H, BAR_H / 2, colorBorder);
    int filled = barW * pct / 100;
    if (filled > 0) {
        canvas.fillRoundRect(barX, lineY - BAR_H / 2, std::max(filled, BAR_H), BAR_H, BAR_H / 2, colorWhite);
    }
    screen::markDirty(left, lineY - 12, right - left, 24);
    screen::flush();
}
