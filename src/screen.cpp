#include "screen.h"

#include "config.h"

#include <driver/ppa.h>
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>
#include <soc/dma2d_struct.h>
#include <soc/ppa_struct.h>

#include <algorithm>

namespace screen {
namespace {

constexpr int CANVAS_W = 1280;
constexpr int CANVAS_H = 720;
constexpr size_t CANVAS_BYTES = (size_t)CANVAS_W * CANVAS_H * 2;

M5Canvas g_canvas(&M5.Display);
uint16_t *g_buffer = nullptr;

void *g_fb = nullptr;  // the panel's live framebuffer, portrait
size_t g_fbBytes = 0;
int g_panelW = 0, g_panelH = 0;

ppa_client_handle_t g_srm = nullptr;
ppa_client_handle_t g_fillClient = nullptr;
// Turns at the PPA: a band pushed straight to the panel - the 3D view's,
// from one core - holds it from its start to its end, and a blit of the
// canvas, from the other, waits for it rather than going at once. Two
// clients sending together from two cores now and then lost a push's end.
// And the ends of a rotate-and-copy and of a fill, given from the PPA's
// interrupt.
//
// Every rotate-and-copy, the canvas's blits and the 3D view's bands alike,
// goes through the one client, g_srm, and none blocks: each is sent and its
// end waited for, PPA_WAIT_MS at most. A blocking blit whose end never came
// froze the board for good on 17 Oct - the loop waiting on it, in the 3D
// view's details strip - and two clients, a blit's and the bands', taking
// turns from two cores, had lost ends before. Should one still not finish in
// time, the PPA is given up for the rest of the session and the CPU pushes
// the screen instead: slower, but going on.
SemaphoreHandle_t g_pushTurn = nullptr, g_pushDone = nullptr, g_fillDone = nullptr;
constexpr uint32_t PPA_WAIT_MS = 500, PPA_TURN_MS = 2000;
volatile bool g_ppaStuck = false;
bool g_ppaOk = false;
// A part of the screen whose pixels the panel has rather than the canvas -
// the 3D view's, sent straight to it a band at a time - for a screenshot to
// take from the panel's framebuffer instead, and a blit to keep off.
int g_liveX = 0, g_liveY = 0, g_liveW = 0, g_liveH = 0;
ppa_srm_rotation_angle_t g_angle = PPA_SRM_ROTATION_ANGLE_90;

struct Rect {
    int16_t x, y, w, h;
};

// 16 is comfortably more than the number of table rows, so a normal refresh
// never has to fall back to the merged bounding box.
constexpr int MAX_RECTS = 16;
Rect g_rects[MAX_RECTS];
int g_rectCount = 0;
// The same, kept for takeChanged(): not cleared by a flush, only by being taken.
Rect g_changed[MAX_RECTS];
int g_changedCount = 0;

OverlayFn g_overlay = nullptr;
Rect g_overlayRect = {0, 0, 0, 0};

bool intersects(const Rect &a, const Rect &b) {
    return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

// What watch() is watching, and whether anything has been drawn over it -
// touched until something is watched.
Rect g_watch = {0, 0, 0, 0};
bool g_touched = true;

void noteDrawn(const Rect &r) {
    if (intersects(r, g_watch)) {
        g_touched = true;
    }
}

// Cells in the same row share a y/h, so merging those into one wide rect keeps
// the rect count at one per row instead of one per cell.
constexpr int MERGE_GAP = 96;

// Adds a rect to a list, absorbed by one it lies inside, merged into one in
// the same band it is close to (with `merge`), or - when the list is full -
// with everything collapsed into one bounding box. Cheaper than paying the
// per-transfer overhead many times over.
//
// The merge suits a flush, where a strip takes no longer to push than its
// pieces. It doesn't suit a conversion for the encoder, which costs its area:
// on a replay it joined the side panel, the plot and the footer beside it into
// one box of nearly the whole screen.
void addRect(Rect *rects, int &count, int x, int y, int w, int h, bool merge) {
    for (int i = 0; i < count; i++) {
        Rect &r = rects[i];
        if (x >= r.x && y >= r.y && x + w <= r.x + r.w && y + h <= r.y + r.h) {
            return;  // already covered
        }
        // same band, near enough horizontally: widen it instead of adding one
        if (merge && y >= r.y && y + h <= r.y + r.h && x <= r.x + r.w + MERGE_GAP && x + w + MERGE_GAP >= r.x) {
            int right = std::max<int>(r.x + r.w, x + w);
            r.x = std::min<int>(r.x, x);
            r.w = right - r.x;
            return;
        }
    }
    if (count == MAX_RECTS) {
        int l = rects[0].x, t = rects[0].y, rgt = l + rects[0].w, bot = t + rects[0].h;
        for (int i = 1; i < count; i++) {
            l = std::min<int>(l, rects[i].x);
            t = std::min<int>(t, rects[i].y);
            rgt = std::max<int>(rgt, rects[i].x + rects[i].w);
            bot = std::max<int>(bot, rects[i].y + rects[i].h);
        }
        l = std::min(l, x);
        t = std::min(t, y);
        rgt = std::max(rgt, x + w);
        bot = std::max(bot, y + h);
        count = 1;
        rects[0] = {(int16_t)l, (int16_t)t, (int16_t)(rgt - l), (int16_t)(bot - t)};
        return;
    }
    rects[count++] = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
}


// A canvas row is contiguous and 2560 bytes long - a whole number of 64-byte
// cache lines - so any span of rows is a flat, correctly aligned range.
constexpr size_t ROW_BYTES = (size_t)CANVAS_W * 2;

void writeBackCanvas() {
    // The PPA reads the canvas by DMA, so the CPU's dirty cache lines have to
    // be in PSRAM first.
    esp_cache_msync(g_buffer, CANVAS_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

#ifdef RENDER_PROFILE
size_t g_syncedBytes = 0;
#endif

// The canvas's work at the PPA, for takePpaPerf().
PpaPerf g_perf;

// Writes back only the rows the dirty rects cover, rather than all 1.8MB of
// the canvas. A refresh that repaints one cell touches 48 rows - 123KB - so
// the full-canvas sync that used to precede every flush was doing fifteen
// times the work of the transfer it was preparing for.
//
// Safe because every canvas write is covered by the dirty list at the flush
// that follows it: a partial redraw marks the region it touched, and a full
// repaint goes through clear(), which marks the lot. Rows outside the list
// were therefore written back by an earlier flush and are already clean in
// PSRAM, which is what lets the merged bounding-box blit below stay correct.
void writeBackDirty() {
    struct Span {
        int16_t top, bot;
    };
    Span spans[MAX_RECTS];
    int n = 0;
    for (int i = 0; i < g_rectCount; i++) {
        spans[n++] = {g_rects[i].y, (int16_t)(g_rects[i].y + g_rects[i].h)};
    }
    // Merge overlapping bands - cells in the same table row share their rows,
    // and syncing those once each rather than once per cell is the whole point.
    std::sort(spans, spans + n, [](const Span &a, const Span &b) { return a.top < b.top; });
    int64_t t0 = esp_timer_get_time();
    int i = 0;
    while (i < n) {
        int top = spans[i].top, bot = spans[i].bot;
        while (++i < n && spans[i].top <= bot) {
            bot = std::max<int>(bot, spans[i].bot);
        }
        esp_cache_msync((uint8_t *)g_buffer + (size_t)top * ROW_BYTES, (size_t)(bot - top) * ROW_BYTES,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M);
#ifdef RENDER_PROFILE
        g_syncedBytes += (size_t)(bot - top) * ROW_BYTES;
#endif
    }
    g_perf.syncUs += (uint64_t)(esp_timer_get_time() - t0);
}

// The ends of transfers, given from the PPA's interrupt - and how long the
// rotate-and-copies take, start to finish, for the bench to say.
volatile int64_t g_pushStartUs = 0;
volatile uint32_t g_pushBusyUs = 0, g_pushCount = 0;

bool IRAM_ATTR pushDone(ppa_client_handle_t, ppa_event_data_t *, void *done) {
    g_pushBusyUs = g_pushBusyUs + (uint32_t)(esp_timer_get_time() - g_pushStartUs);
    g_pushCount = g_pushCount + 1;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)done, &woken);
    return woken == pdTRUE;
}

bool IRAM_ATTR fillDone(ppa_client_handle_t, ppa_event_data_t *, void *done) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)done, &woken);
    return woken == pdTRUE;
}

// The last few transfers, for a stall's report: which - b a blit, B a band, f
// a fill, y a frame turned to YUV for a video - from which core, where to and
// how big, and when sent and done (0 for not yet). Noted under the turn, so
// one at a time.
struct Sent {
    char kind;
    uint8_t core;
    int16_t x, y, w, h;
    uint32_t sentUs, doneUs;
};
constexpr int SENT_KEPT = 16;
Sent g_sent[SENT_KEPT];
uint32_t g_sentCount = 0;

void noteSent(char kind, int x, int y, int w, int h) {
    g_sent[g_sentCount++ % SENT_KEPT] = {kind, (uint8_t)xPortGetCoreID(), (int16_t)x, (int16_t)y, (int16_t)w,
                                         (int16_t)h, (uint32_t)esp_timer_get_time(), 0};
}

void noteDone() {
    if (g_sentCount != 0) {
        g_sent[(g_sentCount - 1) % SENT_KEPT].doneUs = (uint32_t)esp_timer_get_time();
    }
}

// What the stall left behind: the transfers before it, and the 2D-DMA's
// channels and the PPA's own state - whether the hardware finished and its
// end went unheard, or never finished.
void ppaReport() {
    uint32_t now = (uint32_t)esp_timer_get_time();
    Serial.println("[screen] the transfers before it - kind, core, x y w h, sent, took:");
    for (int i = SENT_KEPT; i >= 1; i--) {
        if (g_sentCount < (uint32_t)i) {
            continue;
        }
        const Sent &t = g_sent[(g_sentCount - i) % SENT_KEPT];
        Serial.printf("  %c core %u  %d %d %d %d  %uus ago, ", t.kind, t.core, t.x, t.y, t.w, t.h,
                      (unsigned)(now - t.sentUs));
        if (t.doneUs != 0) {
            Serial.printf("took %uus\n", (unsigned)(t.doneUs - t.sentUs));
        } else {
            Serial.println("NOT DONE");
        }
    }
    for (int c = 0; c < 3; c++) {
        auto &ch = DMA2D.out_channel[c];
        Serial.printf("  dma2d tx%d: raw %08x st %08x ena %08x state %08x dscr %08x link %08x conf0 %08x\n", c,
                      (unsigned)ch.out_int_raw.val, (unsigned)ch.out_int_st.val, (unsigned)ch.out_int_ena.val,
                      (unsigned)ch.out_state.val, (unsigned)ch.out_dscr.val, (unsigned)ch.out_link_addr.val,
                      (unsigned)ch.out_conf0.val);
    }
    for (int c = 0; c < 2; c++) {
        auto &ch = DMA2D.in_channel[c];
        Serial.printf("  dma2d rx%d: raw %08x st %08x ena %08x state %08x dscr %08x link %08x conf0 %08x\n", c,
                      (unsigned)ch.in_int_raw.val, (unsigned)ch.in_int_st.val, (unsigned)ch.in_int_ena.val,
                      (unsigned)ch.in_state.val, (unsigned)ch.in_dscr.val, (unsigned)ch.in_link_addr.val,
                      (unsigned)ch.in_conf0.val);
    }
    Serial.printf("  ppa: raw %08x st %08x ena %08x sr_status %08x sr_param_err %08x\n", (unsigned)PPA.int_raw.val,
                  (unsigned)PPA.int_st.val, (unsigned)PPA.int_ena.val, (unsigned)PPA.sr_status.val,
                  (unsigned)PPA.sr_param_err_st.val);
}

// The PPA given up, said once - with what it left behind - and the CPU pushes
// the screen from here on.
void ppaGiveUp(const char *what) {
    if (!g_ppaStuck) {
        g_ppaStuck = true;
        Serial.printf("[screen] a PPA %s never finished - the CPU pushes the screen from now on\n", what);
        ppaReport();
    }
}

// Fallback for a board where the PPA didn't come up, or has been given up:
// LovyanGFX's own push, clipped to the dirty region so it at least isn't
// pushing the whole screen.
void blitRectFallback(const Rect &r) {
    M5.Display.setClipRect(r.x, r.y, r.w, r.h);
    g_canvas.pushSprite(0, 0);
    M5.Display.clearClipRect();
}

// A band straight onto the panel by the CPU - turned and its bytes swapped as
// the PPA would have them, as dumpToSerial() reads them back - and written
// back from the cache for the panel's DMA to read: the 3D view's, once the
// PPA has been given up.
void pushBandByCpu(const uint16_t *pixels, int x, int y, int w, int rows) {
    uint16_t *fb = (uint16_t *)g_fb;
    bool ninety = g_angle == PPA_SRM_ROTATION_ANGLE_90;
    for (int r = 0; r < rows; r++) {
        const uint16_t *from = pixels + (size_t)r * w;
        int cy = y + r, fx = ninety ? cy : CANVAS_H - 1 - cy;
        for (int c = 0; c < w; c++) {
            int cx = x + c, fy = ninety ? CANVAS_W - 1 - cx : cx;
            fb[(size_t)fy * g_panelW + fx] = __builtin_bswap16(from[c]);
        }
    }
    int top = ninety ? CANVAS_W - x - w : x;
    esp_cache_msync((uint8_t *)g_fb + (size_t)top * g_panelW * 2, (size_t)w * g_panelW * 2,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

// One transfer at a time, the turn taken, and waited for. Queueing them and
// waiting only on the last was measured *slower* - 16.8us per KB against
// 12.5 - because the CPU was never what the transfers were waiting on, and
// the queue and completion interrupt cost more than the round trip they
// replaced.
void blitRect(const Rect &given) {
    if (g_ppaStuck) {
        blitRectFallback(given);
        return;
    }
    // At least 16 pixels each way - grown down and right, or up and left at
    // the canvas's edge - the PPA having been seen never to finish a band
    // only 4 rows high; the canvas holds the whole screen, so the rest is
    // only drawn again as it is. Not where that would reach the panel's live
    // part, which the canvas hasn't.
    Rect r = given;
    if (r.w < 16 || r.h < 16) {
        Rect g = r;
        if (g.w < 16) {
            g.w = 16;
            g.x = (int16_t)std::min<int>(g.x, CANVAS_W - 16);
        }
        if (g.h < 16) {
            g.h = 16;
            g.y = (int16_t)std::min<int>(g.y, CANVAS_H - 16);
        }
        Rect live = {(int16_t)g_liveX, (int16_t)g_liveY, (int16_t)g_liveW, (int16_t)g_liveH};
        if (g_liveW <= 0 || !intersects(g, live) || intersects(given, live)) {
            r = g;
        }
    }
    ppa_srm_oper_config_t op = {};
    op.in.buffer = g_buffer;
    op.in.pic_w = CANVAS_W;
    op.in.pic_h = CANVAS_H;
    op.in.block_w = r.w;
    op.in.block_h = r.h;
    op.in.block_offset_x = r.x;
    op.in.block_offset_y = r.y;
    op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;

    op.out.buffer = g_fb;
    op.out.buffer_size = g_fbBytes;
    op.out.pic_w = g_panelW;
    op.out.pic_h = g_panelH;
    // Where the rotated block lands in the portrait framebuffer. Verified on
    // hardware for both angles rather than trusted from the docs.
    if (g_angle == PPA_SRM_ROTATION_ANGLE_90) {
        op.out.block_offset_x = r.y;
        op.out.block_offset_y = CANVAS_W - r.x - r.w;
    } else {
        op.out.block_offset_x = CANVAS_H - r.y - r.h;
        op.out.block_offset_y = r.x;
    }
    op.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;

    op.rotation_angle = g_angle;
    op.scale_x = 1.0f;
    op.scale_y = 1.0f;
    // The PPA's RGB565 byte order is the opposite of the one LovyanGFX uses for
    // this panel; without this every colour comes out channel-swapped.
    op.byte_swap = true;
    op.mode = PPA_TRANS_MODE_NON_BLOCKING;
    op.user_data = g_pushDone;

    if (xSemaphoreTake(g_pushTurn, pdMS_TO_TICKS(PPA_TURN_MS)) != pdTRUE) {
        ppaGiveUp("band push, holding the turn,");
        blitRectFallback(r);
        return;
    }
    int64_t t0 = esp_timer_get_time();
    g_pushStartUs = t0;
    noteSent('b', r.x, r.y, r.w, r.h);
    esp_err_t err = ppa_do_scale_rotate_mirror(g_srm, &op);
    bool finished = err == ESP_OK && xSemaphoreTake(g_pushDone, pdMS_TO_TICKS(PPA_WAIT_MS)) == pdTRUE;
    if (finished) {
        noteDone();
        g_perf.blits++;
        g_perf.blitUs += (uint64_t)(esp_timer_get_time() - t0);
        g_perf.blitPixels += (uint32_t)r.w * r.h;
    }
    xSemaphoreGive(g_pushTurn);
    if (err != ESP_OK) {
        // Worth knowing about: a full queue would otherwise drop the transfer
        // and leave that region of the panel showing the previous frame.
        Serial.printf("[screen] PPA blit failed: %s\n", esp_err_to_name(err));
        blitRectFallback(r);
    } else if (!finished) {
        ppaGiveUp("blit");
        blitRectFallback(r);
    }
}

bool g_pushing = false;

}  // namespace

M5Canvas &canvas() { return g_canvas; }

bool pushBandStart(const uint16_t *pixels, int x, int y, int w, int rows) {
    if (!g_ppaOk) {
        return false;
    }
    if (g_ppaStuck) {
        pushBandByCpu(pixels, x, y, w, rows);
        return true;
    }
    esp_cache_msync((void *)pixels, ((size_t)w * rows * 2 + 63) & ~(size_t)63, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    ppa_srm_oper_config_t op = {};
    op.in.buffer = pixels;
    op.in.pic_w = w;
    op.in.pic_h = rows;
    op.in.block_w = w;
    op.in.block_h = rows;
    op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    op.out.buffer = g_fb;
    op.out.buffer_size = g_fbBytes;
    op.out.pic_w = g_panelW;
    op.out.pic_h = g_panelH;
    // As blitRect() places a rect.
    if (g_angle == PPA_SRM_ROTATION_ANGLE_90) {
        op.out.block_offset_x = y;
        op.out.block_offset_y = CANVAS_W - x - w;
    } else {
        op.out.block_offset_x = CANVAS_H - y - rows;
        op.out.block_offset_y = x;
    }
    op.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    op.rotation_angle = g_angle;
    op.scale_x = 1.0f;
    op.scale_y = 1.0f;
    op.byte_swap = true;
    op.mode = PPA_TRANS_MODE_NON_BLOCKING;
    op.user_data = g_pushDone;
    if (xSemaphoreTake(g_pushTurn, pdMS_TO_TICKS(PPA_TURN_MS)) != pdTRUE) {
        ppaGiveUp("blit, holding the turn,");
        pushBandByCpu(pixels, x, y, w, rows);
        return true;
    }
    g_pushStartUs = esp_timer_get_time();
    noteSent('B', x, y, w, rows);
    esp_err_t err = ppa_do_scale_rotate_mirror(g_srm, &op);
    if (err != ESP_OK) {
        xSemaphoreGive(g_pushTurn);
        Serial.printf("[screen] band push refused: %s (x %d y %d w %d rows %d)\n", esp_err_to_name(err), x, y, w, rows);
        return false;
    }
    g_pushing = true;
    return true;
}

const char *pushTurnHolder() {
    TaskHandle_t holder = g_pushTurn != nullptr ? xSemaphoreGetMutexHolder(g_pushTurn) : nullptr;
    return holder != nullptr ? pcTaskGetName(holder) : "";
}

void pushStats(uint32_t &busyUs, uint32_t &count) {
    busyUs = g_pushBusyUs;
    count = g_pushCount;
    g_pushBusyUs = 0;
    g_pushCount = 0;
}

PpaPerf takePpaPerf() {
    PpaPerf was = g_perf;
    g_perf = PpaPerf();
    return was;
}

// The band given up on, if it doesn't come in time: the PPA with it, and the
// next frame's bands pushed by the CPU.
bool pushBandWait() {
    if (!g_pushing) {
        return true;
    }
    bool finished = xSemaphoreTake(g_pushDone, pdMS_TO_TICKS(PPA_WAIT_MS)) == pdTRUE;
    if (finished) {
        noteDone();
    }
    g_pushing = false;
    xSemaphoreGive(g_pushTurn);
    if (!finished) {
        ppaGiveUp("band push");
    }
    return finished;
}

namespace {

// See pushStrips(): the strips' height - the 3D view's deepest band - and
// the narrowest half worth pushing that way, where the transfers' own 85us
// each begin to eat what the internal RAM saves. And the internal RAM left
// free with both strips taken, for the WiFi's buffers meanwhile: TLS's go to
// the PSRAM (main.cpp's tlsCalloc()).
constexpr int STRIP_ROWS = 16, STRIP_MIN_W = 200;
constexpr size_t STRIP_KEEP_FREE = 64 * 1024;

size_t stripBytes(int w) { return ((size_t)w * STRIP_ROWS * 2 + 63) & ~(size_t)63; }

}  // namespace

bool pushStrips(int x, int y, int w, int h, StripPerf *perf, int widest) {
    int64_t t0 = esp_timer_get_time();
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    w = std::min(w, CANVAS_W - x);
    h = std::min(h, CANVAS_H - y);
    if (!g_ppaOk || g_ppaStuck || g_liveW > 0 || g_buffer == nullptr || w < STRIP_MIN_W || h < STRIP_ROWS) {
        return false;
    }
    StripPerf p;
    p.freeBefore = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    p.largestBefore = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (perf != nullptr) {
        *perf = p;  // what there was, should they not be had
    }
    // As wide as the rect if two strips that wide can be had with enough
    // left; else halves; else not at all.
    int stripW = widest > 0 ? std::min(w, widest) : w;
    uint16_t *strip[2] = {nullptr, nullptr};
    while (true) {
        size_t bytes = stripBytes(stripW);
        if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) >= 2 * bytes + STRIP_KEEP_FREE &&
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) >= bytes) {
            for (uint16_t *&s : strip) {
                s = (uint16_t *)heap_caps_aligned_alloc(64, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            }
            if (strip[0] != nullptr && strip[1] != nullptr) {
                break;
            }
            for (uint16_t *&s : strip) {
                heap_caps_free(s);
                s = nullptr;
            }
        }
        int half = (w + 1) / 2;
        if (stripW <= half || half < STRIP_MIN_W) {
            return false;
        }
        stripW = half;
    }
    int64_t t1 = esp_timer_get_time();
    p.allocUs = (uint32_t)(t1 - t0);
    p.stripW = stripW;

    // Strip i into one buffer while strip i - 1 goes from the other; i + 1
    // into that one only once its push is done.
    int across = (w + stripW - 1) / stripW, slabs = (h + STRIP_ROWS - 1) / STRIP_ROWS;
    bool ok = true, pushing = false;
    for (int i = 0; i < slabs * across && ok; i++) {
        int sy = std::min(y + i / across * STRIP_ROWS, y + h - STRIP_ROWS);
        int sx = x + i % across * stripW, sw = std::min(stripW, x + w - sx);
        uint16_t *to = strip[i & 1];
        int64_t c0 = esp_timer_get_time();
        for (int r = 0; r < STRIP_ROWS; r++) {
            memcpy(to + (size_t)r * sw, g_buffer + (size_t)(sy + r) * CANVAS_W + sx, (size_t)sw * 2);
        }
        int64_t c1 = esp_timer_get_time();
        p.copyUs += (uint32_t)(c1 - c0);
        if (pushing) {
            ok = pushBandWait();
            pushing = false;
            p.waitUs += (uint32_t)(esp_timer_get_time() - c1);
        }
        if (ok) {
            ok = pushBandStart(to, sx, sy, sw, STRIP_ROWS);
            pushing = ok;
            p.strips += ok;
        }
    }
    if (pushing) {
        int64_t w0 = esp_timer_get_time();
        ok = pushBandWait();
        p.waitUs += (uint32_t)(esp_timer_get_time() - w0);
    }
    for (uint16_t *s : strip) {
        heap_caps_free(s);
    }
    p.totalUs = (uint32_t)(esp_timer_get_time() - t0);
    if (perf != nullptr) {
        *perf = p;
    }
    return ok;
}

#ifdef BENCH3D
void benchGiveUpPpa() {
    Serial.println("[screen] the PPA given up, for the bench");
    g_ppaStuck = true;
}

bool pushBand(const uint16_t *pixels, int x, int y, int w, int rows, int client) {
    (void)client;
    return pushBandStart(pixels, x, y, w, rows) && pushBandWait();
}

void benchReadPanel(int x, int y, int w, int h, uint16_t *out) {
    // What the PPA wrote, not what the cache last held of it.
    esp_cache_msync(g_fb, g_fbBytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    const uint16_t *fb = (const uint16_t *)g_fb;
    bool ninety = g_angle == PPA_SRM_ROTATION_ANGLE_90;
    for (int cy = y; cy < y + h; cy++) {
        int fx = ninety ? cy : CANVAS_H - 1 - cy;
        for (int cx = x; cx < x + w; cx++) {
            int fy = ninety ? CANVAS_W - 1 - cx : cx;
            out[(size_t)(cy - y) * w + (cx - x)] = __builtin_bswap16(fb[(size_t)fy * g_panelW + fx]);
        }
    }
}
#endif

bool init() {
    g_pushTurn = xSemaphoreCreateMutex();
    g_pushDone = xSemaphoreCreateBinary();
    g_fillDone = xSemaphoreCreateBinary();
    auto panel = (lgfx::Panel_DSI *)M5.Display.getPanel();
    g_fb = panel->config_detail().buffer;
    g_panelW = panel->config().panel_width;
    g_panelH = panel->config().panel_height;
    g_fbBytes = (size_t)g_panelW * g_panelH * 2;

    // 64-byte aligned because the PPA's DMA works in cache lines.
    g_buffer = (uint16_t *)heap_caps_aligned_alloc(64, CANVAS_BYTES, MALLOC_CAP_SPIRAM);
    if (g_buffer == nullptr) {
        Serial.println("[screen] canvas allocation failed");
        return false;
    }
    g_canvas.setColorDepth(16);
    g_canvas.setBuffer(g_buffer, CANVAS_W, CANVAS_H, 16);

    if (g_fb != nullptr) {
        ppa_client_config_t srmCfg = {};
        srmCfg.oper_type = PPA_OPERATION_SRM;
        srmCfg.max_pending_trans_num = 1;  // one at a time, the turn taken
        ppa_client_config_t fillCfg = {};
        fillCfg.oper_type = PPA_OPERATION_FILL;
        fillCfg.max_pending_trans_num = 1;
        ppa_event_callbacks_t pushed = {}, filled = {};
        pushed.on_trans_done = pushDone;
        filled.on_trans_done = fillDone;
        g_ppaOk = g_pushTurn != nullptr && g_pushDone != nullptr && g_fillDone != nullptr &&
                  ppa_register_client(&srmCfg, &g_srm) == ESP_OK &&
                  ppa_client_register_event_callbacks(g_srm, &pushed) == ESP_OK &&
                  ppa_register_client(&fillCfg, &g_fillClient) == ESP_OK &&
                  ppa_client_register_event_callbacks(g_fillClient, &filled) == ESP_OK;
    }
    if (!g_ppaOk) {
        Serial.println("[screen] PPA unavailable - falling back to CPU pushSprite");
    } else {
        // M5.begin() drew its splash through the CPU; flush those writes before
        // the PPA starts writing the same framebuffer behind the cache's back.
        esp_cache_msync(g_fb, g_fbBytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
    return true;
}

void setRotation(int lgfxRotation) {
    g_angle = (lgfxRotation == 1) ? PPA_SRM_ROTATION_ANGLE_270 : PPA_SRM_ROTATION_ANGLE_90;
    markAllDirty();
}

void fillRect(int x, int y, int w, int h, uint16_t color) {
    if (!g_ppaOk || g_ppaStuck) {
        g_canvas.fillRect(x, y, w, h, color);
        markDirty(x, y, w, h);
        return;
    }
    // The PPA fills RGB565 in the same swapped byte order it reads it in, and
    // the fill operation has no byte_swap flag - so hand it the bytes already
    // swapped, as an ARGB8888 value it will truncate back to exactly this.
    uint16_t v = __builtin_bswap16(color);
    uint32_t argb = 0xFF000000 | (uint32_t)(((v >> 11) & 0x1F) << 3) << 16 |
                    (uint32_t)(((v >> 5) & 0x3F) << 2) << 8 | (uint32_t)((v & 0x1F) << 3);

    ppa_fill_oper_config_t op = {};
    op.out.buffer = g_buffer;
    op.out.buffer_size = CANVAS_BYTES;
    op.out.pic_w = CANVAS_W;
    op.out.pic_h = CANVAS_H;
    op.out.block_offset_x = x;
    op.out.block_offset_y = y;
    op.out.fill_cm = PPA_FILL_COLOR_MODE_RGB565;
    op.fill_block_w = w;
    op.fill_block_h = h;
    op.fill_argb_color.val = argb;
    op.mode = PPA_TRANS_MODE_NON_BLOCKING;
    op.user_data = g_fillDone;

    // Whole rows either side of the DMA, which is what a canvas row being a
    // whole number of cache lines allows: written back first so no stale CPU
    // line lands on top of the fill later, then dropped so the CPU reads the
    // fill rather than what it had cached. The write-back is what makes the
    // drop safe for the parts of those rows outside the block.
    uint8_t *rows = (uint8_t *)g_buffer + (size_t)y * ROW_BYTES;
    size_t rowsBytes = (size_t)h * ROW_BYTES;
    int64_t t0 = esp_timer_get_time();
    esp_cache_msync(rows, rowsBytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    int64_t t1 = esp_timer_get_time();
    g_perf.syncUs += (uint64_t)(t1 - t0);
    bool filled = false;
    if (xSemaphoreTake(g_pushTurn, pdMS_TO_TICKS(PPA_TURN_MS)) == pdTRUE) {
        noteSent('f', x, y, w, h);
        filled = ppa_do_fill(g_fillClient, &op) == ESP_OK;
        if (filled && xSemaphoreTake(g_fillDone, pdMS_TO_TICKS(PPA_WAIT_MS)) != pdTRUE) {
            filled = false;
            ppaGiveUp("fill");
        } else if (filled) {
            noteDone();
        }
        xSemaphoreGive(g_pushTurn);
    } else {
        ppaGiveUp("band push, holding the turn,");
    }
    if (!filled) {
        g_canvas.fillRect(x, y, w, h, color);
    } else {
        int64_t t2 = esp_timer_get_time();
        g_perf.fills++;
        g_perf.fillUs += (uint64_t)(t2 - t1);
        esp_cache_msync(rows, rowsBytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        g_perf.syncUs += (uint64_t)(esp_timer_get_time() - t2);
    }
    markDirty(x, y, w, h);
}

void clear(uint16_t color) {
    fillRect(0, 0, CANVAS_W, CANVAS_H, color);
    markAllDirty();  // one rect for the lot, rather than one on top of the list
}

void markAllDirty() {
    g_rectCount = 1;
    g_rects[0] = {0, 0, CANVAS_W, CANVAS_H};
    g_changedCount = 1;
    g_changed[0] = g_rects[0];
    g_touched = true;
}

void markDirty(int x, int y, int w, int h) {
    if (w <= 0 || h <= 0) {
        return;
    }
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > CANVAS_W) { w = CANVAS_W - x; }
    if (y + h > CANVAS_H) { h = CANVAS_H - y; }
    if (w <= 0 || h <= 0) {
        return;
    }
    addRect(g_rects, g_rectCount, x, y, w, h, true);
    addRect(g_changed, g_changedCount, x, y, w, h, false);
    noteDrawn({(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h});
}

void watch(int x, int y, int w, int h) {
    g_watch = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
    g_touched = false;
}

bool touched() { return g_touched; }

int takeChanged(Region *out, int max) {
    int n = std::min(g_changedCount, max);
    for (int i = 0; i < n; i++) {
        out[i] = {g_changed[i].x, g_changed[i].y, g_changed[i].w, g_changed[i].h};
    }
    g_changedCount = 0;
    return n;
}

void setOverlay(OverlayFn draw, int x, int y, int w, int h) {
    g_overlay = draw;
    g_overlayRect = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
}

void flush() {
    if (g_rectCount == 0 || g_buffer == nullptr) {
        return;
    }
    // Repainted only when something underneath has been drawn over it, so a
    // flush elsewhere on the screen doesn't pay for pushing it again.
    if (g_overlay != nullptr) {
        bool touched = false;
        for (int i = 0; i < g_rectCount && !touched; i++) {
            touched = intersects(g_rects[i], g_overlayRect);
        }
        if (touched) {
            // The overlay draws on the shared canvas, so the font and text
            // style it leaves behind would carry on into whatever is drawn
            // next - and a screen that sets a size but trusts the font it finds
            // would draw in the banner's. Put back as they were.
            const lgfx::IFont *font = g_canvas.getFont();
            lgfx::TextStyle style = g_canvas.getTextStyle();
            bool drawn = g_overlay(g_canvas);
            g_canvas.setFont(font);
            g_canvas.setTextStyle(style);
            if (drawn) {
                markDirty(g_overlayRect.x, g_overlayRect.y, g_overlayRect.w, g_overlayRect.h);
            }
        }
    }
#ifdef RENDER_PROFILE
    uint32_t t0 = micros();
    int rects = g_rectCount;
    g_syncedBytes = 0;
#endif

    if (!g_ppaOk) {
        for (int i = 0; i < g_rectCount; i++) {
            blitRectFallback(g_rects[i]);
        }
        g_rectCount = 0;
        return;
    }

    writeBackDirty();

    // Each transfer costs roughly its own area plus a fixed overhead, so once
    // the separate rects cover most of their common bounding box it's quicker
    // to send the box once. (Fifteen row-sized transfers measure ~56ms; the
    // whole screen in one go is ~42ms.)
    int l = g_rects[0].x, t = g_rects[0].y, rgt = l + g_rects[0].w, bot = t + g_rects[0].h;
    uint32_t sum = (uint32_t)g_rects[0].w * g_rects[0].h;
    for (int i = 1; i < g_rectCount; i++) {
        l = std::min<int>(l, g_rects[i].x);
        t = std::min<int>(t, g_rects[i].y);
        rgt = std::max<int>(rgt, g_rects[i].x + g_rects[i].w);
        bot = std::max<int>(bot, g_rects[i].y + g_rects[i].h);
        sum += (uint32_t)g_rects[i].w * g_rects[i].h;
    }
    uint32_t boxArea = (uint32_t)(rgt - l) * (bot - t);

    if (g_rectCount > 1 && sum * 10 >= boxArea * 6) {
        Rect box = {(int16_t)l, (int16_t)t, (int16_t)(rgt - l), (int16_t)(bot - t)};
        blitRect(box);
    } else {
        for (int i = 0; i < g_rectCount; i++) {
            blitRect(g_rects[i]);
        }
    }
    g_rectCount = 0;

#ifdef RENDER_PROFILE
    Serial.printf("[render] flush %d rect(s), %uKB synced, in %.2f ms\n", rects,
                  (unsigned)(g_syncedBytes / 1024), (micros() - t0) / 1000.0);
#endif
}

void pushRects(const Region *rects, int n) {
    if (g_buffer == nullptr || n <= 0) {
        return;
    }
    if (!g_ppaOk) {
        for (int i = 0; i < n; i++) {
            markDirty(rects[i].x, rects[i].y, rects[i].w, rects[i].h);
        }
        flush();
        return;
    }
    // Any reaching under the overlay - which has to be painted over them
    // again - the ordinary way, with all its care: the overlay is always set,
    // showing or not, so it is only those boxes that count.
    bool marked = false;
    int top = CANVAS_H, bottom = 0;
    for (int i = 0; i < n; i++) {
        if (g_overlay != nullptr && intersects({rects[i].x, rects[i].y, rects[i].w, rects[i].h}, g_overlayRect)) {
            markDirty(rects[i].x, rects[i].y, rects[i].w, rects[i].h);
            marked = true;
            continue;
        }
        top = std::min<int>(top, std::max<int>(0, rects[i].y));
        bottom = std::max<int>(bottom, std::min<int>(CANVAS_H, rects[i].y + rects[i].h));
    }
    if (marked) {
        flush();
    }
    if (bottom <= top) {
        return;
    }
    // The rest written back in one go, every row any of them takes in: what
    // that costs is the lines that are dirty, which are the ones that have to
    // go anyway, and some 10us besides - where a call a row, a box at a time,
    // comes to about 3us a row.
    int64_t t0 = esp_timer_get_time();
    esp_cache_msync((uint8_t *)g_buffer + (size_t)top * ROW_BYTES, (size_t)(bottom - top) * ROW_BYTES,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    g_perf.syncUs += (uint64_t)(esp_timer_get_time() - t0);
    for (int i = 0; i < n; i++) {
        int x = std::max<int>(0, rects[i].x), y = std::max<int>(0, rects[i].y);
        int r = std::min<int>(CANVAS_W, rects[i].x + rects[i].w), b = std::min<int>(CANVAS_H, rects[i].y + rects[i].h);
        if (r <= x || b <= y ||
            (g_overlay != nullptr && intersects({rects[i].x, rects[i].y, rects[i].w, rects[i].h}, g_overlayRect))) {
            continue;
        }
        blitRect({(int16_t)x, (int16_t)y, (int16_t)(r - x), (int16_t)(b - y)});
        addRect(g_changed, g_changedCount, x, y, r - x, b - y, false);
        noteDrawn({(int16_t)x, (int16_t)y, (int16_t)(r - x), (int16_t)(b - y)});
    }
}

namespace {

bool convertBlock(uint8_t *out, size_t outBytes, int x, int y, int w, int h) {
    ppa_srm_oper_config_t op = {};
    op.in.buffer = g_buffer;
    op.in.pic_w = CANVAS_W;
    op.in.pic_h = CANVAS_H;
    op.in.block_w = w;
    op.in.block_h = h;
    op.in.block_offset_x = x;
    op.in.block_offset_y = y;
    op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    op.out.buffer = out;
    op.out.buffer_size = outBytes;
    op.out.pic_w = CANVAS_W;
    op.out.pic_h = CANVAS_H;
    op.out.block_offset_x = x;
    op.out.block_offset_y = y;
    op.out.srm_cm = PPA_SRM_COLOR_MODE_YUV420;
    op.out.yuv_range = PPA_COLOR_RANGE_LIMIT;
    op.out.yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601;
    op.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
    op.scale_x = 1.0f;
    op.scale_y = 1.0f;
    op.byte_swap = true;  // the canvas's byte order, as for blitRect()
    // On the screen's own client, so as the pushes are: in its turn, its end
    // given by pushDone() - which every transfer on it calls, a blocking one
    // too, and given no semaphore to give it asserted, the export's first
    // frame resetting the Tab5.
    op.mode = PPA_TRANS_MODE_NON_BLOCKING;
    op.user_data = g_pushDone;
    if (g_ppaStuck || xSemaphoreTake(g_pushTurn, pdMS_TO_TICKS(PPA_TURN_MS)) != pdTRUE) {
        Serial.println("[screen] PPA YUV conversion: the PPA is stuck or busy");
        return false;
    }
    g_pushStartUs = esp_timer_get_time();
    noteSent('y', x, y, w, h);
    esp_err_t err = ppa_do_scale_rotate_mirror(g_srm, &op);
    bool finished = err == ESP_OK && xSemaphoreTake(g_pushDone, pdMS_TO_TICKS(PPA_WAIT_MS)) == pdTRUE;
    if (finished) {
        noteDone();
    }
    xSemaphoreGive(g_pushTurn);
    if (err != ESP_OK) {
        Serial.printf("[screen] PPA YUV conversion failed: %s\n", esp_err_to_name(err));
        return false;
    }
    if (!finished) {
        ppaGiveUp("YUV conversion");
        return false;
    }
    return true;
}

}  // namespace

bool toYuv420(uint8_t *out, size_t outBytes, const Region &r) {
    if (!g_ppaOk || g_buffer == nullptr || outBytes < YUV420_BYTES) {
        return false;
    }
    int x0 = std::max(0, (int)r.x) & ~1, y0 = std::max(0, (int)r.y) & ~1;
    int x1 = std::min(CANVAS_W, (r.x + r.w + 1) & ~1), y1 = std::min(CANVAS_H, (r.y + r.h + 1) & ~1);
    if (x1 <= x0 || y1 <= y0) {
        return true;
    }
    // Whole rows, as for a flush: all that the PPA will read has to be in PSRAM.
    esp_cache_msync((uint8_t *)g_buffer + (size_t)y0 * ROW_BYTES, (size_t)(y1 - y0) * ROW_BYTES,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    return convertBlock(out, outBytes, x0, y0, x1 - x0, y1 - y0);
}

bool toYuv420(uint8_t *out, size_t outBytes) {
    if (!g_ppaOk || g_buffer == nullptr || outBytes < YUV420_BYTES) {
        return false;
    }
    // The whole canvas is read, not just what is dirty, so all of it has to
    // be in PSRAM - including regions drawn since the last flush.
    writeBackCanvas();
    return convertBlock(out, outBytes, 0, 0, CANVAS_W, CANVAS_H);
}

void setLiveRect(int x, int y, int w, int h) {
    g_liveX = x, g_liveY = y, g_liveW = w, g_liveH = h;
}

void dumpToSerial() {
    if (g_buffer == nullptr) {
        return;
    }
    if (g_liveW > 0 && g_fb != nullptr) {
        // A row at a time, the live part of it from the framebuffer: turned
        // back from the panel's portrait as blitRect() turns a pixel into it,
        // and in the canvas's byte order again, the PPA having swapped it.
        Serial.setTxTimeoutMs(SERIAL_TX_TIMEOUT_DUMP_MS);
        Serial.flush();
        Serial.printf("\nSCREENSHOT %d %d\n", CANVAS_W, CANVAS_H);
        esp_cache_msync(g_fb, g_fbBytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        static uint16_t row[CANVAS_W];
        const uint16_t *fb = (const uint16_t *)g_fb;
        for (int y = 0; y < CANVAS_H; y++) {
            memcpy(row, g_buffer + (size_t)y * CANVAS_W, sizeof row);
            if (y >= g_liveY && y < g_liveY + g_liveH) {
                for (int x = g_liveX; x < g_liveX + g_liveW; x++) {
                    int fx = g_angle == PPA_SRM_ROTATION_ANGLE_90 ? y : CANVAS_H - 1 - y;
                    int fy = g_angle == PPA_SRM_ROTATION_ANGLE_90 ? CANVAS_W - 1 - x : x;
                    row[x] = __builtin_bswap16(fb[(size_t)fy * g_panelW + fx]);
                }
            }
            const uint8_t *p = (const uint8_t *)row;
            size_t left = sizeof row;
            while (left > 0) {
                size_t n = Serial.write(p, std::min<size_t>(left, 4096));
                p += n;
                left -= n;
                if (n == 0) {
                    delay(1);
                }
            }
        }
        Serial.flush();
        Serial.print("\nSCREENSHOT END\n");
        Serial.setTxTimeoutMs(SERIAL_TX_TIMEOUT_MS);
        return;
    }
    // Waited on as long as it takes: the tool is reading, and a write given up
    // on would leave a hole in the picture.
    Serial.setTxTimeoutMs(SERIAL_TX_TIMEOUT_DUMP_MS);
    Serial.flush();
    Serial.printf("\nSCREENSHOT %d %d\n", CANVAS_W, CANVAS_H);
    // In chunks: a single 1.8MB write can outlast the CDC driver's transmit
    // timeout while the host catches up, and the remainder would be dropped.
    const uint8_t *p = (const uint8_t *)g_buffer;
    size_t left = CANVAS_BYTES;
    while (left > 0) {
        size_t n = Serial.write(p, std::min<size_t>(left, 4096));
        p += n;
        left -= n;
        if (n == 0) {
            delay(1);
        }
    }
    Serial.flush();
    Serial.print("\nSCREENSHOT END\n");
    Serial.setTxTimeoutMs(SERIAL_TX_TIMEOUT_MS);
}

}  // namespace screen
