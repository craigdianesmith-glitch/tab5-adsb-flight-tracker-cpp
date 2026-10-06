#include "screen.h"

#include "config.h"

#include <driver/ppa.h>
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

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
bool g_ppaOk = false;
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
}

// Deliberately blocking, one transfer at a time. Queueing these as
// PPA_TRANS_MODE_NON_BLOCKING and waiting only on the last was measured
// *slower* - 16.8us per KB against 12.5 - because the CPU was never what the
// transfers were waiting on, and the queue and completion interrupt cost more
// than the round trip they replaced.
void blitRect(const Rect &r) {
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
    op.mode = PPA_TRANS_MODE_BLOCKING;

    esp_err_t err = ppa_do_scale_rotate_mirror(g_srm, &op);
    if (err != ESP_OK) {
        // Worth knowing about: a full queue would otherwise drop the transfer
        // and leave that region of the panel showing the previous frame.
        Serial.printf("[screen] PPA blit failed: %s\n", esp_err_to_name(err));
    }
}

// Fallback for a board where the PPA didn't come up: LovyanGFX's own push,
// clipped to the dirty region so it at least isn't pushing the whole screen.
void blitRectFallback(const Rect &r) {
    M5.Display.setClipRect(r.x, r.y, r.w, r.h);
    g_canvas.pushSprite(0, 0);
    M5.Display.clearClipRect();
}

}  // namespace

M5Canvas &canvas() { return g_canvas; }

bool init() {
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
        srmCfg.max_pending_trans_num = 1;  // sufficient while every transfer is blocking
        ppa_client_config_t fillCfg = {};
        fillCfg.oper_type = PPA_OPERATION_FILL;
        fillCfg.max_pending_trans_num = 1;
        g_ppaOk = (ppa_register_client(&srmCfg, &g_srm) == ESP_OK) &&
                  (ppa_register_client(&fillCfg, &g_fillClient) == ESP_OK);
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
    if (!g_ppaOk) {
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
    op.mode = PPA_TRANS_MODE_BLOCKING;

    // Whole rows either side of the DMA, which is what a canvas row being a
    // whole number of cache lines allows: written back first so no stale CPU
    // line lands on top of the fill later, then dropped so the CPU reads the
    // fill rather than what it had cached. The write-back is what makes the
    // drop safe for the parts of those rows outside the block.
    uint8_t *rows = (uint8_t *)g_buffer + (size_t)y * ROW_BYTES;
    size_t rowsBytes = (size_t)h * ROW_BYTES;
    esp_cache_msync(rows, rowsBytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    if (ppa_do_fill(g_fillClient, &op) != ESP_OK) {
        g_canvas.fillRect(x, y, w, h, color);
    } else {
        esp_cache_msync(rows, rowsBytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
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
}

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
    op.mode = PPA_TRANS_MODE_BLOCKING;
    esp_err_t err = ppa_do_scale_rotate_mirror(g_srm, &op);
    if (err != ESP_OK) {
        Serial.printf("[screen] PPA YUV conversion failed: %s\n", esp_err_to_name(err));
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

void dumpToSerial() {
    if (g_buffer == nullptr) {
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
