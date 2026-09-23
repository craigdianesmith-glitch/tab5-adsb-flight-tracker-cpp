#include "screen.h"

#include <driver/ppa.h>
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>

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

// Cells in the same row share a y/h, so merging those into one wide rect keeps
// the rect count at one per row instead of one per cell.
constexpr int MERGE_GAP = 96;

void writeBackCanvas() {
    // The PPA reads the canvas by DMA, so the CPU's dirty cache lines have to
    // be in PSRAM first.
    esp_cache_msync(g_buffer, CANVAS_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

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

    ppa_do_scale_rotate_mirror(g_srm, &op);
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
        srmCfg.max_pending_trans_num = 1;
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

void clear(uint16_t color) {
    if (!g_ppaOk) {
        g_canvas.fillScreen(color);
        markAllDirty();
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
    op.out.fill_cm = PPA_FILL_COLOR_MODE_RGB565;
    op.fill_block_w = CANVAS_W;
    op.fill_block_h = CANVAS_H;
    op.fill_argb_color.val = argb;
    op.mode = PPA_TRANS_MODE_BLOCKING;

    writeBackCanvas();  // don't let stale CPU lines land on top of the fill
    if (ppa_do_fill(g_fillClient, &op) != ESP_OK) {
        g_canvas.fillScreen(color);
    } else {
        esp_cache_msync(g_buffer, CANVAS_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    }
    markAllDirty();
}

void markAllDirty() {
    g_rectCount = 1;
    g_rects[0] = {0, 0, CANVAS_W, CANVAS_H};
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

    for (int i = 0; i < g_rectCount; i++) {
        Rect &r = g_rects[i];
        if (x >= r.x && y >= r.y && x + w <= r.x + r.w && y + h <= r.y + r.h) {
            return;  // already covered
        }
        // same band, near enough horizontally: widen it instead of adding one
        if (y >= r.y && y + h <= r.y + r.h && x <= r.x + r.w + MERGE_GAP && x + w + MERGE_GAP >= r.x) {
            int right = std::max<int>(r.x + r.w, x + w);
            r.x = std::min<int>(r.x, x);
            r.w = right - r.x;
            return;
        }
    }

    if (g_rectCount == MAX_RECTS) {
        // Out of slots: collapse everything into one bounding box. Cheaper than
        // paying the per-transfer overhead many times over.
        int l = g_rects[0].x, t = g_rects[0].y, rgt = l + g_rects[0].w, bot = t + g_rects[0].h;
        for (int i = 1; i < g_rectCount; i++) {
            l = std::min<int>(l, g_rects[i].x);
            t = std::min<int>(t, g_rects[i].y);
            rgt = std::max<int>(rgt, g_rects[i].x + g_rects[i].w);
            bot = std::max<int>(bot, g_rects[i].y + g_rects[i].h);
        }
        l = std::min(l, x);
        t = std::min(t, y);
        rgt = std::max(rgt, x + w);
        bot = std::max(bot, y + h);
        g_rectCount = 1;
        g_rects[0] = {(int16_t)l, (int16_t)t, (int16_t)(rgt - l), (int16_t)(bot - t)};
        return;
    }
    g_rects[g_rectCount++] = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
}

void flush() {
    if (g_rectCount == 0 || g_buffer == nullptr) {
        return;
    }
#ifdef RENDER_PROFILE
    uint32_t t0 = micros();
    int rects = g_rectCount;
#endif

    if (!g_ppaOk) {
        for (int i = 0; i < g_rectCount; i++) {
            blitRectFallback(g_rects[i]);
        }
        g_rectCount = 0;
        return;
    }

    writeBackCanvas();

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
    Serial.printf("[render] flush %d rect(s) in %.2f ms\n", rects, (micros() - t0) / 1000.0);
#endif
}

}  // namespace screen
