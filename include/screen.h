#pragma once

#include <M5Unified.h>

// One shared landscape (1280x720) off-screen canvas that every screen draws
// into, pushed to the panel by the ESP32-P4's PPA (its 2D graphics
// accelerator) rather than by the CPU.
//
// Why: the Tab5's panel framebuffer is physically portrait (720x1280), so in
// the landscape orientation this app runs at, every horizontal line of the UI
// is a *column* in memory. LovyanGFX's rotated pushSprite therefore can't
// memcpy - it walks the image pixel by pixel. Measured on this device, a
// full-screen push costs 650ms that way; the PPA does the same rotation in
// 42ms, a single table row in under 4ms, and one table cell in 0.6ms.
//
// So: draw unrotated (cheap - straight memcpy inside the canvas), mark what
// changed, and let the hardware rotate just those regions onto the panel.
namespace screen {

// Call once after M5.begin(). Returns false only if the canvas can't be
// allocated; a missing PPA falls back to LovyanGFX's own push.
bool init();

// The canvas every screen draws into. Always landscape, whatever the rotation.
M5Canvas &canvas();

// 3 = normal, 1 = device flipped end-for-end. Mirrors main.cpp's g_rotation.
void setRotation(int lgfxRotation);

// Hardware fill of the whole canvas (~5ms, vs ~42ms for canvas().fillScreen()).
void clear(uint16_t color);

// Hardware fill of part of it, marked dirty. Worth it for a large area: the
// radar's plot square fills in a fraction of the 15ms canvas().fillRect()
// takes, the CPU being slow to write that much into PSRAM.
void fillRect(int x, int y, int w, int h, uint16_t color);

void markDirty(int x, int y, int w, int h);
void markAllDirty();

// A part of the canvas that a screen keeps from one frame to the next and
// draws again only when what it shows changes - the follow telemetry - and
// that has to be drawn again too once anything else has been drawn over it:
// a full redraw, a card. watch() starts watching (x, y, w, h), once it has
// been drawn; touched() says whether anything has been marked dirty over it
// since, or pushed by pushRects(), which every canvas write is.
void watch(int x, int y, int w, int h);
bool touched();

// Push everything marked dirty to the panel, then start a fresh dirty list.
void flush();

// Something drawn over whichever screen is up - the alert banner. `draw`
// paints it into the canvas and returns true, or returns false when it isn't
// to be shown on the screen that is up. It is repainted at any flush whose
// dirty regions touch (x, y, w, h), so a screen redrawing underneath can't
// wipe it; to show it the first time, mark that region dirty and flush.
// Taking it down is the caller's business: the screen under it has to be
// repainted, since the overlay was drawn over its pixels.
using OverlayFn = bool (*)(M5Canvas &canvas);
void setOverlay(OverlayFn draw, int x, int y, int w, int h);

// The whole canvas, converted by the PPA to YUV 4:2:0 in the packed layout
// the P4's H.264 encoder takes (odd lines U Y Y, even lines V Y Y), for
// encoding as a video frame. BT.601, limited range - what a decoder assumes
// of a stream that doesn't say. `out` must be 64-byte aligned and hold
// 1280 * 720 * 3 / 2 bytes. False if the PPA isn't available or refused.
constexpr size_t YUV420_BYTES = 1280 * 720 * 3 / 2;
bool toYuv420(uint8_t *out, size_t outBytes);

// For an encoder that keeps its last frame and converts only what has changed
// since: every region marked dirty since the last call, whether or not it has
// been flushed. The list collapses to a bounding box when it overflows, so it
// may say more than changed, never less.
struct Region {
    int16_t x, y, w, h;
};
constexpr int MAX_REGIONS = 16;
int takeChanged(Region *out, int max);
// As toYuv420(), for just one region of the canvas into the same place in
// `out`. Widened to even edges, which YUV 4:2:0's shared colour needs.
bool toYuv420(uint8_t *out, size_t outBytes, const Region &r);

// Regions pushed to the panel now, each as it is, outside the dirty list -
// for a plot that knows the few small places it changed, which flush()'s
// sixteen merged regions would have made one box of nearly the whole plot.
// A transfer is 0.085ms and 53ns a pixel, measured: twenty small boxes cost a
// fifth of the plot pushed whole. Kept for the encoder, as markDirty() would.
void pushRects(const Region *rects, int n);

// A band `w` pixels wide and `rows` tall, from `pixels` - one row after
// another, anywhere a DMA can read, 64-byte aligned - straight to the panel
// with its top-left at (x, y), past the canvas: the push started, and then -
// before `pixels` is touched again, or another push started - waited for, for
// the caller to draw the next band while this one goes. One push at a time,
// and the canvas's flushes waiting their turn behind one under way.
bool pushBandStart(const uint16_t *pixels, int x, int y, int w, int rows);
bool pushBandWait();
// The task holding the turn at the PPA - a band's push under way, or a
// blit - for a hang's report: "" if none.
const char *pushTurnHolder();
// The time the pushes since the last call took, start to finish, and how
// many there were.
void pushStats(uint32_t &busyUs, uint32_t &count);
// The canvas's own work at the PPA since the last call, for the [perf] line -
// apart from pushStats(), which the 3D bench reads and resets: its blits to
// the panel and its fills, how many and how long the CPU waited on them, the
// pixels the blits came to, and the cache written back for them.
struct PpaPerf {
    uint32_t blits = 0, fills = 0;
    uint64_t blitUs = 0, fillUs = 0, blitPixels = 0, syncUs = 0;
};
PpaPerf takePpaPerf();

// A big rect of the canvas - the plot's column, the whole screen - pushed
// through the chip's own memory rather than read by the PPA from the PSRAM:
// copied 16 rows at a time into one of two strips there, and each pushed as
// the 3D view's bands are (pushBandStart()) while the next is copied. The PPA
// then reads 36-byte pieces from internal RAM, and only its writes go to
// the PSRAM - about 20ns a pixel where blitRect()'s is 46, measured on the
// 3D view's bands. A height that isn't a whole number of strips has its last
// strip 16 rows up from the bottom, a few rows pushed twice.
//
// The strips are taken for the push and given back after it, and only with
// enough of that memory left for the WiFi: two 40KB blocks for the whole
// width, or failing that two halves 640 pixels wide; `widest` caps them, 0
// for as wide as the rect. False - nothing pushed - when they can't be had,
// the rect is under 16 rows or a half strip under 200 pixels, the PPA isn't
// there or has been given up, or the 3D view is open; or should the PPA
// refuse a strip partway, the rest left unpushed. For the caller to push it
// with blitRect()'s way then. The canvas's cache is left as it was: its
// rows still to be written back by the next flush, as ever.
//
// For the bench to measure, for now - "##=X" - before flush() takes it up.
struct StripPerf {
    int strips = 0, stripW = 0;  // how many pushed, and the widest
    uint32_t allocUs = 0, copyUs = 0, waitUs = 0, totalUs = 0;
    size_t freeBefore = 0, largestBefore = 0;  // internal RAM's, as they were
};
bool pushStrips(int x, int y, int w, int h, StripPerf *perf = nullptr, int widest = 0);

#ifdef BENCH3D
// The PPA given up, as though a transfer had never finished - for the bench
// to try the CPU's pushes in its place.
void benchGiveUpPpa();
// Both halves in one, for the bench: `client` 0 or 1, by the core.
bool pushBand(const uint16_t *pixels, int x, int y, int w, int rows, int client);
// The panel's pixels for (x, y, w, h), read back from its framebuffer and
// turned to the canvas's way up and byte order, as dumpToSerial() reads them:
// into `out`, w * h of them, a row after another. For the bench to set one
// way of pushing against another.
void benchReadPanel(int x, int y, int w, int h, uint16_t *out);
#endif

// A part of the screen - x, y, w, h - whose pixels the panel holds rather
// than the canvas, as the 3D view's are, for a screenshot to take from the
// panel instead; w 0 for none.
void setLiveRect(int x, int y, int w, int h);

// Writes the canvas to Serial as a screenshot, for tools/screenshot.py: a
// header line, then the raw 1280x720 RGB565 pixels as the canvas holds them.
// The canvas is landscape whatever the panel's rotation, so this is exactly
// what is on screen, the right way up.
void dumpToSerial();

}  // namespace screen
