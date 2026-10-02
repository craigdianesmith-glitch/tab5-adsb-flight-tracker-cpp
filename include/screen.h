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

// Writes the canvas to Serial as a screenshot, for tools/screenshot.py: a
// header line, then the raw 1280x720 RGB565 pixels as the canvas holds them.
// The canvas is landscape whatever the panel's rotation, so this is exactly
// what is on screen, the right way up.
void dumpToSerial();

}  // namespace screen
