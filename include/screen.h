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

void markDirty(int x, int y, int w, int h);
void markAllDirty();

// Push everything marked dirty to the panel, then start a fresh dirty list.
void flush();

}  // namespace screen
