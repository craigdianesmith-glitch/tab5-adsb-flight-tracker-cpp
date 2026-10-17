#include "labels3d.h"

#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#include <esp_heap_caps.h>
#endif

#ifndef PROGMEM
#define PROGMEM
#endif
#if __has_include(<lgfx/Fonts/glcdfont.h>)
#include <lgfx/Fonts/glcdfont.h>  // Font0's letters, a byte a column
#else
#include "glcdfont.h"
#endif

namespace labels3d {

namespace {

// As few as this in view, and the parked have labels too; and seen this big
// - by the camera - a parked aircraft has one however busy the view.
constexpr int QUIET = 6;
constexpr float PARKED_LABELLED_PX = 160;

// How much the labels say by how many there are to be: up to TWO_MOST, two
// lines; up to ONE_MOST, one; more, the callsign alone. Shorter as soon as
// the view fills - a quarter of a second, so that one aircraft passing
// through doesn't do it - but longer again only when it is well clear of the
// line, and has been for a second and a half: a view that hovers at the line
// doesn't have its labels flicking between the two.
constexpr int TWO_MOST = 4, ONE_MOST = 10;
constexpr float BUSIER_S = 0.25f, QUIETER_S = 1.5f;

// Where a label can go round its aircraft's point, in the order they are
// tried - after the one it had the frame before, where it can stay there.
enum Slot : uint8_t { UP_RIGHT, UP_LEFT, RIGHT, LEFT, HIGH_RIGHT, HIGH_LEFT, SLOTS };

constexpr int OFF = LEAD;
constexpr int GAP = 4;  // between one label and the next

bool overlaps(const Rect &a, const Rect &b, int gap) {
    return a.x0 < b.x1 + gap && b.x0 < a.x1 + gap && a.y0 < b.y1 + gap && b.y0 < a.y1 + gap;
}

// The box `slot` puts a w by h label in round the point (x, y), and where
// the line to the point leaves it.
Rect boxAt(Slot slot, int x, int y, int w, int h, int &fromX, int &fromY) {
    Rect r;
    switch (slot) {
    case UP_RIGHT: r = {x + OFF, y - OFF - h, x + OFF + w, y - OFF}; break;
    case UP_LEFT: r = {x - OFF - w, y - OFF - h, x - OFF, y - OFF}; break;
    case RIGHT: r = {x + OFF + 4, y - h / 2, x + OFF + 4 + w, y - h / 2 + h}; break;
    case LEFT: r = {x - OFF - 4 - w, y - h / 2, x - OFF - 4, y - h / 2 + h}; break;
    case HIGH_RIGHT: r = {x + OFF / 2, y - 2 * OFF - 2 * h, x + OFF / 2 + w, y - 2 * OFF - h}; break;
    default: r = {x - OFF / 2 - w, y - 2 * OFF - 2 * h, x - OFF / 2, y - 2 * OFF - h}; break;
    }
    bool right = slot == UP_RIGHT || slot == RIGHT || slot == HIGH_RIGHT;
    fromX = right ? r.x0 : r.x1 - 1;
    fromY = slot == RIGHT || slot == LEFT ? y : r.y1 - 1;
    return r;
}

// What a label says, in as many lines.
void words(const Candidate &c, Says says, char *line1, char *line2, size_t size) {
    line2[0] = 0;
    if (says == Says::TWO_LINES) {
        snprintf(line1, size, "%s", c.callsign);
        if (c.type != nullptr && c.type[0] != 0) {
            snprintf(line2, size, "%s %s", c.height, c.type);
        } else {
            snprintf(line2, size, "%s", c.height);
        }
    } else if (says == Says::ONE_LINE) {
        snprintf(line1, size, "%s %s", c.callsign, c.height);
    } else {
        snprintf(line1, size, "%s", c.callsign);
    }
}

}  // namespace

Labeller::Memo *Labeller::memoOf(uint32_t id, bool make) {
    for (int i = 0; i < memoCount_; i++) {
        if (memos_[i].id == id) {
            return &memos_[i];
        }
    }
    if (!make) {
        return nullptr;
    }
    Memo *m = &memos_[0];
    if (memoCount_ < MEMOS) {
        m = &memos_[memoCount_++];
    } else {
        // Full: the one out of view longest makes way.
        for (int i = 1; i < MEMOS; i++) {
            if (memos_[i].unseen > m->unseen) {
                m = &memos_[i];
            }
        }
    }
    *m = {id, UP_RIGHT, 0, 0, 0};
    return m;
}

void Labeller::reset() {
    memoCount_ = 0;
    says_ = Says::TWO_LINES;
    pending_ = 0;
    fresh_ = true;
}

int Labeller::place(const Candidate *candidates, int count, const Rect *keepOut, int keepOutCount, int width,
                    int height, float dt, Placed *out, int max) {
    max = std::min(max, MOST);
    if (bits_ == nullptr) {
        size_t bytes = (size_t)MOST * BITS_BYTES;
#if defined(ESP_PLATFORM)
        bits_ = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
#endif
        if (bits_ == nullptr) {
            bits_ = (uint8_t *)malloc(bytes);
        }
        if (bits_ == nullptr) {
            return 0;
        }
    }
    for (int i = 0; i < memoCount_; i++) {
        memos_[i].unseen = (uint16_t)std::min(60000, memos_[i].unseen + 1);
    }

    // Who may have one: all moving or flying, and the parked only in a
    // quiet view or by the camera. Then in turn: flying, then moving, then
    // parked; of each, those labelled the frame before, so that they keep
    // them; then the nearest - the biggest on the screen.
    int *order = order_, *score = score_, n = 0;
    for (int i = 0; i < count && n < CONSIDERED; i++) {
        const Candidate &c = candidates[i];
        Memo *memo = memoOf(c.id, false);
        if (memo != nullptr) {
            memo->unseen = 0;
        }
        if (c.kind == Kind::PARKED && count > QUIET && c.sizePx < PARKED_LABELLED_PX) {
            continue;
        }
        score[n] = (int)c.kind * 1000 + (memo != nullptr && memo->shown ? 500 : 0) + (int)std::min(c.sizePx, 400.0f);
        order[n++] = i;
    }
    for (int i = 1; i < n; i++) {  // a few dozen at most: an insertion sort
        for (int j = i; j > 0 && score[j] > score[j - 1]; j--) {
            std::swap(score[j], score[j - 1]);
            std::swap(order[j], order[j - 1]);
        }
    }

    // How much they say, by how many there are that could have one.
    Says want = n > ONE_MOST ? Says::CALLSIGN : n > TWO_MOST ? Says::ONE_LINE : Says::TWO_LINES;
    if (want > says_ && !fresh_) {
        // Longer only when well clear of the line.
        int clear = says_ == Says::CALLSIGN ? ONE_MOST - 2 : TWO_MOST - 1;
        if (n > clear) {
            want = says_;
        }
    }
    if (fresh_ || want == says_) {
        says_ = want;
        pending_ = 0;
    } else if ((pending_ += dt) >= (want < says_ ? BUSIER_S : QUIETER_S)) {
        says_ = want;
        pending_ = 0;
    }
    fresh_ = false;

    // Each in turn where it covers nothing it mustn't: in the slot it had,
    // if it can; else the first that's clear; else, saying less, the same
    // again.
    Rect *taken = taken_;
    int takenCount = 0, placed = 0;
    for (int k = 0; k < n && placed < max; k++) {
        const Candidate &c = candidates[order[k]];
        Memo *memo = memoOf(c.id, true);
        int x = (int)c.x, y = (int)c.y;
        bool done = false;
        for (int s = (int)says_; s >= 0 && !done; s--) {
            Placed &p = out[placed];
            p.says = (Says)s;
            words(c, p.says, p.line1, p.line2, sizeof p.line1);
            int chars = (int)std::max(strlen(p.line1), strlen(p.line2));
            int w = chars * CHAR_W + 2 * PAD, h = (p.line2[0] != 0 ? 2 : 1) * LINE + 2 * PAD;
            for (int t = -1; t < SLOTS && !done; t++) {
                Slot slot = t < 0 ? (Slot)memo->slot : (Slot)t;
                if ((t < 0 && !memo->shown) || (t >= 0 && memo->shown && slot == memo->slot)) {
                    continue;
                }
                Rect box = boxAt(slot, x, y, w, h, p.fromX, p.fromY);
                bool clear = box.x0 >= 0 && box.y0 >= 0 && box.x1 <= width && box.y1 <= height;
                for (int i = 0; i < keepOutCount && clear; i++) {
                    clear = !overlaps(box, keepOut[i], 0);
                }
                for (int i = 0; i < takenCount && clear; i++) {
                    clear = !overlaps(box, taken[i], GAP);
                }
                if (!clear) {
                    continue;
                }
                p.candidate = order[k];
                p.box = box;
                p.toX = x, p.toY = y;
                memo->slot = slot;
                memo->now = 1;
                done = true;
                // Its label, and the aircraft it is for, are the next's to
                // keep clear of.
                int half = (int)std::min(c.sizePx / 2, 120.0f), deep = (int)std::min(c.sizePx / 3, 80.0f);
                taken[takenCount++] = box;
                taken[takenCount++] = {x - half, y - 4, x + half, y + deep};
            }
        }
        placed += done;
    }

    for (int i = 0; i < memoCount_; i++) {
        memos_[i].shown = memos_[i].now;
        memos_[i].now = 0;
    }
    for (int i = 0; i < placed; i++) {
        write(out[i], bits_ + (size_t)i * BITS_BYTES);
    }
    return placed;
}

Labeller::~Labeller() { free(bits_); }

// Each of Font0's dots two pixels square; the line a pixel wide.
void write(Placed &p, uint8_t *bits) {
    int x0 = std::min(p.box.x0, p.toX), x1 = std::max(p.box.x1, p.toX + 1);
    int y0 = std::min(p.box.y0, p.toY), y1 = std::max(p.box.y1, p.toY + 1);
    p.bitsX = std::max(0, x0) & ~7;
    p.bitsY = y0;
    p.stride = std::min(MOST_STRIDE, (x1 - p.bitsX + 7) / 8);
    p.rows = std::min(MOST_ROWS, y1 - y0);
    size_t plane = (size_t)p.stride * p.rows;
    memset(bits, 0, 2 * plane);
    p.bright = bits;
    p.dim = bits + plane;
    auto dot = [&](uint8_t *to, int x, int y) {
        x -= p.bitsX, y -= p.bitsY;
        if (x >= 0 && x < p.stride * 8 && y >= 0 && y < p.rows) {
            to[y * p.stride + (x >> 3)] |= 0x80 >> (x & 7);
        }
    };
    auto words = [&](uint8_t *to, const char *line, int x, int y) {
        for (const char *ch = line; *ch != 0; ch++, x += CHAR_W) {
            const unsigned char *glyph = &font[(uint8_t)*ch * 5];
            for (int col = 0; col < 5; col++) {
                for (int row = 0; row < 8; row++) {
                    if (glyph[col] >> row & 1) {
                        for (int d = 0; d < 4; d++) {
                            dot(to, x + col * 2 + (d & 1), y + row * 2 + (d >> 1));
                        }
                    }
                }
            }
        }
    };
    words((uint8_t *)p.bright, p.line1, p.box.x0 + PAD, p.box.y0 + PAD);
    words((uint8_t *)p.dim, p.line2, p.box.x0 + PAD, p.box.y0 + PAD + LINE);
    int n = std::max(abs(p.toX - p.fromX), abs(p.toY - p.fromY));
    for (int i = 0; i <= n; i++) {
        dot((uint8_t *)p.dim, p.fromX + (p.toX - p.fromX) * i / std::max(1, n),
            p.fromY + (p.toY - p.fromY) * i / std::max(1, n));
    }
}

namespace {

bool g_simd = true;

}  // namespace

#if defined(ESP_PLATFORM)
// stamp()'s inner loop with the P4's PIE vector instructions - see
// labels3d_pie.S - eight pixels to an instruction, from a 16-byte boundary.
extern "C" void labels3d_stamp8(uint16_t *row, int chunks, const uint8_t *bits, const uint16_t *constants);
#endif

void setSimd(bool on) { g_simd = on; }

void stamp(uint16_t *band, int y, int rows, int width, const Placed *placed, int count, uint16_t bright,
           uint16_t dim) {
#if defined(ESP_PLATFORM)
    // labels3d_stamp8's constants for each colour - see there.
    alignas(16) uint16_t constants[2][2][8];
    for (int c = 0; c < 2; c++) {
        for (int lane = 0; lane < 8; lane++) {
            constants[c][0][lane] = c == 0 ? bright : dim;
            constants[c][1][lane] = (uint16_t)(0x8080 >> lane);
        }
    }
#endif
    for (int i = 0; i < count; i++) {
        const Placed &p = placed[i];
        // One starting past the band's right-hand end has nothing in it -
        // and a count of bytes under nought, which the PIE loop once took
        // for a count of four thousand million, and wrote on through the
        // heap: the board's crash at Glasgow, 17 Oct.
        if (p.bitsX >= width) {
            continue;
        }
        int r0 = std::max(p.bitsY, y), r1 = std::min(p.bitsY + p.rows, y + rows);
        // Whole bytes of the bitmaps, eight pixels each from bitsX - those
        // that are in the row - and then a last one the row ends part way
        // through, if it does.
        int bytes = std::min(p.stride, (width - p.bitsX) / 8);
        for (int r = r0; r < r1; r++) {
            uint16_t *px = band + (size_t)(r - y) * width + p.bitsX;
            for (int c = 0; c < 2; c++) {
                const uint8_t *bits = (c == 0 ? p.bright : p.dim) + (size_t)(r - p.bitsY) * p.stride;
                uint16_t color = c == 0 ? bright : dim;
                int b = 0;
#if defined(ESP_PLATFORM)
                if (g_simd && bytes > 0) {
                    labels3d_stamp8(px, bytes, bits, &constants[c][0][0]);
                    b = bytes;
                }
#endif
                for (; b < bytes; b++) {
                    for (int bit = 0; bits[b] != 0 && bit < 8; bit++) {
                        if (bits[b] & 0x80 >> bit) {
                            px[b * 8 + bit] = color;
                        }
                    }
                }
                for (int bit = 0; bytes < p.stride && bit < 8 && p.bitsX + bytes * 8 + bit < width; bit++) {
                    if (bits[bytes] & 0x80 >> bit) {
                        px[bytes * 8 + bit] = color;
                    }
                }
            }
        }
    }
}

}  // namespace labels3d
