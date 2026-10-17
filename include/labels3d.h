#pragma once

#include <stdint.h>

// Which of the traffic in the 3D view gets a label, how much it says, and
// where it goes: the busier the view, the fewer and the shorter. In a quiet
// one each aircraft has its callsign over its height and type; as it fills,
// one line - callsign and height - and then the callsign alone. Parked
// aircraft lose theirs first, then the farthest of those moving, and there
// are never more than ten. None covers the aircraft followed, the captions
// or another label, and each stays where it was the frame before while it
// can, so that they don't jump about as the view moves.
//
// And how they go over the picture: each label's words, and its line to its
// aircraft, drawn once a frame into bitmaps, a bit a pixel, and stamped into
// the bands of the picture as the 3D view draws them - no box, just their
// pixels, cut into the picture with an AND and their colour ORed in - eight
// pixels to an instruction on the P4.
namespace labels3d {

// What an aircraft is doing - which has a label first.
enum class Kind : uint8_t { PARKED, MOVING, AIRBORNE };

// How much a label says.
enum class Says : uint8_t { CALLSIGN, ONE_LINE, TWO_LINES };

struct Candidate {
    float x, y;        // the point over the aircraft, on the screen
    float sizePx;      // how long it looks there
    uint32_t id;       // the same aircraft from one frame to the next: its address
    Kind kind;
    const char *callsign;
    const char *height;  // "FL350^", "3200ft", "GND"
    const char *type;    // "A320", or ""
};

struct Rect {
    int x0, y0, x1, y1;  // x1 and y1 just past it
};

// The words are Font0's five by seven letters at twice the size, as the
// radar writes them: 12 pixels a character, LINE a line, with PAD round them
// inside the box. LEAD from the aircraft's point to its label's box.
constexpr int CHAR_W = 12, LINE = 18, PAD = 5, LEAD = 10;

struct Placed {
    int candidate;  // which of those given
    Says says;
    Rect box;          // where its words go, with room round them: none of it drawn
    int fromX, fromY;  // where its line to the aircraft leaves the box
    int toX, toY;      // and the point it goes to
    char line1[24], line2[24];
    // Its words and its line, a bit a pixel and the first bit the leftmost,
    // in two bitmaps: the first line's words, to be in the bright colour;
    // the second line's and the line to the aircraft, in the dim one. Each
    // `rows` rows of `stride` bytes from row bitsY, and from column bitsX -
    // rounded down to a multiple of 8, for each byte to go with eight pixels
    // of a row that start on one. Good until the next place().
    const uint8_t *bright, *dim;
    int bitsX, bitsY, stride, rows;
};

constexpr int MOST = 10;

// The most a label's two bitmaps take: across, 23 characters, its line to
// the aircraft beside it and a byte's worth of rounding; down, two lines,
// and its line from the highest it can be over its aircraft.
constexpr int MOST_STRIDE = (23 * CHAR_W + 2 * PAD + LEAD + 4 + 7 + 7) / 8;
constexpr int MOST_ROWS = 2 * (2 * LINE + 2 * PAD) + 2 * LEAD + 1;
constexpr int BITS_BYTES = 2 * MOST_STRIDE * MOST_ROWS;

// A label's words and its line into its bitmaps - BITS_BYTES at `bits` - for
// its box and its line: what place() does for those it places, for a label
// put somewhere by hand.
void write(Placed &p, uint8_t *bits);

class Labeller {
public:
    Labeller() = default;
    Labeller(const Labeller &) = delete;
    Labeller &operator=(const Labeller &) = delete;
    ~Labeller();

    // This frame's labels for `count` aircraft in view - `keepOut` what none
    // may cover, as the captions and the aircraft followed, and `dt` the
    // seconds since the frame before. Up to `max` (and MOST); returns how
    // many.
    int place(const Candidate *candidates, int count, const Rect *keepOut, int keepOutCount, int width, int height,
              float dt, Placed *out, int max);
    // Forget the frames before: a new view.
    void reset();

    Says says() const { return says_; }

private:
    // An aircraft labelled lately: where its label went.
    struct Memo {
        uint32_t id;
        uint8_t slot;
        uint8_t shown;    // labelled in the frame before
        uint8_t now;      // labelled in this one
        uint16_t unseen;  // frames since it was last in view
    };
    static constexpr int MEMOS = 64, CONSIDERED = 64;

    Memo *memoOf(uint32_t id, bool make);

    Memo memos_[MEMOS] = {};
    int memoCount_ = 0;
    Says says_ = Says::TWO_LINES;
    float pending_ = 0;  // how long the view has called for saying more, or less
    bool fresh_ = true;  // the first frame of a view: as it calls for, at once
    // place()'s working, kept here rather than on the stack of the task that
    // calls it - the Arduino loop's is 8KB, and the 3D view already deep in
    // it - and the bitmaps, in PSRAM.
    int order_[CONSIDERED], score_[CONSIDERED];
    Rect taken_[MOST * 2];
    uint8_t *bits_ = nullptr;
};

// The labels stamped into a band of the picture: `rows` rows of `width`
// pixels from row y, as the 3D view draws a frame a band at a time - each
// one's first line in `bright` and its second, and its line to its
// aircraft, in `dim`; between them, the picture as it was. The colours as
// the band's pixels are - byte-swapped, for the panel. Rows start on a
// 16-byte boundary, as the 3D view's do.
void stamp(uint16_t *band, int y, int rows, int width, const Placed *placed, int count, uint16_t bright,
           uint16_t dim);

// Whether stamp() uses the P4's PIE vector instructions, where it can (the
// default) - or does it all in C, to compare.
void setSimd(bool on);

}  // namespace labels3d
