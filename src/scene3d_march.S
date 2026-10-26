// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's terrain engine: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

// The land at its height, marched in assembly: scene3d.cpp's marchColumn<>()
// - a column of the 3D view's terrain, out from the eye - written for the
// ESP32-P4's RV32IMAFC core by hand, both its passes, the whole column in
// one call. What the march needs in registers, the floating-point unit's
// latencies covered by what doesn't wait on them, and nothing but the
// common case in the loops: what is rare - a sample off the grid, a block
// to test for skipping, a sample before one seen worked out again - done
// out of line, and nothing handed back to C.

// int scene3d_march_flat(MarchState *m);
// int scene3d_march_round(MarchState *m);
//
// One column's march over flat ground, or the round earth - marchColumn<>
// exactly, fused as it is, to the bit - from step m->farFirst with row
// m->filled - 1's 1 / depth at m->at: the column's rows filled from there
// up, and the top one returned (and left in m->filled). Sixteen steps a go,
// each go in two passes, as the C goes:
//
// First each step's sample on its own, against the line of sight through
// the top filled row as the go starts: its place on the grid, in 256ths of
// a cell - its cell and where in it at once - and the line there rounded up
// to a whole metre, set against its cell's highest corner, an integer: two
// compares a sample, the block it is in and the grid itself, kept as the
// stretch of the grid the next sample must fall in; under the line, hidden.
// Otherwise its height between its cell's corners, in integers, its depth,
// 1 / it, its row - and, from the sample before's, w's change a row down and
// w at its top row, all that the rows it may fill want. Each such sample a
// record: its step, its top row, that change and w there.
//
// Then the records in order, a compare each: one whose top row is above the
// top filled is seen, and its rows filled - those the drawing reads, the
// even ones, each band's first and the top - band by band, from the top
// filled up, bfloat16. Where its sample before wasn't worked out in the
// first pass - hidden by its cell, or in a block skipped - that one is
// worked out again for the change between them, as the C does, and this
// one with it: a record holds neither w nor row. Then, after each
// sixteenth step, the column given up once the highest land along it from
// there couldn't rise above the rows filled.
//
// Near the eye - nearer than skipTo - into another block of the grid, the
// line sloped, the block is skipped whole where its highest can't reach the
// line of sight anywhere across it. In the ring where an airfield's ground
// eases into the land's, the height eased as the C eases it. The steps run
// in stretches of one kind - near the eye or not, in the ring or not - each
// a loop of its own, so that none tests for what it can't meet.
//
// MarchState's fields, by offset (static_assert'ed in scene3d.cpp): the
// frame's: 0 far (the bases 1600 bytes on), 4 &far[steps], 8 &far[the first
// step not nearer than skipTo], 12 heights, 16 cellMaxBytes, 20 rowBytes,
// 24 xLim, 28 yLim, 32 bcols, 36 blockMax, 40 zHighAt, 44 bandRows, 48 the
// row above a band's first in bytes from it, 52 Fz, 56 Dz, 60 f, 64 cy -
// 0.5, 68 cy + 0.5, 72 f Dz, 76 fall, 80 2 fall, 84 0.5, 88 1, 92 1/65536,
// 96 1e-3, 100 -1e-4, 104 -inf, 108 1e9, 112 1e-9, 116 2500, 120 1/2500,
// 124 3; the column's: 128 X0, 132 XS, 136 XS2, 140 Y0, 144 YS, 148 YS2,
// 152 A, 156 B, 160 f B, 164 eh2f, 168 gez, 172 groundUp, 176 e2, 180 eh2,
// 184 1/XS, 188 1/YS, 192 easeLo, 196 easeHi, 200 &far[the ring's first
// step], 204 &far[the first past it], 208 &far[first], 212 row filled - 1's
// 1 / depth, 216 filled, 220 its row in its band, 224 the sample before's
// w, 228 its row; and what it hands back: 232 samples, 236 blocks skipped,
// 240 samples worked out, 244 those the second pass went through, 248 seen,
// 252 samples before worked out again, 256 blocks tested, 260 samples off
// the grid.
//
// Registers: s0 the step's &far[i] (the second pass, the record), s1 the
// stretch's end (the second pass, scratch), s2 the next record, s3 heights,
// s4 cellMaxBytes, s5 rowBytes, s6 xLim, s7 yLim, s8-s11 the stretch of the
// grid a sample must fall in to be in the block before's - its x and y
// from, its width and height - a0 m, a5 filled, a6 row filled - 1's 1 /
// depth, a7 its row in its band, ra the go's end; fs0-fs3 XS, X0, YS, Y0,
// fs4 the line's slope, fs5 A, fs6 B, fs7 Fz, fs8 Dz, fs9 f, fs10 cy - 0.5,
// fs11 1/65536, fa0 0.5, fa1 1, fa2 1e-3, fa3 0, fa4 and fa5 the last sample
// worked out's w and row, fa6 groundUp (round, XS2), fa7 (round) YS2, ft11
// (round) eh2f; ra the go's end in the first pass, and in the second the way
// back from its own _sample. It calls nothing else: s0-s11, fs0-fs11 and ra
// saved.

#define MS_FAR          0
#define MS_FAREND       4
#define MS_FARSKIPTO    8
#define MS_HEIGHTS      12
#define MS_CELLMAXBYTES 16
#define MS_ROWBYTES     20
#define MS_XLIM         24
#define MS_YLIM         28
#define MS_BCOLS        32
#define MS_BLOCKMAX     36
#define MS_ZHIGHAT      40
#define MS_BANDROWS     44
#define MS_BANDBACK2    48
#define MS_FZ           52
#define MS_DZ           56
#define MS_F            60
#define MS_CYLESS       64
#define MS_CYMORE       68
#define MS_FDZ          72
#define MS_FALL         76
#define MS_TWOFALL      80
#define MS_HALF         84
#define MS_ONE          88
#define MS_PERQ         92
#define MS_TINY         96
#define MS_NOTSLOPED    100
#define MS_MINUSINF     104
#define MS_FAROFF       108
#define MS_TINYSTEP     112
#define MS_FLATIN       116
#define MS_PERRING      120
#define MS_THREE        124
#define MS_X0           128
#define MS_XS           132
#define MS_XS2          136
#define MS_Y0           140
#define MS_YS           144
#define MS_YS2          148
#define MS_A            152
#define MS_B            156
#define MS_FB           160
#define MS_EH2F         164
#define MS_GEZ          168
#define MS_GROUNDUP     172
#define MS_E2           176
#define MS_EH2          180
#define MS_RXS          184
#define MS_RYS          188
#define MS_EASELO       192
#define MS_EASEHI       196
#define MS_FARLO        200
#define MS_FARHI        204
#define MS_FARFIRST     208
#define MS_AT           212
#define MS_FILLED       216
#define MS_OFF          220
#define MS_LASTW        224
#define MS_LASTRHO      228
#define MS_SAMPLES      232
#define MS_SKIPS        236
#define MS_WORKED       240
#define MS_PASSED       244
#define MS_SEEN         248
#define MS_AGAIN        252
#define MS_TESTED       256
#define MS_OFFGRID      260

// The stack frame: the registers saved, the go's state, and the records -
// the one before the go's first, then up to sixteen - 16 bytes each: the
// step's &far[i], its top row (an int's most, none), w's change a row down
// and w at the top row.
#define F_FPI           100     // the go's first step
#define F_FPSKIP        104     // the last step skipped
#define F_LASTFILLED    108     // filled when the line of sight was last worked out
#define F_SLOPED        112
#define F_FPEND         116     // the go's end
#define F_TOP           120     // the second pass's top and step, kept over a call
#define F_STEP          124
#define F_REC           128     // the record before the go's first
#define F_REC0          144     // the go's first
#define F_SIZE          400
// Sixteen steps a go - marchColumn's GO, static_assert'ed beside it - so
// sixteen records at most after the one before: to the frame's end. The
// go's end below (ori 60, srli 6) counts the same sixteen steps.
#define GO_STEPS        16
    .if F_REC0 + GO_STEPS * 16 > F_SIZE
    .error "a go's records run past the march's frame"
    .endif

// --- the first pass's parts ---------------------------------------------------

// x and y, the step's place on the grid in 256ths of a cell, in ft2 and ft3,
// and the line of sight there in ft4, from s in ft0 and its base in ft1.
.macro PLACE round
  .if \round
    fmadd.s ft2, ft0, fa6, fs0      // XS + s XS2
    fmadd.s ft3, ft0, fa7, fs2      // YS + s YS2
    fmadd.s ft4, ft0, fs4, ft1      // the line of sight: base + s slope
    fmadd.s ft2, ft0, ft2, fs1      // x
    fmadd.s ft3, ft0, ft3, fs3      // y
  .else
    fmadd.s ft2, ft0, fs0, fs1      // x = X0 + s XS
    fmadd.s ft3, ft0, fs2, fs3      // y
    fmadd.s ft4, ft0, fs4, ft1      // the line of sight: base + s slope
  .endif
.endm

// &heights[the cell] in `out` from x and y in 256ths, t0 and t1; `tmp`
// scratch. Masked, its row one of the grid's first four - the bench's.
.macro CELL out, tmp, masked
    srai    \out, t1, 8
  .if \masked
    andi    \out, \out, 3
  .endif
    mul     \out, \out, s5
    srai    \tmp, t0, 7
    andi    \tmp, \tmp, -2
    add     \out, \out, \tmp
    add     \out, \out, s3
.endm

// The height at x, y (t0, t1) between the corners of the cell at t3 - as
// marchColumn's between() has it, in 65536ths of a metre - in metres, in
// ft2. t0, t1, t5 and a1-a4 scratch.
.macro BETWEEN
    lh      a1, 0(t3)               // (x, y)
    lh      a2, 2(t3)               // (x + 1, y)
    add     t5, t3, s5
    lh      a3, 0(t5)               // (x, y + 1)
    lh      a4, 2(t5)               // (x + 1, y + 1)
    andi    t0, t0, 255             // where in the cell
    andi    t1, t1, 255
    sub     a2, a2, a1
    mul     a2, a2, t0
    slli    a1, a1, 8
    add     a1, a1, a2              // along the top
    sub     a4, a4, a3
    mul     a4, a4, t0
    slli    a3, a3, 8
    add     a3, a3, a4              // along the bottom
    sub     a3, a3, a1
    mul     a3, a3, t1
    slli    a1, a1, 8
    add     a1, a1, a3              // between them
    fcvt.s.w ft2, a1
    fmul.s  ft2, ft2, fs11          // metres
.endm

// The height in ft2 eased into the airfield's ground at s, ft0: t = (|eye
// + s h| - 2500) / 2500, 0 to 1, and groundUp + (h - groundUp) t t (3 - 2t).
// ft3-ft5 scratch.
.macro EASE
    flw     ft3, MS_EH2(a0)
    flw     ft4, MS_E2(a0)
    fadd.s  ft3, ft3, ft0           // eh2 + s
    fmadd.s ft3, ft0, ft3, ft4      // e2 + s (eh2 + s)
    fsqrt.s ft3, ft3
    flw     ft4, MS_FLATIN(a0)
    fsub.s  ft3, ft3, ft4
    flw     ft4, MS_PERRING(a0)
    fmul.s  ft3, ft3, ft4
    fmax.s  ft3, ft3, fa3           // 0 to 1
    fmin.s  ft3, ft3, fa1
    fmul.s  ft4, ft3, ft3           // t t
    fadd.s  ft3, ft3, ft3           // 2 t
    flw     ft5, MS_THREE(a0)
    fsub.s  ft3, ft5, ft3           // 3 - 2 t
    fmul.s  ft3, ft4, ft3
    fsub.s  ft2, ft2, fa6           // h - groundUp
    fmadd.s ft2, ft2, ft3, fa6
.endm

// The sample at s, ft0, its base ft1 and its height ft2, put on the screen:
// its depth zc in ft4 and its yc in ft5 - on to `behind` where zc is no
// more than 0.5 - its w = 1 / zc in ft6, its row in ft7 and the first whole
// row it reaches in t4.
.macro PROJECT round, behind
  .if \round
    fmadd.s ft3, ft0, ft11, ft1     // the base there: base + s eh2f
    fsub.s  ft3, ft2, ft3           // the height over it
  .else
    fsub.s  ft3, ft2, ft1           // the height over the base
  .endif
    fmul.s  ft4, ft3, fs7           // dz Fz
    fmul.s  ft5, ft3, fs8           // dz Dz
    fmadd.s ft4, ft0, fs5, ft4      // zc = s A + dz Fz
    fmadd.s ft5, ft0, fs6, ft5      // yc = s B + dz Dz
    flt.s   t4, fa0, ft4
    beqz    t4, \behind             // behind the eye: no row
    fdiv.s  ft6, fa1, ft4           // w = 1 / zc
    fmul.s  ft7, ft5, ft6
    fmadd.s ft7, ft7, fs9, fs10     // its row: yc w f + cy - 0.5
    fcvt.w.s t4, ft7, rup           // the first whole row it reaches
.endm

// From the sample before's w and row, fa4 and fa5, and this one's, ft6 and
// ft7, its top row t4: w's change a row down in ft9 - nought unless the one
// before is in front of the eye (its w over nought) and lower - and w at
// the top row in ft10, as the
// second pass of marchColumn has them. The record: the step, its top row,
// those two; and this sample the one before the next. t5, t6 and ft8 scratch.
.macro RECORD
    fsub.s  ft8, fa5, ft7           // the row before's less this one's
    fsub.s  ft9, fa4, ft6           // and its w less this one's
    flt.s   t5, fa3, fa4            // a w before to go by...
    flt.s   t6, fa2, ft8            // ...and the row before lower than this one's
    fcvt.s.w ft10, t4
    and     t5, t5, t6
    fdiv.s  ft9, ft9, ft8           // w's change a row down...
    fsub.s  ft10, ft10, ft7         // top - row
    bnez    t5, 1f
    fmv.w.x ft9, zero               // ...or none: level
1:
    fmadd.s ft10, ft10, ft9, ft6    // w at the top row, straight on from this sample...
    fmin.s  ft10, ft10, ft6         // ...no nearer than it
    sw      s0, 0(s2)
    sw      t4, 4(s2)
    fsw     ft9, 8(s2)
    fsw     ft10, 12(s2)
    addi    s2, s2, 16
    fmv.s   fa4, ft6
    fmv.s   fa5, ft7
.endm

// Into another block near the eye, the line sloped: the block - its column
// a1, its row a2 - skipped whole if its highest can't reach the line of
// sight anywhere across it, as marchColumn tests it: where the ray leaves
// it, sOut; the line there and here, and where the line, bent by the fall,
// is lowest between; under them all, the block's steps after this one
// skipped, on to the drive from the step after its last - a step or more
// skipped. Otherwise on to `cell`, this step's sample as any other. s, x,
// y and the line in ft0, ft2, ft3 and ft4; t0-t2, t6 and ft0-ft4 kept, t3-t5,
// a1-a3 and ft5-ft10 scratch.
.macro BLOCK_TEST pfx, cell, round
#ifdef BENCH3D
    lw      t3, MS_TESTED(a0)
    addi    t3, t3, 1
    sw      t3, MS_TESTED(a0)
#endif
    srai    a1, t0, 11              // the block's column...
    srai    a2, t1, 11              // ...and row
  .if \round
    fadd.s  ft5, ft0, ft0           // 2 s
    fmadd.s ft6, ft5, fa6, fs0      // dX = XS + 2 s XS2
    fmadd.s ft7, ft5, fa7, fs2      // dY
    flw     ft8, MS_TINYSTEP(a0)
    fabs.s  ft9, ft6
    flt.s   t3, ft9, ft8
    fdiv.s  ft6, fa1, ft6           // 1 / dX...
    beqz    t3, 1f
    fmv.w.x ft6, zero               // ...or none, too near square to x
1:
    fabs.s  ft9, ft7
    flt.s   t3, ft9, ft8
    fdiv.s  ft7, fa1, ft7
    beqz    t3, 2f
    fmv.w.x ft7, zero
2:
  .else
    flw     ft6, MS_RXS(a0)         // 1 / XS, nought for none
    flw     ft7, MS_RYS(a0)
  .endif
    // How far on the ray leaves it along x: its east side going east, its
    // west side going west - 1e9, never, not going either way.
    flt.s   t3, fa3, ft6            // east
    flt.s   t4, ft6, fa3            // west
    add     t5, a1, t3
    slli    t5, t5, 11
    fcvt.s.w ft9, t5
    fsub.s  ft9, ft9, ft2
    fmul.s  ft9, ft9, ft6
    or      t3, t3, t4
    bnez    t3, 3f
    flw     ft9, MS_FAROFF(a0)
3:
    flt.s   t3, fa3, ft7            // and along y
    flt.s   t4, ft7, fa3
    add     t5, a2, t3
    slli    t5, t5, 11
    fcvt.s.w ft10, t5
    fsub.s  ft10, ft10, ft3
    fmul.s  ft10, ft10, ft7
    or      t3, t3, t4
    bnez    t3, 4f
    flw     ft10, MS_FAROFF(a0)
4:
    fmin.s  ft9, ft9, ft10
    fmax.s  ft9, ft9, fa3
    fadd.s  ft9, ft0, ft9           // sOut: where the ray leaves the block
    flw     ft5, MS_FALL(a0)
    flw     ft6, MS_GEZ(a0)
    fmul.s  ft7, ft9, ft9
    fmadd.s ft7, ft7, ft5, ft6      // the base there, but for the column's fall
    fmadd.s ft7, ft9, fs4, ft7      // the line of sight there
    fmin.s  ft7, ft4, ft7           // the lower of it and here's
    flw     ft8, MS_TWOFALL(a0)
    fneg.s  ft10, fs4
    fdiv.s  ft10, ft10, ft8         // where the line, bent by the fall, is lowest
    flt.s   t3, ft0, ft10
    beqz    t3, 5f
    flt.s   t3, ft10, ft9
    beqz    t3, 5f
    fmul.s  ft8, ft10, ft10
    fmadd.s ft8, ft8, ft5, ft6
    fmadd.s ft8, ft10, fs4, ft8
    fmin.s  ft7, ft7, ft8           // lower still, between
5:
    lw      t3, MS_BCOLS(a0)
    mul     t3, a2, t3
    add     t3, t3, a1
    slli    t3, t3, 1
    lw      t4, MS_BLOCKMAX(a0)
    add     t3, t3, t4
    lh      t3, 0(t3)
    fcvt.s.w ft8, t3                // its highest...
  .if !\round
    flw     ft10, MS_EASELO(a0)     // ...or the airfield's ground, eased over it
    fle.s   t3, ft10, ft9
    beqz    t3, 6f
    flw     ft10, MS_EASEHI(a0)
    flt.s   t3, ft0, ft10
    beqz    t3, 6f
    fmax.s  ft8, ft8, fa6
6:
  .endif
    flt.s   t3, ft8, ft7
    beqz    t3, \cell               // it may reach the line
    mv      t3, s0                  // its last step: eight at a time, then one
    lw      t4, MS_FAREND(a0)
7:
    addi    t5, t3, 32
    bgeu    t5, t4, 8f
    flw     ft10, 32(t3)
    flt.s   a3, ft10, ft9
    beqz    a3, 8f
    mv      t3, t5
    j       7b
8:
    addi    t5, t3, 4
    bgeu    t5, t4, 9f
    flw     ft10, 4(t3)
    flt.s   a3, ft10, ft9
    beqz    a3, 9f
    mv      t3, t5
    j       8b
9:
    addi    t5, s0, 4
    bleu    t3, t5, \cell           // a step or none: not worth it
    lw      t4, MS_SKIPS(a0)        // hidden, all its steps
    addi    t4, t4, 1
    sw      t4, MS_SKIPS(a0)
    sw      t3, F_FPSKIP(sp)
    addi    t4, t3, 4               // its steps in this go no samples after all
    bleu    t4, ra, 10f
    mv      t4, ra
10:
    sub     t4, t4, t5
    srli    t4, t4, 2
    lw      a3, MS_SAMPLES(a0)
    sub     a3, a3, t4
    sw      a3, MS_SAMPLES(a0)
    addi    s0, t3, 4               // on from the step after its last
    j       \pfx\()_drive
.endm

// The first pass over the stretch [s0, s1) of steps of one kind: near the
// eye (into another block, tested) or not; in the airfield's ring or not.
// Each step: its sample hidden by its cell, or worked out and recorded.
// Then back to the drive.
.macro PASS1 pfx, v, round, near, ring, masked
\pfx\()_\v:
    flw     ft0, 0(s0)              // s
    flw     ft1, 1600(s0)           // its base
    PLACE   \round
    fcvt.w.s t0, ft2, rtz           // x and y in 256ths of a cell
    fcvt.w.s t1, ft3, rtz
    fcvt.w.s t2, ft4, rup           // the line, up to a whole metre
  .if \near
    sub     t3, t0, s8              // in the block before's part of the grid?
    bgeu    t3, s10, \pfx\()_\v\()_event
    sub     t3, t1, s9
    bgeu    t3, s11, \pfx\()_\v\()_event
  .else
    bgeu    t0, s6, \pfx\()_\v\()_off   // off the grid?
    bgeu    t1, s7, \pfx\()_\v\()_off
  .endif
\pfx\()_\v\()_cell:
  .if \ring
    flt.s   t6, fa6, ft4            // the airfield's ground under the line?
  .endif
    CELL    t3, t4, \masked
    add     t4, t3, s4
    lh      t4, 0(t4)               // its cell's highest corner
  .if \ring
    slt     t4, t4, t2
    and     t4, t4, t6
    beqz    t4, \pfx\()_\v\()_work
  .else
    bge     t4, t2, \pfx\()_\v\()_work
  .endif
    addi    s0, s0, 4               // under the line of sight, all its cell
    bltu    s0, s1, \pfx\()_\v
    j       \pfx\()_drive
\pfx\()_\v\()_work:
    BETWEEN
\pfx\()_\v\()_h:
  .if \ring
    EASE
  .endif
    PROJECT \round, \pfx\()_\v\()_behind
    bltz    t4, \pfx\()_\v\()_above
\pfx\()_\v\()_top:
    RECORD
    addi    s0, s0, 4
    bltu    s0, s1, \pfx\()_\v
    j       \pfx\()_drive
\pfx\()_\v\()_above:
    li      t4, 0                   // above the screen: from its top
    j       \pfx\()_\v\()_top
\pfx\()_\v\()_behind:
    li      t4, 0x7fffffff          // no row, never seen...
    sw      s0, 0(s2)
    sw      t4, 4(s2)
    addi    s2, s2, 16
    fmv.w.x fa4, zero               // ...and none to go by
    fmv.w.x fa5, zero
    addi    s0, s0, 4
    bltu    s0, s1, \pfx\()_\v
    j       \pfx\()_drive
\pfx\()_\v\()_off:  // off the grid: the sea's height
#ifdef BENCH3D
    lw      t3, MS_OFFGRID(a0)
    addi    t3, t3, 1
    sw      t3, MS_OFFGRID(a0)
#endif
    fmv.w.x ft2, zero
    j       \pfx\()_\v\()_h
  .if \near
\pfx\()_\v\()_event:
    bgeu    t0, s6, \pfx\()_\v\()_off
    bgeu    t1, s7, \pfx\()_\v\()_off
    li      t3, -2048               // into another block: its corner...
    and     s8, t0, t3
    and     s9, t1, t3
    sub     s10, s6, s8             // ...and as much of it as is on the grid
    sub     s11, s7, s9
    li      t3, 2048
    bleu    s10, t3, 1f
    mv      s10, t3
1:
    bleu    s11, t3, 2f
    mv      s11, t3
2:
    lw      t3, F_SLOPED(sp)
    beqz    t3, \pfx\()_\v\()_cell  // the line not sloped: none to skip
    BLOCK_TEST \pfx, \pfx\()_\v\()_cell, \round
  .endif
.endm

// The second pass's fill of rows [t0, filled) - the top row and filled -
// from w's change a row down, ft0, and w at the top row, ft1: those the
// drawing reads, band by band from row filled - 1 up - the even rows, then
// the band's first, or the top, if odd - each fmaf(row - top, change, w at
// top), bfloat16, rounded. Then filled the top, and a6 and a7 its row
// above's. t1-t4, a1-a3, s1 scratch; t5 bandRows, t6 the step back to the
// band before in bytes, a4 0x8000.
.macro FILL
    addi    t1, a5, -1              // the row at a6
1:  // the band's rows [lo, row]: lo its first, or the top
    sub     t2, t1, a7              // the band's first row
    mv      t3, t2
    bge     t2, t0, 2f
    mv      t3, t0                  // lo
2:
    andi    t4, t1, 1
    sub     a1, t1, t4              // the even row at or above it
    slli    t4, t4, 1
    sub     a2, a6, t4              // and where it is kept
    blt     a1, t3, 4f              // none in the band's rows
    sub     a3, a1, t0              // its row from the top
3:
    fcvt.s.w ft2, a3
    fmadd.s ft2, ft2, ft0, ft1
    fmv.x.w s1, ft2
    add     s1, s1, a4
    srli    s1, s1, 16
    sh      s1, 0(a2)
    addi    a3, a3, -2
    addi    a2, a2, -4
    addi    a1, a1, -2
    bge     a1, t3, 3b
4:
    andi    t4, t3, 1
    sub     a1, t1, t3
    slli    a1, a1, 1
    sub     a2, a6, a1              // lo's place
    beqz    t4, 5f                  // lo even: kept already
    sub     a3, t3, t0
    fcvt.s.w ft2, a3
    fmadd.s ft2, ft2, ft0, ft1
    fmv.x.w s1, ft2
    add     s1, s1, a4
    srli    s1, s1, 16
    sh      s1, 0(a2)
5:
    mv      a6, a2
    bne     t3, t2, 6f              // lo the top, short of the band's first
    add     a6, a6, t6              // the band before's last row
    addi    a7, t5, -1
    beq     t3, t0, 7f              // the top the band's first: done
    addi    t1, t3, -1              // on up into the band before
    j       1b
6:
    addi    a6, a6, -2              // the row above the top, in its band
    sub     a7, t0, t2
    addi    a7, a7, -1
7:
    mv      a5, t0
.endm

// The sample at step t2 (&far[i]) worked out as the first pass works it
// out: its w in ft6 - nought, behind the eye - and its row in ft7. Called
// (jal) by the second pass for a sample seen whose sample before wasn't
// worked out, for both. t0, t1, t3-t5, a1-a4 and ft0-ft7 scratch.
.macro SAMPLE pfx, round, masked
\pfx\()_sample:
    flw     ft0, 0(t2)              // s
    flw     ft1, 1600(t2)           // its base
    PLACE   \round
    fcvt.w.s t0, ft2, rtz
    fcvt.w.s t1, ft3, rtz
    fmv.w.x ft2, zero               // off the grid, the sea's height
    bgeu    t0, s6, 1f
    bgeu    t1, s7, 1f
    CELL    t3, t4, \masked
    BETWEEN
1:
  .if !\round
    lw      t4, MS_FARLO(a0)        // in the airfield's ring?
    bltu    t2, t4, 2f
    lw      t4, MS_FARHI(a0)
    bgeu    t2, t4, 2f
    EASE
2:
  .endif
    fmv.w.x ft6, zero
    PROJECT \round, 3f
3:
    ret
.endm

// --- the march ----------------------------------------------------------------

.macro MARCH_FN name, pfx, round, masked
    .align  2
    .global \name
    .type   \name, @function
\name:
    addi    sp, sp, -F_SIZE
    sw      s0, 0(sp)
    sw      s1, 4(sp)
    sw      s2, 8(sp)
    sw      s3, 12(sp)
    sw      s4, 16(sp)
    sw      s5, 20(sp)
    sw      s6, 24(sp)
    sw      s7, 28(sp)
    sw      s8, 32(sp)
    sw      s9, 36(sp)
    sw      s10, 40(sp)
    sw      s11, 44(sp)
    fsw     fs0, 48(sp)
    fsw     fs1, 52(sp)
    fsw     fs2, 56(sp)
    fsw     fs3, 60(sp)
    fsw     fs4, 64(sp)
    fsw     fs5, 68(sp)
    fsw     fs6, 72(sp)
    fsw     fs7, 76(sp)
    fsw     fs8, 80(sp)
    fsw     fs9, 84(sp)
    fsw     fs10, 88(sp)
    fsw     fs11, 92(sp)
    sw      ra, 96(sp)
    lw      s3, MS_HEIGHTS(a0)
    lw      s4, MS_CELLMAXBYTES(a0)
    lw      s5, MS_ROWBYTES(a0)
    lw      s6, MS_XLIM(a0)
    lw      s7, MS_YLIM(a0)
    li      s8, 0                   // no block yet: no part of the grid
    li      s9, 0
    li      s10, 0
    li      s11, 0
    flw     fs0, MS_XS(a0)
    flw     fs1, MS_X0(a0)
    flw     fs2, MS_YS(a0)
    flw     fs3, MS_Y0(a0)
    flw     fs5, MS_A(a0)
    flw     fs6, MS_B(a0)
    flw     fs7, MS_FZ(a0)
    flw     fs8, MS_DZ(a0)
    flw     fs9, MS_F(a0)
    flw     fs10, MS_CYLESS(a0)
    flw     fs11, MS_PERQ(a0)
    flw     fa0, MS_HALF(a0)
    flw     fa1, MS_ONE(a0)
    flw     fa2, MS_TINY(a0)
    fmv.w.x fa3, zero
    flw     fa4, MS_LASTW(a0)       // the sample before the first: the flat ground's edge, or none
    flw     fa5, MS_LASTRHO(a0)
  .if \round
    flw     fa6, MS_XS2(a0)
    flw     fa7, MS_YS2(a0)
    flw     ft11, MS_EH2F(a0)
  .else
    flw     fa6, MS_GROUNDUP(a0)
  .endif
    lw      a5, MS_FILLED(a0)
    lw      a6, MS_AT(a0)
    lw      a7, MS_OFF(a0)
    lw      t0, MS_FARFIRST(a0)
    sw      t0, F_FPI(sp)
    addi    t1, t0, -4              // the record before the first: the step before it, as if worked out
    sw      t1, F_REC(sp)
    lw      t2, MS_FAR(a0)
    addi    t2, t2, -4
    sw      t2, F_FPSKIP(sp)        // none skipped
    li      t3, -1
    sw      t3, F_LASTFILLED(sp)
    sw      zero, MS_SAMPLES(a0)
    sw      zero, MS_SKIPS(a0)
#ifdef BENCH3D
    sw      zero, MS_WORKED(a0)
    sw      zero, MS_PASSED(a0)
    sw      zero, MS_SEEN(a0)
    sw      zero, MS_AGAIN(a0)
    sw      zero, MS_TESTED(a0)
    sw      zero, MS_OFFGRID(a0)
#endif

\pfx\()_go:  // a go: steps [i, end), end the next sixteenth or the last
    lw      t0, F_FPI(sp)
    lw      t1, MS_FAR(a0)
    sub     t2, t0, t1
    ori     t2, t2, 60
    addi    t2, t2, 4
    add     ra, t1, t2
    lw      t3, MS_FAREND(a0)
    bleu    ra, t3, 1f
    mv      ra, t3
1:
    sw      ra, F_FPEND(sp)
    lw      t4, F_LASTFILLED(sp)
    beq     t4, a5, 2f
    // The line of sight through the top filled row as the go starts - or,
    // not sloped, none to hide a sample by its cell.
    sw      a5, F_LASTFILLED(sp)
    fcvt.s.w ft0, a5
    flw     ft1, MS_CYMORE(a0)
    fsub.s  ft0, ft0, ft1           // k
    flw     ft2, MS_FDZ(a0)
    fnmsub.s ft2, ft0, fs7, ft2     // f Dz - k Fz
    flw     ft3, MS_NOTSLOPED(a0)
    flt.s   t4, ft2, ft3
    sw      t4, F_SLOPED(sp)
    flw     ft4, MS_MINUSINF(a0)
    beqz    t4, 3f
    flw     ft5, MS_FB(a0)
    fmsub.s ft4, ft0, fs5, ft5      // k A - f B
    fdiv.s  ft4, ft4, ft2           // gk
3:
    flw     ft5, MS_EH2F(a0)
    fadd.s  fs4, ft4, ft5           // its slope from each step's base
2:
    lw      t5, F_FPSKIP(sp)
    addi    t5, t5, 4               // from the step after the last skipped
    mv      s0, t0
    bgeu    t0, t5, 4f
    mv      s0, t5
4:
    addi    s2, sp, F_REC0
    bgeu    s0, ra, \pfx\()_pass2   // all of it skipped
    sub     t6, ra, s0
    srli    t6, t6, 2
    lw      t4, MS_SAMPLES(a0)
    add     t4, t4, t6
    sw      t4, MS_SAMPLES(a0)

\pfx\()_drive:  // the first pass, on from step s0: the stretch of one kind of step from it
    bgeu    s0, ra, \pfx\()_pass2
    lw      t0, MS_FARSKIPTO(a0)
    mv      s1, ra
  .if !\round
    lw      t1, MS_FARLO(a0)
    lw      t2, MS_FARHI(a0)
  .endif
    bgeu    s0, t0, 5f              // not near the eye: no blocks
    bleu    s1, t0, 6f
    mv      s1, t0
6:
  .if \round
    j       \pfx\()_n
  .else
    bltu    s0, t1, 7f              // short of the ring
    bgeu    s0, t2, \pfx\()_n       // past it
    bleu    s1, t2, 8f
    mv      s1, t2
8:
    j       \pfx\()_nr
7:
    bleu    s1, t1, \pfx\()_n
    mv      s1, t1
    j       \pfx\()_n
  .endif
5:
  .if \round
    j       \pfx\()_f
  .else
    bltu    s0, t1, 7f
    bgeu    s0, t2, \pfx\()_f
    bleu    s1, t2, 8f
    mv      s1, t2
8:
    j       \pfx\()_fr
7:
    bleu    s1, t1, \pfx\()_f
    mv      s1, t1
    j       \pfx\()_f
  .endif

    PASS1   \pfx, n, \round, 1, 0, \masked
    PASS1   \pfx, f, \round, 0, 0, \masked
  .if !\round
    PASS1   \pfx, nr, 0, 1, 1, \masked
    PASS1   \pfx, fr, 0, 0, 1, \masked
  .endif

\pfx\()_pass2:  // the go's samples worked out, in order: each whose top row is above the top filled one seen
    addi    s0, sp, F_REC0
    bgeu    s0, s2, \pfx\()_end
    li      a4, 0x8000
    lw      t5, MS_BANDROWS(a0)
    lw      t6, MS_BANDBACK2(a0)
\pfx\()_record:
    lw      t0, 4(s0)               // its top row
    blt     t0, a5, \pfx\()_seen
\pfx\()_next:
    addi    s0, s0, 16
    bltu    s0, s2, \pfx\()_record
#ifdef BENCH3D
    addi    t0, sp, F_REC0
    sub     t0, s2, t0
    srli    t0, t0, 4
    lw      t1, MS_WORKED(a0)
    add     t1, t1, t0
    sw      t1, MS_WORKED(a0)
    lw      t1, MS_PASSED(a0)
    add     t1, t1, t0
    sw      t1, MS_PASSED(a0)
#endif
    lw      t0, -16(s2)             // the go's last record: the one before the next go's
    sw      t0, F_REC(sp)
\pfx\()_end:  // given up once the highest land about, this far off or further, would be under the rows filled
    lw      t0, F_FPEND(sp)
    lw      t1, MS_FAREND(a0)
    beq     t0, t1, \pfx\()_out     // the steps run out
    flw     ft0, -4(t0)             // s, the go's last step's
    lw      t2, MS_FAR(a0)
    sub     t2, t0, t2
    addi    t2, t2, -4
    srli    t2, t2, 6
    slli    t2, t2, 2
    lw      t3, MS_ZHIGHAT(a0)
    add     t2, t2, t3
    flw     ft1, 0(t2)              // the highest land along the column from this sixteenth on
    fcvt.s.w ft2, a5
    flw     ft3, MS_CYMORE(a0)
    fsub.s  ft2, ft2, ft3           // k
    flt.s   t2, fa3, ft1
    beqz    t2, 1f
    fmul.s  ft4, ft1, fs7
    fmadd.s ft4, ft0, fs5, ft4      // zcHigh = s A + zh Fz
    flt.s   t2, fa0, ft4
    beqz    t2, 2f
    fmul.s  ft5, ft1, fs8
    fmadd.s ft5, ft0, fs6, ft5      // ycHigh
    fmul.s  ft5, fs9, ft5           // f ycHigh
    fmul.s  ft4, ft2, ft4           // k zcHigh
    flt.s   t2, ft4, ft5
    bnez    t2, \pfx\()_out
    j       2f
1:
    flt.s   t2, fa3, fs5            // A > 0
    beqz    t2, 2f
    flw     ft5, MS_FB(a0)
    fmul.s  ft4, ft2, fs5           // k A
    flt.s   t2, ft4, ft5
    bnez    t2, \pfx\()_out
2:
    sw      t0, F_FPI(sp)           // on to the next go
    j       \pfx\()_go

\pfx\()_seen:  // seen: its rows filled, from w's change and w at its top row
#ifdef BENCH3D
    lw      t1, MS_SEEN(a0)
    addi    t1, t1, 1
    sw      t1, MS_SEEN(a0)
#endif
    lw      t1, 0(s0)               // its step
    lw      t2, -16(s0)             // the step of the record before
    flw     ft0, 8(s0)              // w's change a row down
    flw     ft1, 12(s0)             // w at the top row
    addi    t2, t2, 4
    bne     t2, t1, \pfx\()_again   // the sample before not worked out
\pfx\()_fill:
    FILL
    beqz    a5, \pfx\()_done        // all of it: the column done
    j       \pfx\()_next

\pfx\()_again:  // the sample before, hidden by its cell or skipped: it, and this, worked out again
#ifdef BENCH3D
    lw      t2, MS_AGAIN(a0)
    addi    t2, t2, 1
    sw      t2, MS_AGAIN(a0)
#endif
    sw      t0, F_TOP(sp)
    sw      t1, F_STEP(sp)
    addi    t2, t1, -4
    jal     \pfx\()_sample
    fmv.s   ft8, ft6                // the one before's w...
    fmv.s   ft9, ft7                // ...and row
    lw      t2, F_STEP(sp)
    jal     \pfx\()_sample          // this one's
    lw      t0, F_TOP(sp)
    fmv.w.x ft0, zero
    flt.s   t2, fa3, ft8            // a w before to go by...
    fsub.s  ft10, ft9, ft7
    flt.s   t3, fa2, ft10           // ...and lower than this one
    and     t2, t2, t3
    beqz    t2, 1f
    fsub.s  ft2, ft8, ft6
    fdiv.s  ft0, ft2, ft10          // w's change a row down
1:
    fcvt.s.w ft2, t0
    fsub.s  ft2, ft2, ft7
    fmadd.s ft2, ft2, ft0, ft6
    fmin.s  ft1, ft2, ft6           // w at the top row
    li      a4, 0x8000
    lw      t5, MS_BANDROWS(a0)
    lw      t6, MS_BANDBACK2(a0)
    j       \pfx\()_fill

\pfx\()_done:  // the top row filled
#ifdef BENCH3D
    addi    t0, sp, F_REC0
    sub     t1, s2, t0
    srli    t1, t1, 4
    lw      t2, MS_WORKED(a0)
    add     t2, t2, t1
    sw      t2, MS_WORKED(a0)
    addi    t1, s0, 16
    sub     t1, t1, t0
    srli    t1, t1, 4
    lw      t2, MS_PASSED(a0)
    add     t2, t2, t1
    sw      t2, MS_PASSED(a0)
#endif
\pfx\()_out:
    sw      a5, MS_FILLED(a0)
    mv      a0, a5
    lw      s0, 0(sp)
    lw      s1, 4(sp)
    lw      s2, 8(sp)
    lw      s3, 12(sp)
    lw      s4, 16(sp)
    lw      s5, 20(sp)
    lw      s6, 24(sp)
    lw      s7, 28(sp)
    lw      s8, 32(sp)
    lw      s9, 36(sp)
    lw      s10, 40(sp)
    lw      s11, 44(sp)
    flw     fs0, 48(sp)
    flw     fs1, 52(sp)
    flw     fs2, 56(sp)
    flw     fs3, 60(sp)
    flw     fs4, 64(sp)
    flw     fs5, 68(sp)
    flw     fs6, 72(sp)
    flw     fs7, 76(sp)
    flw     fs8, 80(sp)
    flw     fs9, 84(sp)
    flw     fs10, 88(sp)
    flw     fs11, 92(sp)
    lw      ra, 96(sp)
    addi    sp, sp, F_SIZE
    ret

    SAMPLE  \pfx, \round, \masked
    .size   \name, .-\name
.endm

    MARCH_FN scene3d_march_flat, .Lmf, 0, 0
    MARCH_FN scene3d_march_round, .Lmr, 1, 0
#ifdef BENCH3D
    // For the bench's probe of what memory costs the march: its heights
    // read from the grid's first four rows.
    MARCH_FN scene3d_march_flat_masked, .Lmfm, 0, 1
    MARCH_FN scene3d_march_round_masked, .Lmrm, 1, 1
#endif
