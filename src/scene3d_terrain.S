// The land at its height, in assembly: the innermost loops of the 3D view's
// terrain (see scene3d.cpp's terrainMarch() and terrainBand()), written for
// the ESP32-P4's RV32IMAFC core by hand - every value the loop needs kept in
// a register, the floating-point unit's latencies covered by work that
// doesn't wait on them, and nothing but the common case in the loop: what
// is rare is handed back to C, which does it and comes back.

// int scene3d_terrain_band(TerrainCol *t);
//
// A band's land at its height for the flat cover - scene3d.cpp's
// terrainSampleC() exactly, a sample at a time in the same order - a pair
// of rows at a time across the columns, so that each sample's neighbours
// are its column's on either side: much the same depth, the same part of
// the cover, often the same cell and haze. Where the band has no land,
// nothing: a pair of rows above the frame's highest land is passed over
// whole, and across a pair the columns are taken eight at a time - a
// strip - from the least and the most of their highest rows of land and
// the least of their flat tops (see TerrainStrips): a strip with no land in
// the pair passed over, one that is land in every column with no flat
// ground the common way, and one that is some of each the general way.
//
// The common way, strips of it together, two columns a turn: each one's
// 1 / depth, bfloat16, and the depth from it, the two divisions apart; its
// level told from the 1 / depth itself, against the bfloat16s the map's
// level takes (wLo up, wRange of them - see TerrainMap); where on the
// cover's grid that is, u = ue + z (rowU[y] + cu) and v likewise; the cell
// there, from the map of the part of the cover being read - two
// multiply-adds, two casts and two compares; and the distance's bits with
// its cell, its key': the cell and the haze's index, which the haze is
// read by. The same key' as the sample before, the same pairs of pixels:
// four stores. The two columns' sums interleaved, each one's waits filled
// by the other's.
//
// The general way, a sample at a time: a column whose land starts in the
// pair - on its odd row, its own sample there - the airfield's flat ground,
// the row's own depth from the column's flat top down; an odd row alone,
// a band's first, or an even one, its last.
//
// Off the map - the depth another level's, the place off the map's part -
// the part cellAt() would read the sample in is looked for among those the
// core has kept this frame (TerrainMaps), its level's depths and the part's
// own place, and read as cellSlow() reads it: as the map reads it where
// the map reads this one, else as the part does. Only a sample in none of
// them is handed to C, which does it the long way and keeps the map it
// makes for the next.
//
// A sample's pairs of pixels, for its cell and haze, its key: the last's,
// or from the core's cache, the even rows' and the odd's together - worked
// out as pairInt() does where it isn't there, and kept.
//
// It returns 0 once the band is done, and 1 at a sample it can't do, with
// t->y, t->px, t->zc, the column's flat top, cu and cv, the map, the
// parities' last keys and pairs as C needs them to do it the long way, and
// t->rowAt and t->c where to go on from.
//
// Registers: s0 the column's pixels on the unit's first row, s1 its 1 /
// depths there, s2 the column, s3 the row under's pixels - in the general
// way, the link back - s4 its cu and cv, s5 the row's pixels, column
// nought's, s6 a column's bytes of 1 / depths, s7 where the run or the
// strip ends, s8 yb, s9 the unit's end, s10 the map's cells, s11 its
// columns, a1 its rows, a2 its stride, a3 and a4 its level's 1 / depths -
// wLo, wRange - a5 the last key', a6 and a7 the last pairs, even rows' and
// odd's, t6 the unit's first row, ra the haze's index bits of a distance;
// fa6, fa7, ft8, ft9 the map's au, bu, av, bv, ft10 1, fs0-fs2 the row's
// rowU, rowV, rowFar; flat, fa2 and fa3 ue and ve; round, U0-U5 in fa2,
// fa0, fa1, ft11, fs8, fs9, V0-V2 in fa3, fs10, fs11, V3-V5 read from t.
// The common way, two columns, s8 its 1 / depths' step - quad, the rows
// under in s8 and s9. It calls nothing: s0-s11, ra and fs0-fs11 saved.

// TerrainCol's fields, by offset (static_assert'ed in scene3d.cpp).
    .equ    T_PX, 0
    .equ    T_ZC, 4
    .equ    T_Y, 8
    .equ    T_YB, 12
    .equ    T_FLATTOP, 16
    .equ    T_STRIDE, 20
    .equ    T_CU, 24
    .equ    T_CV, 28
    .equ    T_UE, 32
    .equ    T_VE, 36
    .equ    T_ROWU, 40
    .equ    T_ROWV, 44
    .equ    T_ROWFAR, 48
    .equ    T_FLATZ, 52
    .equ    T_HAZE, 56
    .equ    T_ZLO, 60
    .equ    T_ZHI, 64
    .equ    T_AU, 68
    .equ    T_BU, 72
    .equ    T_AV, 76
    .equ    T_BV, 80
    .equ    T_CELLS, 84
    .equ    T_COLS, 88
    .equ    T_ROWS, 92
    .equ    T_MSTRIDE, 96
    .equ    T_KEYE, 100
    .equ    T_PAIRE, 104
    .equ    T_KEYO, 108
    .equ    T_PAIRO, 112
    .equ    T_SAMPLES, 116
    .equ    T_PAIRS, 120
    .equ    T_TOP, 124
    .equ    T_C, 128
    .equ    T_BANDCOLS, 132
    .equ    T_Y0, 136
    .equ    T_TOPFLAT, 140
    .equ    T_COLUV, 144
    .equ    T_ROWAT, 148
    .equ    T_ZS, 152
    .equ    T_BANDBYTES, 156
    .equ    T_COLOR, 160
    .equ    T_COVERRGB, 164
    .equ    T_HAZECOLOR, 168
    .equ    T_SWAP, 172
    .equ    T_MISSES, 176
    .equ    T_MAPU, 180
    .equ    T_MAPV, 204
    .equ    T_WLO, 228
    .equ    T_WRANGE, 232
    .equ    T_MAPS, 236
    .equ    T_STRIPS, 240
    .equ    T_SWITCHES, 244
// TerrainMap's, TerrainMaps' and TerrainPairs'.
    .equ    M_ZLO, 0
    .equ    M_ZHI, 4
    .equ    M_AU, 8
    .equ    M_BU, 12
    .equ    M_AV, 16
    .equ    M_BV, 20
    .equ    M_CELLS, 24
    .equ    M_COLS, 28
    .equ    M_ROWS, 32
    .equ    M_STRIDE, 36
    .equ    M_WLO, 40
    .equ    M_WRANGE, 44
    .equ    M_SPLIT, 48
    .equ    M_U0, 52
    .equ    M_V0, 56
    .equ    M_U1, 60
    .equ    M_V1, 64
    .equ    M_KU, 68
    .equ    M_KV, 72
    .equ    M_COLS1, 76
    .equ    M_ROWS1, 80
    .equ    M_SIZE, 96
    .equ    MS_COUNT, 4
    .equ    MS_MAP, 16
    .equ    P_KEY, 4
    .equ    P_PAIR, 260
// The stack frame's: s0-s11 and ra, fs0-fs11, then what is kept in it.
    .equ    F_FSAVE, 52
    .equ    F_KEYE, 100             // the parities' last keys
    .equ    F_KEYO, 104
    .equ    F_ZROW, 108             // the unit's first row's 1 / depths, column nought's
    .equ    F_END, 112              // past the common way's run, its pixels
    .equ    F_STRIDE, 116           // a row's bytes
    .equ    F_SEG, 120              // the run's pixels where its samples were last counted
    .equ    F_KB, 124               // B's key', over A's pairs being found
    .equ    F_KP, 128               // a sample's key', over its pairs being found
    .equ    F_ROW, 132              // the general way's sample's row
    .equ    F_WHOLE, 136            // the unit whole: 1 - the common way can have it
    .equ    F_PAIRS, 140            // five registers, over a sample's pairs worked out
    .equ    FRAME, 176

// A level of one channel, as pairInt() packs it: x = c top + dither,
// (x + 1 + (x >> 8)) >> 8, no more than top.
.macro LEVEL c, top, dither, out, tmp
    li      \tmp, \top
    mul     \out, \c, \tmp
    addi    \out, \out, \dither
    srai    \tmp, \out, 8
    add     \out, \out, \tmp
    addi    \out, \out, 1
    srai    \out, \out, 8
    li      \tmp, \top
    blt     \out, \tmp, 95f
    mv      \out, \tmp
95:
.endm

// The hazed colour in s9, s10, s11 packed to 5:6:5, dithered `dither`,
// bytes swapped for the panel where it wants them, into `out`.
.macro PACK565 dither, out
    LEVEL   s9, 31, \dither, a1, a2
    slli    \out, a1, 11
    LEVEL   s10, 63, \dither, a1, a2
    slli    a1, a1, 5
    or      \out, \out, a1
    LEVEL   s11, 31, \dither, a1, a2
    or      \out, \out, a1
    lw      a1, T_SWAP(a0)          // swapped for the panel?
    beqz    a1, 96f
    srli    a1, \out, 8
    slli    \out, \out, 24
    srli    \out, \out, 16
    or      \out, \out, a1
96:
.endm

// The pairs of pixels for key t5 - cell | haze << 8 - worked out as
// pairInt() does, into a6 for even rows and a7 for odd, and kept in the
// cache at t2, slot t0, with the key: the cell's lit colour hazed `haze`
// << 3 of 256 towards the haze's, each channel; packed to 5:6:5 twice
// each, dithered 0 and 127 for the even rows' two pixels, 191 and 63 for
// the odd's. t4 and t5 kept; s9-s11, a1 and a2 kept on the stack.
.macro MAKE_PAIRS
    sw      s9, F_PAIRS(sp)
    sw      s10, F_PAIRS+4(sp)
    sw      s11, F_PAIRS+8(sp)
    sw      a1, F_PAIRS+12(sp)
    sw      a2, F_PAIRS+16(sp)
    slli    t3, t0, 1
    add     t3, t3, t2
    sh      t5, P_KEY(t3)           // the key, kept
    slli    t3, t0, 3
    add     t3, t3, t2              // its pairs, P_PAIR on
    andi    t0, t5, 255             // the cell
    srli    t1, t5, 8
    slli    t1, t1, 3               // the haze, of 256
    lw      a1, T_COVERRGB(a0)
    slli    a2, t0, 1
    add     a2, a2, t0
    add     a1, a1, a2              // the cell's lit colour
    lw      a2, T_HAZECOLOR(a0)     // the haze's
    lbu     s9, 0(a1)
    lbu     s10, 1(a1)
    lbu     s11, 2(a1)
    lw      t0, 0(a2)
    sub     t0, t0, s9
    mul     t0, t0, t1
    srai    t0, t0, 8
    add     s9, s9, t0              // red, hazed
    lw      t0, 4(a2)
    sub     t0, t0, s10
    mul     t0, t0, t1
    srai    t0, t0, 8
    add     s10, s10, t0            // green
    lw      t0, 8(a2)
    sub     t0, t0, s11
    mul     t0, t0, t1
    srai    t0, t0, 8
    add     s11, s11, t0            // blue
    PACK565 0, t0
    PACK565 127, t1
    slli    t1, t1, 16
    or      a6, t0, t1              // the even rows' pair
    sw      a6, P_PAIR(t3)
    PACK565 191, t0
    PACK565 63, t1
    slli    t1, t1, 16
    or      a7, t0, t1              // the odd rows'
    sw      a7, P_PAIR+4(t3)
    lw      t0, T_MISSES(a0)
    addi    t0, t0, 1
    sw      t0, T_MISSES(a0)        // pairs worked out
    lw      s9, F_PAIRS(sp)
    lw      s10, F_PAIRS+4(sp)
    lw      s11, F_PAIRS+8(sp)
    lw      a1, F_PAIRS+12(sp)
    lw      a2, F_PAIRS+16(sp)
.endm

// The map's fields in registers, from t; and back.
.macro MAP_FROM_T
    flw     fa6, T_AU(a0)
    flw     fa7, T_BU(a0)
    flw     ft8, T_AV(a0)
    flw     ft9, T_BV(a0)
    lw      s10, T_CELLS(a0)
    lw      s11, T_COLS(a0)
    lw      a1, T_ROWS(a0)
    lw      a2, T_MSTRIDE(a0)
    lw      a3, T_WLO(a0)
    lw      a4, T_WRANGE(a0)
.endm
.macro MAP_TO_T
    fsw     fa6, T_AU(a0)
    fsw     fa7, T_BU(a0)
    fsw     ft8, T_AV(a0)
    fsw     ft9, T_BV(a0)
    sw      s10, T_CELLS(a0)
    sw      s11, T_COLS(a0)
    sw      a1, T_ROWS(a0)
    sw      a2, T_MSTRIDE(a0)
    sw      a3, T_WLO(a0)
    sw      a4, T_WRANGE(a0)
.endm

// The run's samples since F_SEG, counted, up to s0.
.macro COUNT_RUN
    lw      t0, F_SEG(sp)
    sub     t0, s0, t0
    srli    t0, t0, 3
    lw      t1, T_SAMPLES(a0)
    add     t1, t1, t0
    sw      t1, T_SAMPLES(a0)
.endm

// The common way's two columns, flat: A at s0, s1, s4, B the column after;
// on to `miss` where either is off the map or another level's.
.macro FAST_FLAT miss
    lhu     t0, 0(s1)               // A's 1 / depth, bfloat16
    add     t1, s1, s6
    lhu     t1, 0(t1)               // B's
    flw     ft2, 0(s4)              // A's cu
    flw     ft3, 4(s4)              // A's cv
    flw     ft4, 8(s4)              // B's
    flw     ft5, 12(s4)
    sub     t2, t0, a3              // A's from its level's first
    slli    t0, t0, 16
    fmv.w.x ft0, t0
    sub     t3, t1, a3              // B's
    slli    t1, t1, 16
    fmv.w.x ft1, t1
    fdiv.s  ft0, ft10, ft0          // A's depth
    fadd.s  ft2, fs0, ft2           // rowU + cu
    fadd.s  ft3, fs1, ft3           // rowV + cv
    fadd.s  ft4, fs0, ft4
    fadd.s  ft5, fs1, ft5
    fdiv.s  ft1, ft10, ft1          // B's depth
    fmadd.s ft2, ft2, ft0, fa2      // A's u
    fmadd.s ft3, ft3, ft0, fa3      // A's v
    fmul.s  ft6, ft0, fs2           // A's distance
    bgeu    t2, a4, \miss           // A another level's
    bgeu    t3, a4, \miss           // B
    fmadd.s ft2, ft2, fa6, fa7      // A's map column, and 1024
    fmadd.s ft3, ft3, ft8, ft9      // A's row
    fmadd.s ft4, ft4, ft1, fa2      // B's u
    fmadd.s ft5, ft5, ft1, fa3      // B's v
    fmul.s  ft7, ft1, fs2           // B's distance
    fcvt.w.s t2, ft2, rtz
    fcvt.w.s t3, ft3, rtz
    fmv.x.w t0, ft6
    fmadd.s ft4, ft4, fa6, fa7
    fmadd.s ft5, ft5, ft8, ft9
    addi    t2, t2, -1024           // the 1024 off: a floor, not a cast's truncation
    addi    t3, t3, -1024
    bgeu    t2, s11, \miss          // A off the map's part
    bgeu    t3, a1, \miss
    mul     t3, t3, a2
    and     t0, t0, ra              // A's haze's index
    fmv.x.w t1, ft7
    add     t3, t3, t2
    add     t3, t3, s10
    lbu     t2, 0(t3)               // A's cell
    fcvt.w.s t4, ft4, rtz
    fcvt.w.s t5, ft5, rtz
    and     t1, t1, ra              // B's haze's index
    addi    t4, t4, -1024
    addi    t5, t5, -1024
    or      t0, t0, t2              // A's key'
    bgeu    t4, s11, \miss          // B off the map's part
    bgeu    t5, a1, \miss
    mul     t5, t5, a2
    add     t5, t5, t4
    add     t5, t5, s10
    lbu     t4, 0(t5)               // B's cell
.endm

// The same over the round cover: dx and dy east and north of the eye, z
// (rowX + cx) and z (rowY + cy); u = U0 + dx (U1 + U3 dx + U4 dy) + dy (U2
// + U5 dy), and v likewise - V3-V5 read from t, a register for the two
// columns' each - the four chains, u and v of each column, together.
.macro FAST_ROUND miss
    lhu     t0, 0(s1)               // A's 1 / depth, bfloat16
    add     t1, s1, s6
    lhu     t1, 0(t1)               // B's
    flw     ft2, 0(s4)              // A's cx
    flw     ft3, 4(s4)              // A's cy
    flw     ft4, 8(s4)              // B's
    flw     ft5, 12(s4)
    sub     t2, t0, a3              // A's from its level's first
    slli    t0, t0, 16
    fmv.w.x ft0, t0
    sub     t3, t1, a3              // B's
    slli    t1, t1, 16
    fmv.w.x ft1, t1
    fdiv.s  ft0, ft10, ft0          // A's depth
    fadd.s  ft2, fs0, ft2           // rowX + cx
    fadd.s  ft3, fs1, ft3           // rowY + cy
    fadd.s  ft4, fs0, ft4
    fadd.s  ft5, fs1, ft5
    flw     ft7, T_MAPV+12(a0)      // V3
    fdiv.s  ft1, ft10, ft1          // B's depth
    fmul.s  ft2, ft2, ft0           // A's dx
    fmul.s  ft3, ft3, ft0           // A's dy
    fmul.s  ft0, ft0, fs2           // A's distance
    bgeu    t2, a4, \miss           // A another level's
    bgeu    t3, a4, \miss           // B
    fmadd.s fa4, ft11, ft2, fa0     // A: U1 + U3 dx
    fmadd.s fa5, ft7, ft2, fs10     // A: V1 + V3 dx
    fmul.s  ft4, ft4, ft1           // B's dx
    fmul.s  ft5, ft5, ft1           // B's dy
    fmul.s  ft1, ft1, fs2           // B's distance
    fmadd.s fs4, fs9, ft3, fa1      // A: U2 + U5 dy
    fmadd.s fs5, ft11, ft4, fa0     // B: U1 + U3 dx
    fmadd.s fs6, ft7, ft4, fs10     // B: V1 + V3 dx
    flw     ft7, T_MAPV+16(a0)      // V4
    fmadd.s fa4, fs8, ft3, fa4      // A: + U4 dy
    fmadd.s fs3, fs9, ft5, fa1      // B: U2 + U5 dy
    fmadd.s fa5, ft7, ft3, fa5      // A: + V4 dy
    fmadd.s fs5, fs8, ft5, fs5      // B: + U4 dy
    fmadd.s fs6, ft7, ft5, fs6      // B: + V4 dy
    flw     ft7, T_MAPV+20(a0)      // V5
    fmadd.s fa4, ft2, fa4, fa2      // A: U0 + dx (...)
    fmadd.s fs7, ft7, ft3, fs11     // A: V2 + V5 dy
    fmadd.s fs5, ft4, fs5, fa2      // B: U0 + dx (...)
    fmadd.s ft7, ft7, ft5, fs11     // B: V2 + V5 dy
    fmadd.s fa5, ft2, fa5, fa3      // A: V0 + dx (...)
    fmadd.s fs6, ft4, fs6, fa3      // B: V0 + dx (...)
    fmadd.s fa4, ft3, fs4, fa4      // A's u: + dy (...)
    fmadd.s fs5, ft5, fs3, fs5      // B's u
    fmv.x.w t0, ft0                 // A's distance's bits
    fmadd.s fa5, ft3, fs7, fa5      // A's v
    fmadd.s fs6, ft5, ft7, fs6      // B's v
    fmv.x.w t1, ft1                 // B's
    and     t0, t0, ra              // A's haze's index
    fmadd.s fa4, fa4, fa6, fa7      // A's map column, and 1024
    and     t1, t1, ra              // B's haze's index
    fmadd.s fs5, fs5, fa6, fa7      // B's
    fmadd.s fa5, fa5, ft8, ft9      // A's row
    fmadd.s fs6, fs6, ft8, ft9      // B's
    fcvt.w.s t2, fa4, rtz
    fcvt.w.s t4, fs5, rtz
    fcvt.w.s t3, fa5, rtz
    fcvt.w.s t5, fs6, rtz
    addi    t2, t2, -1024           // the 1024 off: a floor, not a cast's truncation
    addi    t4, t4, -1024
    addi    t3, t3, -1024
    addi    t5, t5, -1024
    bgeu    t2, s11, \miss          // A off the map's part
    bgeu    t3, a1, \miss
    bgeu    t4, s11, \miss          // B
    bgeu    t5, a1, \miss
    mul     t3, t3, a2
    mul     t5, t5, a2
    add     t3, t3, t2
    add     t5, t5, t4
    add     t3, t3, s10
    add     t5, t5, s10
    lbu     t2, 0(t3)               // A's cell
    lbu     t4, 0(t5)               // B's cell
    or      t0, t0, t2              // A's key'
.endm

.macro TERRAIN_FN name, round, quad
    .text
    .align  2
    .global \name
    .type   \name, @function
\name:
    addi    sp, sp, -FRAME
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
    sw      ra, 48(sp)
    fsw     fs0, F_FSAVE(sp)
    fsw     fs1, F_FSAVE+4(sp)
    fsw     fs2, F_FSAVE+8(sp)
    fsw     fs3, F_FSAVE+12(sp)
    fsw     fs4, F_FSAVE+16(sp)
    fsw     fs5, F_FSAVE+20(sp)
    fsw     fs6, F_FSAVE+24(sp)
    fsw     fs7, F_FSAVE+28(sp)
    fsw     fs8, F_FSAVE+32(sp)
    fsw     fs9, F_FSAVE+36(sp)
    fsw     fs10, F_FSAVE+40(sp)
    fsw     fs11, F_FSAVE+44(sp)
    lw      s6, T_BANDBYTES(a0)
    lw      s8, T_YB(a0)
    lw      t0, T_STRIDE(a0)
    sw      t0, F_STRIDE(sp)
    lui     ra, 0xFFE00             // a distance's bits to its haze's index
.if \round
    flw     fa2, T_MAPU(a0)         // U0
    flw     fa0, T_MAPU+4(a0)       // U1
    flw     fa1, T_MAPU+8(a0)       // U2
    flw     ft11, T_MAPU+12(a0)     // U3
    flw     fs8, T_MAPU+16(a0)      // U4
    flw     fs9, T_MAPU+20(a0)      // U5
    flw     fa3, T_MAPV(a0)         // V0
    flw     fs10, T_MAPV+4(a0)      // V1
    flw     fs11, T_MAPV+8(a0)      // V2
.else
    flw     fa2, T_UE(a0)
    flw     fa3, T_VE(a0)
.endif
    li      t0, 0x3f800000
    fmv.w.x ft10, t0                // 1
    MAP_FROM_T
    lw      a6, T_PAIRE(a0)         // the parities' last pairs and keys
    lw      a7, T_PAIRO(a0)
    lw      t0, T_KEYE(a0)
    sw      t0, F_KEYE(sp)
    lw      t0, T_KEYO(a0)
    sw      t0, F_KEYO(sp)
    li      a5, -1                  // no key' to go by yet
    lw      t6, T_ROWAT(a0)         // the unit
    lw      s2, T_C(a0)             // and the column, it goes on from
.Lunit\@:  // the unit from row t6, from column s2
    bge     t6, s8, .Ldone\@
.if \quad
    andi    s9, t6, -4
    addi    s9, s9, 4
    blt     s9, s8, 1f
    mv      s9, s8                  // the unit's end in the band
1:
    andi    t0, t6, 3
    sub     t1, s9, t6
    xori    t1, t1, 4
    or      t0, t0, t1
    seqz    t0, t0                  // whole: four rows from a multiple of four
.else
    andi    t0, t6, 1
    addi    s9, t6, 1               // an odd row alone, a band's first
    bnez    t0, 2f
    addi    s9, t6, 2
    blt     s9, s8, 1f
    mv      s9, s8                  // or an even one alone, its last
1:
    sub     t0, s9, t6
    addi    t0, t0, -2
2:
    seqz    t0, t0                  // whole: a pair
.endif
    sw      t0, F_WHOLE(sp)
    lw      t0, T_TOP(a0)
    bge     t0, s9, .Lunit_next\@   // no land in it in any column
    slli    t1, t6, 2
    lw      t0, T_ROWU(a0)
    add     t0, t0, t1
    flw     fs0, 0(t0)              // the row's rowU (round, rowX)
    lw      t0, T_ROWV(a0)
    add     t0, t0, t1
    flw     fs1, 0(t0)              // rowV (rowY)
    lw      t0, T_ROWFAR(a0)
    add     t0, t0, t1
    flw     fs2, 0(t0)              // rowFar
    lw      t1, T_Y0(a0)
    sub     t1, t6, t1              // the row in the band
    lw      t0, F_STRIDE(sp)
    mul     t2, t1, t0
    lw      s5, T_COLOR(a0)
    add     s5, s5, t2              // its pixels, column nought's
    lw      t0, T_ZS(a0)
    slli    t1, t1, 1
    add     t0, t0, t1
    sw      t0, F_ZROW(sp)          // its 1 / depths, column nought's
.Lstrip\@:  // the strip of column s2, from s2: no land in the unit, all of it the common way, or some
    lw      t0, T_BANDCOLS(a0)
    bge     s2, t0, .Lunit_next\@
    srli    t1, s2, 3
    slli    t1, t1, 3               // the strip's first column - and its offset, eight bytes a strip
    lw      t2, T_STRIPS(a0)
    add     t2, t2, t1
    addi    t3, t1, 8
    blt     t3, t0, 1f
    mv      t3, t0                  // its end
1:
    lh      t4, 0(t2)               // the least of its tops
    bge     t4, s9, .Lstrip_none\@  // no land in the unit
    lw      t4, F_WHOLE(sp)
    beqz    t4, .Lstrip_some\@      // not a whole unit: the general way
    lh      t4, 2(t2)               // the most of its tops
    blt     t6, t4, .Lstrip_some\@  // land starting in the unit
    lh      t4, 4(t2)               // the least of its flat tops
    blt     t4, s9, .Lstrip_some\@  // the airfield's flat ground in it
2:  // the common way, on through the strips after it that go the same
    bge     t3, t0, 3f
    addi    t2, t2, 8
    lh      t4, 2(t2)
    blt     t6, t4, 3f
    lh      t4, 4(t2)
    blt     t4, s9, 3f
    addi    t3, t3, 8
    j       2b
3:
    blt     t3, t0, 4f
    mv      t3, t0                  // the run's end
4:
    slli    t1, s2, 3
    add     s0, s5, t1              // A's pixels
    lw      t1, F_STRIDE(sp)
    add     s3, s0, t1              // and the row under's
    slli    t1, t3, 3
    add     s7, s5, t1
    sw      s7, F_END(sp)           // past the run's last
    addi    s7, s7, -8              // its last with a column after it
    lw      t0, F_ZROW(sp)
    mul     t1, s2, s6
    add     s1, t0, t1              // A's 1 / depth
    lw      t0, T_COLUV(a0)
    slli    t1, s2, 3
    add     s4, t0, t1              // A's cu and cv
    sw      s0, F_SEG(sp)
    j       .Lf_enter\@
.Lstrip_none\@:
    mv      s2, t3
    j       .Lstrip\@
.Lstrip_some\@:  // the general way, to the strip's end
    mv      s7, t3
    slli    t0, s2, 3
    add     s0, s5, t0
    lw      t0, F_ZROW(sp)
    mul     t1, s2, s6
    add     s1, t0, t1
    lw      t0, T_COLUV(a0)
    slli    t1, s2, 3
    add     s4, t0, t1
    jal     s3, .Lgen\@
    j       .Lstrip\@
.Lunit_next\@:
    mv      t6, s9
    li      s2, 0
    j       .Lunit\@

// The common way, two columns a turn.
.Lf_enter\@:
.if \quad
    lw      t1, F_STRIDE(sp)
    add     s8, s3, t1              // the unit's third row
    add     s9, s8, t1              // and fourth
.else
    slli    s8, s6, 1               // two columns' 1 / depths
.endif
    bgeu    s0, s7, .Lf_tail\@      // under two columns
.Lf2\@:
.if \round
    FAST_ROUND .Lf_miss\@
.else
    FAST_FLAT .Lf_miss\@
.endif
    bne     t0, a5, .Lf_keyA\@      // A's key' not the last's
.Lf_backA\@:
    sw      a6, 0(s0)               // A's even row's four pixels
    sw      a6, 4(s0)
    sw      a7, 0(s3)               // and the odd's
    sw      a7, 4(s3)
.if \quad
    sw      a6, 0(s8)
    sw      a6, 4(s8)
    sw      a7, 0(s9)
    sw      a7, 4(s9)
.endif
.Lf_afterA\@:
    or      t1, t1, t4              // B's key'
    bne     t1, a5, .Lf_keyB\@
.Lf_backB\@:
    sw      a6, 8(s0)
    sw      a6, 12(s0)
    sw      a7, 8(s3)
    sw      a7, 12(s3)
.if \quad
    sw      a6, 8(s8)
    sw      a6, 12(s8)
    sw      a7, 8(s9)
    sw      a7, 12(s9)
.endif
.Lf_afterB\@:
    addi    s0, s0, 16
    addi    s3, s3, 16
    addi    s4, s4, 16
.if \quad
    addi    s8, s8, 16
    addi    s9, s9, 16
    add     s1, s1, s6
    add     s1, s1, s6
.else
    add     s1, s1, s8
.endif
    bltu    s0, s7, .Lf2\@
.Lf_tail\@:  // the run's last column, if one is left, the general way
    COUNT_RUN
    lw      s8, T_YB(a0)
.if \quad
    addi    s9, t6, 4
.endif
    sub     s2, s0, s5
    srli    s2, s2, 3               // its column
    lw      t0, F_END(sp)
    bgeu    s0, t0, .Lstrip\@       // none
    addi    s7, s2, 1
    jal     s3, .Lgen\@
    j       .Lstrip\@
.Lf_miss\@:  // A or B off the map, or another level's: both the general way, then on
    COUNT_RUN
    lw      s8, T_YB(a0)
.if \quad
    addi    s9, t6, 4
.endif
    sub     s2, s0, s5
    srli    s2, s2, 3
    addi    s7, s2, 2
    jal     s3, .Lgen\@
    lw      s7, F_END(sp)
    addi    s7, s7, -8
    lw      t1, F_STRIDE(sp)
    add     s3, s0, t1
    sw      s0, F_SEG(sp)
    j       .Lf_enter\@
.Lf_keyA\@:  // A's key' not the last's: its key, and its pairs - nothing there, no pixels
    or      t1, t1, t4              // B's key', kept over A's pairs
    sw      t1, F_KB(sp)
    andi    t3, t0, 255             // A's cell
    sltiu   t2, t3, 16
    bnez    t2, .Lf_afterA\@        // nothing there: the rows' own colours
    sw      t0, F_KP(sp)
    srli    t2, t0, 21
    lw      t5, T_HAZE(a0)
    add     t2, t2, t5
    lbu     t2, 0(t2)               // its haze for the distance
    slli    t2, t2, 8
    or      t5, t3, t2              // cell | haze << 8
    jal     t4, .Lpairs\@
    lw      a5, F_KP(sp)            // its key' the last's
    lw      t1, F_KB(sp)
    andi    t4, t1, 255             // B's cell again
    j       .Lf_backA\@
.Lf_keyB\@:  // B's likewise
    andi    t3, t1, 255
    sltiu   t2, t3, 16
    bnez    t2, .Lf_afterB\@
    sw      t1, F_KP(sp)
    srli    t2, t1, 21
    lw      t5, T_HAZE(a0)
    add     t2, t2, t5
    lbu     t2, 0(t2)
    slli    t2, t2, 8
    or      t5, t3, t2
    jal     t4, .Lpairs\@
    lw      a5, F_KP(sp)
    j       .Lf_backB\@

// The pairs for key t5 into a6 and a7, and it the last of both parities:
// as they are where it is already, from the cache, or worked out. Back to
// t4.
.Lpairs\@:
    lw      t0, F_KEYE(sp)
    bne     t5, t0, 1f
    lw      t0, F_KEYO(sp)
    beq     t5, t0, 3f              // both parities' last: as they are
1:
    andi    t0, t5, 255             // the cell
    srli    t1, t5, 8               // the haze
    li      t2, 37
    mul     t1, t1, t2
    xor     t0, t0, t1
    andi    t0, t0, 127             // its slot: (cell ^ haze 37) & 127
    lw      t2, T_PAIRS(a0)
    slli    t1, t0, 1
    add     t1, t1, t2
    lhu     t1, P_KEY(t1)           // the slot's key
    bne     t1, t5, 2f              // another's: worked out
    slli    t1, t0, 3
    add     t1, t1, t2
    lw      a6, P_PAIR(t1)
    lw      a7, P_PAIR+4(t1)
    j       4f
2:
    MAKE_PAIRS
4:
    sw      t5, F_KEYE(sp)
    sw      t5, F_KEYO(sp)
3:
    jr      t4

// The general way: columns s2 to s7 of the unit, a sample at a time, from
// s0, s1, s4 at column s2 - each column with land in the unit sampled on
// the unit's first row, or on its own first in it, the rows from there to
// the unit's end. Back to s3.
.Lgen\@:
    bge     s2, s7, .Lg_ret\@
    lw      t0, T_TOPFLAT(a0)
    slli    t1, s2, 2
    add     t0, t0, t1
    lw      t0, 0(t0)               // top | flat top << 16
    slli    t1, t0, 16
    srai    t1, t1, 16              // the column's highest row of land
    bge     t1, s9, .Lg_next\@      // none in the unit
    srai    t2, t0, 16              // its flat top
    mv      t3, t6
    bge     t6, t1, .Lg_row\@
    mv      t3, t1                  // its land from a row in the unit: its sample there
.Lg_row\@:
    sw      t3, F_ROW(sp)
    slli    t4, t3, 2
    lw      t5, T_ROWU(a0)
    add     t5, t5, t4
    flw     fs4, 0(t5)              // the row's rowU (round, rowX)
    lw      t5, T_ROWV(a0)
    add     t5, t5, t4
    flw     fs5, 0(t5)              // rowV (rowY)
    lw      t5, T_ROWFAR(a0)
    add     t5, t5, t4
    flw     fs6, 0(t5)              // rowFar
    blt     t3, t2, .Lg_w\@
    lw      t5, T_FLATZ(a0)
    add     t5, t5, t4
    flw     ft0, 0(t5)              // the airfield's flat ground: the row's own depth
    j       .Lg_z\@
.Lg_w\@:
    sub     t4, t3, t6
    slli    t4, t4, 1
    add     t4, t4, s1
    lhu     t4, 0(t4)               // 1 / depth, bfloat16
    slli    t4, t4, 16
    fmv.w.x ft0, t4
    fdiv.s  ft0, ft10, ft0          // the depth
.Lg_z\@:
    flw     ft2, 0(s4)              // cu - round, cx
    flw     ft3, 4(s4)              // cv - round, cy
    fadd.s  ft2, fs4, ft2           // rowU + cu
    fadd.s  ft3, fs5, ft3           // rowV + cv
.if \round
    fmul.s  ft2, ft2, ft0           // dx
    fmul.s  ft3, ft3, ft0           // dy
    fmadd.s ft4, ft11, ft2, fa0     // U1 + U3 dx
    fmadd.s ft4, fs8, ft3, ft4      // + U4 dy
    fmadd.s ft5, fs9, ft3, fa1      // U2 + U5 dy
    fmadd.s ft4, ft2, ft4, fa2      // U0 + dx (...)
    fmadd.s ft4, ft3, ft5, ft4      // + dy (...): u
    flw     ft6, T_MAPV+12(a0)      // V3
    flw     ft7, T_MAPV+16(a0)      // V4
    fmadd.s ft5, ft6, ft2, fs10     // V1 + V3 dx
    fmadd.s ft5, ft7, ft3, ft5      // + V4 dy
    flw     ft6, T_MAPV+20(a0)      // V5
    fmadd.s ft6, ft6, ft3, fs11     // V2 + V5 dy
    fmadd.s ft5, ft2, ft5, fa3      // V0 + dx (...)
    fmadd.s ft5, ft3, ft6, ft5      // + dy (...): v
.else
    fmadd.s ft4, ft2, ft0, fa2      // u
    fmadd.s ft5, ft3, ft0, fa3      // v
.endif
    fmadd.s ft2, ft4, fa6, fa7      // the map's column, and 1024
    fmadd.s ft3, ft5, ft8, ft9      // its row, and 1024
    flw     ft6, T_ZLO(a0)
    flw     ft7, T_ZHI(a0)
    flt.s   t0, ft6, ft0            // the depth over zLo...
    fle.s   t1, ft0, ft7            // ...and to zHi: the map's level
    fcvt.w.s t4, ft2, rtz
    fcvt.w.s t5, ft3, rtz
    and     t0, t0, t1
    addi    t4, t4, -1024
    addi    t5, t5, -1024
    beqz    t0, .Lg_switch\@        // another level's
    bgeu    t4, s11, .Lg_switch\@   // off the map's part
    bgeu    t5, a1, .Lg_switch\@
    mul     t5, t5, a2
    add     t5, t5, t4
    add     t5, t5, s10
    lbu     t0, 0(t5)               // the cell
.Lg_cell\@:  // the cell t0, its depth ft0: its haze, its key, its pairs, its rows
    fmul.s  ft6, ft0, fs6           // the distance
    lw      t1, T_SAMPLES(a0)
    addi    t1, t1, 1
    sw      t1, T_SAMPLES(a0)       // a sample
    fmv.x.w t1, ft6
    sltiu   t2, t0, 16
    bnez    t2, .Lg_next\@          // nothing there: the rows' own colours
    and     t2, t1, ra
    or      t2, t2, t0
    sw      t2, F_KP(sp)            // its key'
    srli    t1, t1, 21
    lw      t2, T_HAZE(a0)
    add     t1, t1, t2
    lbu     t1, 0(t1)               // the haze for the distance
    slli    t1, t1, 8
    or      t5, t0, t1              // cell | haze << 8
    jal     t4, .Lpairs\@
    lw      a5, F_KP(sp)            // its key' the last's
    lw      t0, F_ROW(sp)
    lw      t1, F_STRIDE(sp)
    sub     t2, t0, t6
    mul     t2, t2, t1
    add     t2, t2, s0              // its row's pixels
.Lg_store\@:  // its rows to the unit's end, each its parity's pair
    andi    t3, t0, 1
    mv      t4, a6
    beqz    t3, .Lg_even\@
    mv      t4, a7
.Lg_even\@:
    sw      t4, 0(t2)
    sw      t4, 4(t2)
    add     t2, t2, t1
    addi    t0, t0, 1
    blt     t0, s9, .Lg_store\@
.Lg_next\@:
    addi    s0, s0, 8
    add     s1, s1, s6
    addi    s4, s4, 8
    addi    s2, s2, 1
    j       .Lgen\@
.Lg_ret\@:
    jr      s3
.Lg_switch\@:  // off the map: the part cellAt() would read it in, among those kept - see cellSlow()
    lw      t0, T_MAPS(a0)
    lw      t1, MS_COUNT(t0)
    addi    t2, t0, MS_MAP          // the first
    slli    t3, t1, 6
    slli    t1, t1, 5
    add     t1, t1, t3
    add     t3, t2, t1              // past the last
.Lg_scan\@:
    bgeu    t2, t3, .Lg_hand\@      // in none of them: C
    flw     ft6, M_ZLO(t2)
    flw     ft7, M_ZHI(t2)
    flt.s   t0, ft6, ft0
    fle.s   t1, ft0, ft7
    and     t0, t0, t1
    beqz    t0, .Lg_scan_next\@     // another level's
    flw     ft6, M_SPLIT(t2)
    fmul.s  fa4, ft4, ft6           // u in the level's units
    fmul.s  fa5, ft5, ft6           // v
    flw     ft6, M_U0(t2)
    flw     ft7, M_U1(t2)
    fle.s   t0, ft6, fa4
    flt.s   t1, fa4, ft7
    and     t0, t0, t1
    beqz    t0, .Lg_scan_next\@
    flw     ft6, M_V0(t2)
    flw     ft7, M_V1(t2)
    fle.s   t0, ft6, fa5
    flt.s   t1, fa5, ft7
    and     t0, t0, t1
    bnez    t0, .Lg_found\@         // its part
.Lg_scan_next\@:
    addi    t2, t2, M_SIZE
    j       .Lg_scan\@
.Lg_found\@:  // the map taken up: its fields where the loops have them, its level's depths in t
    flw     fa6, M_AU(t2)
    flw     fa7, M_BU(t2)
    flw     ft8, M_AV(t2)
    flw     ft9, M_BV(t2)
    lw      s10, M_CELLS(t2)
    lw      s11, M_COLS(t2)
    lw      a1, M_ROWS(t2)
    lw      a2, M_STRIDE(t2)
    lw      a3, M_WLO(t2)
    lw      a4, M_WRANGE(t2)
    flw     ft6, M_ZLO(t2)
    fsw     ft6, T_ZLO(a0)
    flw     ft6, M_ZHI(t2)
    fsw     ft6, T_ZHI(a0)
    lw      t0, T_SWITCHES(a0)
    addi    t0, t0, 1
    sw      t0, T_SWITCHES(a0)
    // the cell as the map reads it, where it reads this one; else as the part does
    fmadd.s ft2, ft4, fa6, fa7
    fmadd.s ft3, ft5, ft8, ft9
    fcvt.w.s t4, ft2, rtz
    fcvt.w.s t5, ft3, rtz
    addi    t4, t4, -1024
    addi    t5, t5, -1024
    bgeu    t4, s11, .Lg_part\@
    bgeu    t5, a1, .Lg_part\@
    mul     t5, t5, a2
    add     t5, t5, t4
    add     t5, t5, s10
    lbu     t0, 0(t5)
    j       .Lg_cell\@
.Lg_part\@:  // as cellAt() reads the part: min(cols1, (uu - u0) ku), and the row likewise
    flw     ft6, M_U0(t2)
    flw     ft7, M_KU(t2)
    fsub.s  ft6, fa4, ft6
    fmul.s  ft6, ft6, ft7
    fcvt.w.s t4, ft6, rtz
    lw      t0, M_COLS1(t2)
    blt     t4, t0, .Lg_pc\@
    mv      t4, t0
.Lg_pc\@:
    flw     ft6, M_V0(t2)
    flw     ft7, M_KV(t2)
    fsub.s  ft6, fa5, ft6
    fmul.s  ft6, ft6, ft7
    fcvt.w.s t5, ft6, rtz
    lw      t0, M_ROWS1(t2)
    blt     t5, t0, .Lg_pr\@
    mv      t5, t0
.Lg_pr\@:
    mul     t5, t5, a2
    add     t5, t5, t4
    add     t5, t5, s10
    lbu     t0, 0(t5)
    j       .Lg_cell\@
.Lg_hand\@:  // back to C at the sample's row, with what it needs; on from the next column
    lw      t3, F_ROW(sp)
    lw      t1, F_STRIDE(sp)
    sub     t2, t3, t6
    mul     t2, t2, t1
    add     t2, t2, s0
    sw      t2, T_PX(a0)            // its pixels on its row
    slli    t0, t6, 1
    sub     t0, s1, t0
    sw      t0, T_ZC(a0)            // its 1 / depths, by row
    sw      t3, T_Y(a0)
    lw      t0, T_TOPFLAT(a0)
    slli    t1, s2, 2
    add     t0, t0, t1
    lw      t0, 0(t0)
    srai    t0, t0, 16
    sw      t0, T_FLATTOP(a0)
    flw     ft6, 0(s4)
    flw     ft7, 4(s4)
    fsw     ft6, T_CU(a0)
    fsw     ft7, T_CV(a0)
    lw      t0, F_KEYE(sp)
    sw      t0, T_KEYE(a0)
    sw      a6, T_PAIRE(a0)
    lw      t0, F_KEYO(sp)
    sw      t0, T_KEYO(a0)
    sw      a7, T_PAIRO(a0)
    MAP_TO_T
    addi    t0, s2, 1
    sw      t0, T_C(a0)             // on from the next column
    sw      t6, T_ROWAT(a0)         // in this unit
    li      t5, 1
    j       .Lout\@
.Ldone\@:  // the band done
    li      t5, 0
.Lout\@:
    mv      a0, t5
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
    lw      ra, 48(sp)
    flw     fs0, F_FSAVE(sp)
    flw     fs1, F_FSAVE+4(sp)
    flw     fs2, F_FSAVE+8(sp)
    flw     fs3, F_FSAVE+12(sp)
    flw     fs4, F_FSAVE+16(sp)
    flw     fs5, F_FSAVE+20(sp)
    flw     fs6, F_FSAVE+24(sp)
    flw     fs7, F_FSAVE+28(sp)
    flw     fs8, F_FSAVE+32(sp)
    flw     fs9, F_FSAVE+36(sp)
    flw     fs10, F_FSAVE+40(sp)
    flw     fs11, F_FSAVE+44(sp)
    addi    sp, sp, FRAME
    ret
    .size   \name, .-\name
.endm

    TERRAIN_FN scene3d_terrain_band, 0, 0
    TERRAIN_FN scene3d_terrain_band_round, 1, 0
    TERRAIN_FN scene3d_terrain_quad, 0, 1
    TERRAIN_FN scene3d_terrain_quad_round, 1, 1

// int scene3d_march_flat(MarchState *m);
//
// One column's march over flat ground - scene3d.cpp's marchColumn<false>,
// fused as it is, to the bit - from step m->i, for as long as its samples go
// the common ways. Each step: its place on the grid, x = X0 + s XS and y
// likewise, as integers in 256ths of a cell - its cell and where in it at
// once; the line of sight through the top filled row there, base + s gk;
// and its cell's highest corner set against that: under it, the sample is
// hidden, which most are, in some thirty instructions. Otherwise its height
// between its cell's four corners, in integers, and set against the line
// again; above it, the sample is put on the screen - its depth, 1 / it, and
// its row - and the rows between it and the top filled one given their
// 1 / depth, straight between this sample's and the one before's, bfloat16;
// then the line of sight through the new top row. Every sixteenth step it
// gives up if the highest land there is couldn't rise above the rows filled.
// In the ring where an airfield's ground eases into the land's, the height
// is eased as C eases it.
//
// Into another block of the grid near the eye, the line sloped, it skips the
// block whole where its highest can't reach the line of sight anywhere
// across it; and a sample seen whose sample before was hidden by its cell
// has that one's height worked out again, for the rows between.
//
// It returns 0 when the column is done - its steps run out, its top row
// filled, or nothing further could be seen - and 1 at a step it leaves for
// C: a sample seen whose sample before is the airfield's flat ground's
// edge, once a column at most. Either way m holds where the march is.
//
// MarchState's fields, by offset (static_assert'ed in scene3d.cpp): 0 far
// (the bases 1600 bytes on), 4 steps, 8 heights, 12 cellMaxBytes, 16
// rowBytes, 20 xLim, 24 yLim, 28 bcols, 32 Fz, 36 Dz, 40 f, 44 cy - 0.5,
// 48 cy + 0.5, 52 zHigh, 56 skipTo, 60 tz, 64 bandRows - 1, 68 bandBack,
// 72 X0, 76 XS, 80 Y0, 84 YS, 88 A, 92 B, 96 iEaseLo, 100 iEaseHi, 104 e2,
// 108 eh2, 112 groundUp, 116 2500, 120 1/2500, 124 1e-3, 128 -1e-4, 132
// -inf, 136 3, 140 i, 144 filled, 148 at, 152 off, 156 k, 160 gk, 164
// gkLine, 168 sloped, 172 prev, 176 known, 180 shown, 184 knownH, 188
// shownRho, 192 shownW, 196 curBlk, 200 samples, 204 1/XS, 208 1/YS, 212
// groundUp + ez, 216 fall, 220 2 fall, 224 easeLo, 228 easeHi, 232
// blockMax, 236 the ring's first step, kept, 240 skips, 244 1e9, 248 the
// column's highest land from each sixteenth step on.
//
// Registers: s0 &far[i], s1 i, s2 steps, s3 heights, s4 cellMaxBytes, s5
// rowBytes, s6 xLim, s7 yLim, s8 curBlk, s9 bcols, s10 filled, s11 prev,
// a1 known, a2 shown, a3 iEaseLo, a4 samples, a5 &tz[at], a6 off, a7 tz;
// fa0-fa3 X0, XS, Y0, YS, fa4 gkLine, fa5 A, fa6 B, fa7 Fz, ft8 Dz, ft9 f,
// ft10 cy - 0.5, ft11 k, fs0 knownH, fs1 shownRho, fs2 shownW, fs3 zHigh,
// fs4 0.5, fs5 1, fs6 skipTo, fs7 1/65536, fs8 gk, fs9 groundUp, fs10 0.
// It calls nothing: s0-s11 and fs0-fs10 saved, the rest free.

// One row up: in the band, or to the band before's last.
.macro FILL_UP
    beqz    a6, 97f
    addi    a6, a6, -1
    addi    a5, a5, -2
    j       98f
97:
    mv      a6, t2
    add     a5, a5, t3
98:
.endm
.macro FILL_STORE
    fmv.x.w t4, ft4
    add     t4, t4, t6
    srli    t4, t4, 16
    sh      t4, 0(a5)
.endm

.macro MARCH_FN name, round
    .align  2
    .global \name
    .type   \name, @function
\name:
    addi    sp, sp, -96
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
    lw      s1, 140(a0)             // i
    lw      t0, 0(a0)               // far
    slli    t1, s1, 2
    add     s0, t0, t1              // &far[i]
    lw      s2, 4(a0)               // steps
    lw      s3, 8(a0)               // heights
    lw      s4, 12(a0)              // the cells' highest, so many bytes on
    lw      s5, 16(a0)              // a row of the grid's bytes
    lw      s6, 20(a0)              // xLim
    lw      s7, 24(a0)              // yLim
    lw      s9, 28(a0)              // bcols
    flw     fa7, 32(a0)             // Fz
    flw     ft8, 36(a0)             // Dz
    flw     ft9, 40(a0)             // f
    flw     ft10, 44(a0)            // cy - 0.5
    flw     fs3, 52(a0)             // zHigh
    flw     fs6, 56(a0)             // skipTo
    lw      a7, 60(a0)              // tz
    flw     fa0, 72(a0)             // X0
    flw     fa1, 76(a0)             // XS
    flw     fa2, 80(a0)             // Y0
    flw     fa3, 84(a0)             // YS
    flw     fa5, 88(a0)             // A
    flw     fa6, 92(a0)             // B
    lw      a3, 96(a0)              // iEaseLo
    flw     fs9, 112(a0)            // groundUp
    lw      s10, 144(a0)            // filled
    lw      t0, 148(a0)             // at
    slli    t0, t0, 1
    add     a5, a7, t0              // &tz[at]
    lw      a6, 152(a0)             // off
    flw     ft11, 156(a0)           // k
    flw     fs8, 160(a0)            // gk
    flw     fa4, 164(a0)            // gkLine
.if \round
    flw     fs3, 252(a0)            // XS2
    flw     fs11, 256(a0)           // YS2
    flw     ft0, 260(a0)            // the column's part of the fall
    fadd.s  fa4, fa4, ft0           // the line test's slope
.endif
    lw      s11, 172(a0)            // prev
    lw      a1, 176(a0)             // known
    lw      a2, 180(a0)             // shown
    flw     fs0, 184(a0)            // knownH
    flw     fs1, 188(a0)            // shownRho
    flw     fs2, 192(a0)            // shownW
    lw      s8, 196(a0)             // curBlk
    lw      a4, 200(a0)             // samples
    li      t0, 0x3f000000
    fmv.w.x fs4, t0                 // 0.5
    li      t0, 0x3f800000
    fmv.w.x fs5, t0                 // 1
    li      t0, 0x37800000
    fmv.w.x fs7, t0                 // 1 / 65536
    fmv.w.x fs10, zero              // 0
    bge     s1, s2, .Lm_done\@y
.Lm_step\@y:
    flw     ft0, 0(s0)              // s
    flw     ft1, 1600(s0)           // its base
.if \round
    fmadd.s ft2, ft0, fs3, fa1
    fmadd.s ft2, ft0, ft2, fa0      // x = X0 + s (XS + s XS2)
    fmadd.s ft3, ft0, fs11, fa3
    fmadd.s ft3, ft0, ft3, fa2      // y
.else
    fmadd.s ft2, ft0, fa1, fa0      // x
    fmadd.s ft3, ft0, fa3, fa2      // y
.endif
    fmadd.s ft4, ft0, fa4, ft1      // the line of sight: base + s gk
    fcvt.w.s t0, ft2, rtz           // in 256ths of a cell
    fcvt.w.s t1, ft3, rtz
    bgeu    t0, s6, .Lm_outside\@y     // off the grid
    bgeu    t1, s7, .Lm_outside\@y
    srai    t2, t1, 11
    mul     t2, t2, s9
    srai    t3, t0, 11
    add     t2, t2, t3              // its block
    bne     t2, s8, .Lm_block\@y
.Lm_cell\@y:
    srai    t3, t1, 8
    mul     t3, t3, s5
    srai    t4, t0, 8
    slli    t4, t4, 1
    add     t3, t3, t4
    add     t3, t3, s3              // &heights[its cell]
    add     t4, t3, s4
    lh      t4, 0(t4)               // its cell's highest corner
    fcvt.s.w ft5, t4
    bge     s1, a3, .Lm_ring\@y        // maybe in the airfield's ring
.Lm_most\@y:
    flt.s   t4, ft5, ft4
    beqz    t4, .Lm_between\@y         // it reaches the line: its height worked out
    addi    a4, a4, 1               // a sample, hidden by its cell
    mv      s11, s1                 // the sample before the next
.Lm_next\@y:
    andi    t4, s1, 15
    xori    t4, t4, 15
    beqz    t4, .Lm_giveup\@y          // a sixteenth step
.Lm_on\@y:
    addi    s1, s1, 1
    addi    s0, s0, 4
    blt     s1, s2, .Lm_step\@y
.Lm_done\@y:
    li      t6, 0
    j       .Lm_save\@y

.Lm_between\@y:  // its height between its cell's corners, 65536ths of a metre
    lh      t4, 0(t3)               // (x, y)
    lh      t5, 2(t3)               // (x + 1, y)
    add     t6, t3, s5
    lh      t2, 0(t6)               // (x, y + 1)
    lh      t6, 2(t6)               // (x + 1, y + 1)
    andi    t0, t0, 255
    andi    t1, t1, 255
    sub     t5, t5, t4
    mul     t5, t5, t0
    slli    t4, t4, 8
    add     t4, t4, t5              // along the top
    sub     t6, t6, t2
    mul     t6, t6, t0
    slli    t2, t2, 8
    add     t2, t2, t6              // along the bottom
    sub     t2, t2, t4
    mul     t2, t2, t1
    slli    t4, t4, 8
    add     t4, t4, t2              // between them
    fcvt.s.w ft5, t4
    fmul.s  ft5, ft5, fs7           // metres
.Lm_compare\@y:  // its height in ft5
    addi    a4, a4, 1               // a sample
    flt.s   t4, ft5, ft4
    beqz    t4, .Lm_seen\@y            // above the line of sight
.Lm_known\@y:
    mv      s11, s1                 // the sample before the next...
    mv      a1, s1                  // ...its height known
    fmv.s   fs0, ft5
    j       .Lm_next\@y

.Lm_seen\@y:  // put on the screen: its depth, 1 / it, and its row
.if \round
    flw     ft6, 260(a0)
    fmadd.s ft6, ft0, ft6, ft1      // the base: base[i] + s eh2f
    fsub.s  ft6, ft5, ft6           // its height over it
.else
    fsub.s  ft6, ft5, ft1           // its height over the base
.endif
    fmul.s  ft7, ft6, fa7
    fmadd.s ft7, ft0, fa5, ft7      // zc = s A + dz Fz
    fmul.s  ft2, ft6, ft8
    fmadd.s ft2, ft0, fa6, ft2      // yc = s B + dz Dz
    flt.s   t4, fs4, ft7
    beqz    t4, .Lm_known\@y           // behind the eye
    lw      t4, 168(a0)             // the line sloped?
    bnez    t4, 1f
    fmul.s  ft3, ft9, ft2           // not: the row's own test
    fmul.s  ft6, ft11, ft7
    fle.s   t4, ft3, ft6
    beqz    t4, .Lm_known\@y
1:
    fdiv.s  ft3, fs5, ft7           // w = 1 / zc
    fmul.s  ft2, ft2, ft3
    fmadd.s ft2, ft2, ft9, ft10     // its row: yc w f + cy - 0.5
    fcvt.w.s t0, ft2, rup           // the first whole row it reaches
    bgez    t0, 2f
    li      t0, 0
2:
    bge     t0, s10, .Lm_shown\@y      // none above those filled
    // The sample before, on the screen: as it was put there, or from its
    // height.
    beq     s11, a2, .Lm_before_shown\@y
    bltz    s11, .Lm_before_edge\@y
    bne     s11, a1, .Lm_before_height\@y  // its height not known: worked out again
    fmv.s   ft1, fs0                // its height
.Lm_before_project\@y:  // the sample before, its height in ft1
    sub     t1, s11, s1
    slli    t1, t1, 2
    add     t1, t1, s0              // &far[prev]
    flw     ft4, 0(t1)
    flw     ft6, 1600(t1)
.if \round
    flw     ft7, 260(a0)
    fmadd.s ft6, ft4, ft7, ft6      // its base: base[prev] + s eh2f
.endif
    fsub.s  ft6, ft1, ft6           // its height over its base
    fmul.s  ft7, ft6, fa7
    fmadd.s ft7, ft4, fa5, ft7      // its zc
    fmul.s  ft1, ft6, ft8
    fmadd.s ft1, ft4, fa6, ft1      // its yc
    flt.s   t4, fs4, ft7
    beqz    t4, .Lm_level\@y           // behind the eye: no slope
    fdiv.s  ft6, fs5, ft7           // its w
    fmul.s  ft1, ft1, ft6
    fmadd.s ft1, ft1, ft9, ft10     // its row
    j       .Lm_slope\@y
.Lm_before_edge\@y:
    li      t4, -1
    beq     s11, t4, .Lm_level\@y      // none: no slope
    j       .Lm_hand_counted\@y        // the flat ground's edge: C
.Lm_before_height\@y:  // its height again, as heightAt() has it - t0, top, kept
    sub     t1, s11, s1
    slli    t1, t1, 2
    add     t1, t1, s0
    flw     ft4, 0(t1)              // its distance
.if \round
    fmadd.s ft6, ft4, fs3, fa1
    fmadd.s ft6, ft4, ft6, fa0      // its x
    fmadd.s ft7, ft4, fs11, fa3
    fmadd.s ft7, ft4, ft7, fa2      // its y
.else
    fmadd.s ft6, ft4, fa1, fa0      // its x
    fmadd.s ft7, ft4, fa3, fa2      // its y
.endif
    fcvt.w.s t1, ft6, rtz
    fcvt.w.s t2, ft7, rtz
    fmv.w.x ft1, zero               // off the grid: the sea's
    bgeu    t1, s6, 23f
    bgeu    t2, s7, 23f
    srai    t3, t2, 8
    mul     t3, t3, s5
    srai    t4, t1, 8
    slli    t4, t4, 1
    add     t3, t3, t4
    add     t3, t3, s3              // &heights[its cell]
    lh      t4, 0(t3)
    lh      t5, 2(t3)
    add     t6, t3, s5
    lh      t3, 0(t6)
    lh      t6, 2(t6)
    andi    t1, t1, 255
    andi    t2, t2, 255
    sub     t5, t5, t4
    mul     t5, t5, t1
    slli    t4, t4, 8
    add     t4, t4, t5
    sub     t6, t6, t3
    mul     t6, t6, t1
    slli    t3, t3, 8
    add     t3, t3, t6
    sub     t3, t3, t4
    mul     t3, t3, t2
    slli    t4, t4, 8
    add     t4, t4, t3
    fcvt.s.w ft1, t4
    fmul.s  ft1, ft1, fs7
23:
    lw      t1, 236(a0)             // in the ring as it was: eased
    blt     s11, t1, .Lm_before_project\@y
    lw      t1, 100(a0)
    bge     s11, t1, .Lm_before_project\@y
    flw     ft6, 104(a0)            // e2
    flw     ft7, 108(a0)            // eh2
    fadd.s  ft7, ft7, ft4
    fmadd.s ft6, ft4, ft7, ft6
    fsqrt.s ft6, ft6
    flw     ft7, 116(a0)
    fsub.s  ft6, ft6, ft7
    flw     ft7, 120(a0)
    fmul.s  ft6, ft6, ft7
    fmax.s  ft6, ft6, fs10
    fmin.s  ft6, ft6, fs5
    fmul.s  ft7, ft6, ft6
    fadd.s  ft6, ft6, ft6
    flw     ft4, 136(a0)            // 3
    fsub.s  ft6, ft4, ft6
    fmul.s  ft7, ft7, ft6
    fsub.s  ft1, ft1, fs9
    fmadd.s ft1, ft1, ft7, fs9
    j       .Lm_before_project\@y
.Lm_before_shown\@y:
    fmv.s   ft1, fs1                // its row
    fmv.s   ft6, fs2                // its w
.Lm_slope\@y:
    fsub.s  ft4, ft1, ft2           // its row less this one's
    flw     ft7, 124(a0)            // 1e-3
    flt.s   t4, ft7, ft4
    beqz    t4, .Lm_level\@y
    fsub.s  ft6, ft6, ft3
    fdiv.s  ft6, ft6, ft4           // w's change a row down
    j       .Lm_fill\@y
.Lm_level\@y:
    fmv.w.x ft6, zero
.Lm_fill\@y:  // rows [top, filled), from filled - 1 up
    fcvt.s.w ft4, t0
    fsub.s  ft4, ft4, ft2           // top - rho
    fmadd.s ft4, ft4, ft6, ft3      // w there, straight on from this sample...
    fmin.s  ft4, ft4, ft3           // ...no nearer than it
    sub     t1, s10, t0             // the rows
    addi    t2, t1, -1
    fcvt.s.w ft7, t2
    fmadd.s ft4, ft7, ft6, ft4      // w at row filled - 1
    lw      t2, 64(a0)              // bandRows - 1
    lw      t3, 68(a0)              // bandBack
    slli    t3, t3, 1
    lui     t6, 8                   // 0x8000: bfloat16, rounded
    andi    t5, t2, 1
    bnez    t5, 30f                 // bands an even number of rows: the even rows and the top
3:  // bands an odd number of rows: every row
    fmv.x.w t4, ft4
    add     t4, t4, t6
    srli    t4, t4, 16
    sh      t4, 0(a5)
    fsub.s  ft4, ft4, ft6           // the row above
    addi    t1, t1, -1
    beqz    a6, 4f
    addi    a6, a6, -1
    addi    a5, a5, -2
    bnez    t1, 3b
    j       5f
4:
    mv      a6, t2                  // the band before's last row
    add     a5, a5, t3
    bnez    t1, 3b
    j       5f
30: // the even rows - every band's first among them - and the top: what the drawing reads
    addi    t5, s10, -1             // the row, filled - 1
    andi    t4, t5, 1
    beqz    t4, 32f
    bne     t5, t0, 31f             // odd: only if it is the top
    FILL_STORE
31:
    fsub.s  ft4, ft4, ft6
    FILL_UP
    addi    t5, t5, -1
    blt     t5, t0, 5f              // past the top: done
32: // an even row
    FILL_STORE
    fsub.s  ft4, ft4, ft6           // the odd row above...
    beq     t5, t0, 34f             // ...past the top: done after a row up
    addi    t4, t5, -1
    beq     t4, t0, 35f             // ...the top: kept too
    fsub.s  ft4, ft4, ft6           // ...neither: two rows up at once
    beqz    a6, 33f
    addi    a6, a6, -2
    addi    a5, a5, -4
    addi    t5, t5, -2
    j       32b
33:
    addi    a6, t2, -1              // the band before's last but one
    add     a5, a5, t3
    addi    a5, a5, -2
    addi    t5, t5, -2
    j       32b
34:
    FILL_UP
    j       5f
35:
    FILL_UP
    FILL_STORE                      // the top, odd
    FILL_UP
5:
    mv      s10, t0                 // filled from top
    beqz    t0, .Lm_done\@y            // all of it: the column done
    // The line of sight through the new top row: base + s gk, its k and
    // gk - or every sample to the full test, where it isn't sloped.
    fcvt.s.w ft4, t0
    flw     ft7, 48(a0)             // cy + 0.5
    fsub.s  ft11, ft4, ft7          // k
    fmul.s  ft4, ft9, ft8           // f Dz
    fnmsub.s ft4, ft11, fa7, ft4    // den = f Dz - k Fz
    flw     ft7, 128(a0)            // -1e-4
    flt.s   t4, ft4, ft7
    sw      t4, 168(a0)             // sloped
    beqz    t4, 6f
    fmul.s  ft7, ft9, fa6           // f B
    fmsub.s ft7, ft11, fa5, ft7     // k A - f B
    fdiv.s  fs8, ft7, ft4           // gk
.if \round
    flw     ft7, 260(a0)
    fadd.s  fa4, fs8, ft7           // the test's slope: gk and the column's part of the fall
.else
    fmv.s   fa4, fs8
.endif
    j       .Lm_shown\@y
6:
    fmv.w.x fs8, zero
    flw     fa4, 132(a0)            // -inf
.Lm_shown\@y:
    mv      a2, s1                  // the last put on the screen
    fmv.s   fs1, ft2
    fmv.s   fs2, ft3
    j       .Lm_known\@y

.Lm_block\@y:  // into another block: near the eye with the line sloped, skipped whole if it can be
    mv      s8, t2                  // curBlk, tested or not
    lw      t4, 168(a0)
    beqz    t4, .Lm_cell\@y            // the line not sloped
    flt.s   t4, ft0, fs6
    beqz    t4, .Lm_cell\@y            // further than skipTo
    srai    t3, t0, 11              // the block's column...
    srai    t5, t1, 11              // ...and row
.if \round
    fadd.s  ft5, ft0, ft0
    fmadd.s ft5, ft5, fs3, fa1      // dX: XS + 2 s XS2
    fabs.s  ft6, ft5
    flw     ft7, 264(a0)            // 1e-9
    flt.s   t4, ft6, ft7
    fdiv.s  ft5, fs5, ft5           // 1 / dX...
    beqz    t4, 24f
    fmv.w.x ft5, zero               // ...or none
24:
.else
    flw     ft5, 204(a0)            // 1 / XS
.endif
    flt.s   t4, fs10, ft5
    beqz    t4, 11f
    addi    t6, t3, 1
    slli    t6, t6, 11
    fcvt.s.w ft6, t6
    fsub.s  ft6, ft6, ft2
    fmul.s  ft6, ft6, ft5           // how far on its east side is
    j       12f
11:
    flt.s   t4, ft5, fs10
    beqz    t4, 13f
    slli    t6, t3, 11
    fcvt.s.w ft6, t6
    fsub.s  ft6, ft6, ft2
    fmul.s  ft6, ft6, ft5           // its west side
    j       12f
13:
    flw     ft6, 244(a0)            // never, along x
12:
.if \round
    fadd.s  ft5, ft0, ft0
    fmadd.s ft5, ft5, fs11, fa3     // dY - ft6, the way out along x, kept
    fabs.s  ft7, ft5
    flw     ft1, 264(a0)
    flt.s   t4, ft7, ft1
    fdiv.s  ft5, fs5, ft5
    beqz    t4, 25f
    fmv.w.x ft5, zero
25:
.else
    flw     ft5, 208(a0)            // 1 / YS
.endif
    flt.s   t4, fs10, ft5
    beqz    t4, 14f
    addi    t6, t5, 1
    slli    t6, t6, 11
    fcvt.s.w ft7, t6
    fsub.s  ft7, ft7, ft3
    fmul.s  ft7, ft7, ft5
    j       15f
14:
    flt.s   t4, ft5, fs10
    beqz    t4, 16f
    slli    t6, t5, 11
    fcvt.s.w ft7, t6
    fsub.s  ft7, ft7, ft3
    fmul.s  ft7, ft7, ft5
    j       15f
16:
    flw     ft7, 244(a0)
15:
    fmin.s  ft6, ft6, ft7
    fmax.s  ft6, ft6, fs10
    fadd.s  ft6, ft0, ft6           // sOut: where the ray leaves it
    flw     ft7, 216(a0)            // fall
    flw     ft2, 212(a0)            // groundUp + ez
    fmul.s  ft5, ft6, ft6
    fmadd.s ft5, ft5, ft7, ft2      // the base there
    fmadd.s ft5, ft6, fa4, ft5      // the line of sight there
    fmin.s  ft5, ft4, ft5           // the lower of it and here's
    fneg.s  ft3, fa4
    flw     ft1, 220(a0)            // 2 fall
    fdiv.s  ft3, ft3, ft1           // where the line, bent by the fall, is lowest
    flt.s   t4, ft0, ft3
    beqz    t4, 17f
    flt.s   t4, ft3, ft6
    beqz    t4, 17f
    fmul.s  ft1, ft3, ft3
    fmadd.s ft1, ft1, ft7, ft2
    fmadd.s ft1, ft3, fa4, ft1
    fmin.s  ft5, ft5, ft1           // lower still, between
17:
    slli    t4, t2, 1
    lw      t6, 232(a0)             // blockMax
    add     t4, t4, t6
    lh      t4, 0(t4)
    fcvt.s.w ft1, t4                // its highest
    flw     ft2, 224(a0)            // the ring's: or the airfield's ground
    fle.s   t4, ft2, ft6
    beqz    t4, 18f
    flw     ft2, 228(a0)
    flt.s   t4, ft0, ft2
    beqz    t4, 18f
    fmax.s  ft1, ft1, fs9
18:
    flt.s   t4, ft1, ft5
    beqz    t4, 19f                 // it may reach the line
    mv      t3, s1                  // its last step: j, eight at a time, then one
    mv      t5, s0
20:
    addi    t4, t3, 8
    bge     t4, s2, 21f
    flw     ft2, 32(t5)
    flt.s   t4, ft2, ft6
    beqz    t4, 21f
    addi    t3, t3, 8
    addi    t5, t5, 32
    j       20b
21:
    addi    t4, t3, 1
    bge     t4, s2, 22f
    flw     ft2, 4(t5)
    flt.s   t4, ft2, ft6
    beqz    t4, 22f
    addi    t3, t3, 1
    addi    t5, t5, 4
    j       21b
22:
    addi    t4, s1, 1
    ble     t3, t4, 19f             // a step or none skipped: not worth it
    lw      t4, 240(a0)
    addi    t4, t4, 1
    sw      t4, 240(a0)             // skips
    addi    a4, a4, 1               // this sample, counted
    mv      s1, t3                  // on from the block's last step
    mv      s0, t5
    j       .Lm_step\@y
19:
    flw     ft1, 1600(s0)           // its base, again
    j       .Lm_cell\@y

.Lm_ring\@y:  // the airfield's ring, from iEaseLo to iEaseHi
    lw      t4, 100(a0)
    blt     s1, t4, 8f
    li      a3, 0x7fffffff          // past it: never again
    j       .Lm_most\@y
8:
    fmax.s  ft5, ft5, fs9           // its highest, or the airfield's ground
    flt.s   t4, ft5, ft4
    beqz    t4, 9f
    addi    a4, a4, 1               // hidden
    mv      s11, s1
    j       .Lm_next\@y
9:
    lh      t4, 0(t3)               // its height between its corners, as above
    lh      t5, 2(t3)
    add     t6, t3, s5
    lh      t2, 0(t6)
    lh      t6, 2(t6)
    andi    t0, t0, 255
    andi    t1, t1, 255
    sub     t5, t5, t4
    mul     t5, t5, t0
    slli    t4, t4, 8
    add     t4, t4, t5
    sub     t6, t6, t2
    mul     t6, t6, t0
    slli    t2, t2, 8
    add     t2, t2, t6
    sub     t2, t2, t4
    mul     t2, t2, t1
    slli    t4, t4, 8
    add     t4, t4, t2
    fcvt.s.w ft5, t4
    fmul.s  ft5, ft5, fs7
.Lm_ease\@y:  // eased into the airfield's ground: t = (|eye + s h| - 2500) / 2500
    flw     ft6, 104(a0)            // e2
    flw     ft7, 108(a0)            // eh2
    fadd.s  ft7, ft7, ft0           // eh2 + s
    fmadd.s ft6, ft0, ft7, ft6      // e2 + s (eh2 + s)
    fsqrt.s ft6, ft6
    flw     ft7, 116(a0)            // 2500
    fsub.s  ft6, ft6, ft7
    flw     ft7, 120(a0)            // 1 / 2500
    fmul.s  ft6, ft6, ft7
    fmax.s  ft6, ft6, fs10          // 0 to 1
    fmin.s  ft6, ft6, fs5
    fmul.s  ft7, ft6, ft6           // t t
    fadd.s  ft6, ft6, ft6           // 2 t
    flw     ft3, 136(a0)            // 3
    fsub.s  ft6, ft3, ft6
    fmul.s  ft7, ft7, ft6           // t t (3 - 2 t)
    fsub.s  ft5, ft5, fs9
    fmadd.s ft5, ft5, ft7, fs9      // groundUp + (h - groundUp) that
    j       .Lm_compare\@y

.Lm_outside\@y:  // off the grid: the sea's height, eased in the ring
    fmv.w.x ft5, zero
    blt     s1, a3, .Lm_compare\@y
    lw      t4, 100(a0)
    blt     s1, t4, .Lm_ease\@y
    li      a3, 0x7fffffff
    j       .Lm_compare\@y

.Lm_giveup\@y:  // the highest land along the column, this far off or further: under the rows filled?
    lw      t4, 248(a0)             // zHighAt
    srai    t5, s1, 4
    slli    t5, t5, 2
    add     t4, t4, t5
    flw     ft5, 0(t4)              // from this sixteenth on
    flt.s   t4, fs10, ft5
    beqz    t4, 10f
    fmul.s  ft6, ft5, fa7
    fmadd.s ft6, ft0, fa5, ft6      // zcHigh = s A + zHigh Fz
    flt.s   t4, fs4, ft6
    beqz    t4, .Lm_on\@y
    fmul.s  ft7, ft5, ft8
    fmadd.s ft7, ft0, fa6, ft7      // ycHigh
    fmul.s  ft7, ft9, ft7           // f ycHigh
    fmul.s  ft6, ft11, ft6          // k zcHigh
    flt.s   t4, ft6, ft7
    bnez    t4, .Lm_done\@y
    j       .Lm_on\@y
10:
    flt.s   t4, fs10, fa5           // A > 0
    beqz    t4, .Lm_on\@y
    fmul.s  ft6, ft9, fa6           // f B
    fmul.s  ft7, ft11, fa5          // k A
    flt.s   t4, ft7, ft6
    bnez    t4, .Lm_done\@y
    j       .Lm_on\@y

.Lm_hand_counted\@y:  // for C, which counts the sample itself
    addi    a4, a4, -1
.Lm_hand\@y:
    li      t6, 1
.Lm_save\@y:
    sw      s1, 140(a0)             // i
    sw      s10, 144(a0)            // filled
    sub     t0, a5, a7
    srai    t0, t0, 1
    sw      t0, 148(a0)             // at
    sw      a6, 152(a0)             // off
    fsw     ft11, 156(a0)           // k
    fsw     fs8, 160(a0)            // gk
.if \round
    lw      t0, 168(a0)             // gkLine: gk, sloped, or every sample to the full test
    fmv.s   ft0, fs8
    bnez    t0, 26f
    flw     ft0, 132(a0)
26:
    fsw     ft0, 164(a0)
.else
    fsw     fa4, 164(a0)            // gkLine
.endif
    sw      s11, 172(a0)            // prev
    sw      a1, 176(a0)             // known
    sw      a2, 180(a0)             // shown
    fsw     fs0, 184(a0)            // knownH
    fsw     fs1, 188(a0)            // shownRho
    fsw     fs2, 192(a0)            // shownW
    sw      s8, 196(a0)             // curBlk
    sw      a4, 200(a0)             // samples
    sw      a3, 96(a0)              // iEaseLo
    mv      a0, t6
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
    addi    sp, sp, 96
    ret
    .size   \name, .-\name
.endm

    MARCH_FN scene3d_march_flat, 0
    MARCH_FN scene3d_march_round, 1

// void scene3d_latencies(uint32_t out[12]);
//
// The bench's measure of the P4's latencies and throughputs the terrain's
// loops wait on, each the best of four runs of 32 turns of eight - the
// loop's own two instructions a turn spread over the eight - with the
// core's interrupts off: cycles for all 256 (fcvt and fmv pairs: 128 of
// each). In order: fadd.s, fmul.s, fmadd.s chains; fmadd.s two chains
// interleaved and four; fmadd.s independent; fcvt.w.s then fcvt.s.w, a
// chain; fcvt.w.s independent; fdiv.s chain; fdiv.s independent; lw, a
// pointer chased; fmv.x.w then fmv.w.x, a chain.

.macro LAT_TEST slot, body:vararg
    li      t5, 4                   // runs
    li      t6, -1                  // the best
91:
    csrr    t2, mcycle
    li      t1, 32
    .balign 16
92:
    .rept 8
    \body
    .endr
    addi    t1, t1, -1
    bnez    t1, 92b
    csrr    t3, mcycle
    sub     t3, t3, t2
    bgeu    t3, t6, 93f
    mv      t6, t3
93:
    addi    t5, t5, -1
    bnez    t5, 91b
    sw      t6, \slot(a0)
.endm

    .align  2
    .global scene3d_latencies
    .type   scene3d_latencies, @function
scene3d_latencies:
    csrrci  a7, mstatus, 8          // interrupts off; their state kept in a7
    addi    sp, sp, -16
    sw      sp, 0(sp)               // a word that points to itself, to chase
    li      t0, 0x3f800000
    fmv.w.x ft1, t0                 // 1
    fmv.w.x ft2, zero               // 0
    fmv.w.x ft0, t0
    fmv.w.x ft3, t0
    fmv.w.x ft4, t0
    fmv.w.x ft5, t0
    LAT_TEST 0, fadd.s ft0, ft0, ft2
    LAT_TEST 4, fmul.s ft0, ft0, ft1
    LAT_TEST 8, fmadd.s ft0, ft0, ft1, ft2
    LAT_TEST 12, fmadd.s ft0, ft0, ft1, ft2; fmadd.s ft3, ft3, ft1, ft2
    LAT_TEST 16, fmadd.s ft0, ft0, ft1, ft2; fmadd.s ft3, ft3, ft1, ft2; fmadd.s ft4, ft4, ft1, ft2; fmadd.s ft5, ft5, ft1, ft2
    LAT_TEST 20, fmadd.s ft6, ft1, ft1, ft2
    LAT_TEST 24, fcvt.w.s t4, ft0, rtz; fcvt.s.w ft0, t4
    LAT_TEST 28, fcvt.w.s t4, ft1, rtz
    LAT_TEST 32, fdiv.s ft0, ft1, ft0
    LAT_TEST 36, fdiv.s ft6, ft1, ft1
    mv      t4, sp
    LAT_TEST 40, lw t4, 0(t4)
    LAT_TEST 44, fmv.x.w t4, ft0; fmv.w.x ft0, t4
    addi    sp, sp, 16
    csrw    mstatus, a7             // interrupts as they were
    ret
    .size   scene3d_latencies, .-scene3d_latencies
