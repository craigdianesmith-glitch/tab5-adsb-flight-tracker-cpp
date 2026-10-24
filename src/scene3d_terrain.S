// The land at its height, in assembly: the colours of the 3D view's
// terrain, a band at a time (see scene3d.cpp's terrainBand(); the march
// that works out its depths is scene3d_march.S's), written for the
// ESP32-P4's RV32IMAFC core by hand - every value the loop needs kept in a
// register, the floating-point unit's latencies covered by work that
// doesn't wait on them, and nothing but the common case in the loop: what
// is rare is handed back to C, which does it and comes back. Four ways in,
// one macro's:
//
//   scene3d_terrain_band        the flat cover, a sample every two rows
//   scene3d_terrain_band_round  the round earth's cover, likewise
//   scene3d_terrain_quad        the flat cover, a sample every four rows -
//                               the governor skimping, the cells a level
//                               coarser, so that one is still four rows
//                               tall where it is four pixels wide
//   scene3d_terrain_quad_round  the round earth's, likewise
//
// A unit is the rows a sample covers - two, or four - from a multiple of
// it: a band's first rows, to the next multiple, are a unit too.

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
    .equ    MP_COUNT, 4
    .equ    MP_MAP, 16
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
    lw      t1, MP_COUNT(t0)
    addi    t2, t0, MP_MAP          // the first
    slli    t3, t1, 6               // count M_SIZE, 96 = 64 + 32
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
