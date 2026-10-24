// The land at its height, in assembly: the innermost loops of the 3D view's
// terrain (see scene3d.cpp's terrainMarch() and terrainBand()), written for
// the ESP32-P4's RV32IMAFC core by hand - every value the loop needs kept in
// a register, the floating-point unit's latencies covered by work that
// doesn't wait on them, and nothing but the common case in the loop: what
// is rare is handed back to C, which does it and comes back.

// int scene3d_terrain_band(TerrainCol *t);
//
// A band's land at its height for the flat cover - scene3d.cpp's
// terrainSampleC() exactly, for as long as it goes the common way - a pair
// of rows at a time across every column, so that each sample's neighbours
// are its column's on either side: much the same depth, the same part of
// the cover, often the same cell, its pair of pixels the last one's. For
// each column with land in the pair: a sample on the even row - or, where
// the column's land starts on the odd row, on that - its 1 / depth,
// bfloat16, and the depth from it, or the airfield's flat ground's, the
// row's own, from the column's flat top down; where on the cover's grid
// that is, u = ue + z (rowU[y] + cu) and v likewise; the cell there, from
// the map of the part of the cover being read - two multiply-adds, two
// casts and two compares, where the depth is the map's level's and the
// place inside its part; the haze for the distance, by its float bits' top
// eleven; and each row's pair of pixels for the cell and haze, from its
// parity's last, or its parity's cache. Two stores a row. A band beginning
// on an odd row has that row on its own first, each column's sample there
// its own.
//
// A pair of pixels not in its parity's cache it works out itself, as
// pairInt() does, and keeps there.
//
// It returns 0 once the band is done, and 1 at a sample it can't do - the
// depth another level's, the place off the map's part - with t->y, t->px,
// t->zc, the column's flat top, cu and cv,
// the parities' last keys and pairs, and the count of samples as C needs
// them to do it the long way, and t->rowAt and t->c where to go on from.
//
// TerrainCol's fields, by offset (static_assert'ed in scene3d.cpp):
//   0 px, 4 zc (by row), 8 y, 12 yb, 16 flatTop, 20 stride (bytes), 24 cu,
//   28 cv, 32 ue, 36 ve, 40 rowU, 44 rowV, 48 rowFar, 52 flatZ, 56 haze,
//   60 zLo, 64 zHi, 68 au, 72 bu, 76 av, 80 bv, 84 cells, 88 cols, 92 rows,
//   96 mapStride, 100 keyE, 104 pairE, 108 keyO, 112 pairO, 116 samples,
//   120 pairs[0], 124 pairs[1], 128 c, 132 the band's columns, 136 y0,
//   140 topFlat, 144 colUV, 148 rowAt, 152 zs, 156 a column's bytes of
//   them, 160 color, 164 coverRgb, 168 the haze's colour, 172 swap, 176
//   pairs worked out, 180 the round cover's U0-U5 and 204 V0-V5.
// TerrainPairs: 0 serial, 4 key[128] (16 bits), 260 pair[128].
//
// Registers: s0 px (the column's, on the row), s1 its 1 / depths there, s2
// the column, s3 the band's columns, s4 &topFlat[c], s5 stride, s6 a
// column's bytes of 1 / depths, s7 &colUV[c], s8 yb, s9 the haze table,
// s10 the map's cells, s11 its columns, a1 its rows, a2 its stride, a3-a6
// the even rows' last key and pair and the odd's, a7 the samples, t6 the
// row, t5 a link for the cache's lookups; fa2, fa3 ue, ve, fa4-fa7 zLo,
// zHi, au, bu, ft8-ft10 av, bv, 1; fs0-fs3 the row's rowU, rowV, rowFar
// and flatZ, fs4-fs7 the row under's. It calls nothing: s0-s11 and fs0-fs7
// are saved, and s9-s11 and a1 again while a pair is worked out.

// The sample at row `row` of column s2 - t0 its top | flat top << 16, as
// the caller has it - its 1 / depth `zoff` bytes on from s1 and its row's
// values in ru, rv, rf, fz: its cell | haze << 8 in t1, or on to `none`
// where there is nothing; to `hand` where it can't go on. The division
// first: what follows, to the u and v, goes on under it.
.macro TERRAIN_SAMPLE ru, rv, rf, fz, zoff, row, none, hand, round=0, zreg=s1
    srai    t2, t0, 16              // the column's flat top
    blt     \row, t2, 91f
    fmv.s   ft1, \fz                // the airfield's flat ground: the row's own depth
    j       92f
91:
    lhu     t0, \zoff(\zreg)        // 1 / depth, bfloat16
    slli    t0, t0, 16
    fmv.w.x ft0, t0
    fdiv.s  ft1, ft10, ft0          // the depth
92:
    flw     ft2, 0(s7)              // cu - round, cx
    flw     ft3, 4(s7)              // cv - round, cy
    fadd.s  ft4, \ru, ft2           // rowU + cu
    fadd.s  ft5, \rv, ft3           // rowV + cv
.if \round
    // Round: dx and dy east and north of the eye, z (rowX + cx) and z
    // (rowY + cy); u = U0 + dx (U1 + U3 dx + U4 dy) + dy (U2 + U5 dy), and
    // v likewise - U0 to U5 in fa2, fa0, fa1, ft11, fs8, fs9; V0 to V2 in
    // fa3, fs10, fs11, V3 to V5 read from t at 216.
    fmul.s  ft4, ft4, ft1           // dx
    fmul.s  ft5, ft5, ft1           // dy
    fmadd.s ft2, ft11, ft4, fa0     // U1 + U3 dx
    fmadd.s ft2, fs8, ft5, ft2      // + U4 dy
    fmadd.s ft3, fs9, ft5, fa1      // U2 + U5 dy
    fmadd.s ft2, ft4, ft2, fa2      // U0 + dx (...)
    fmadd.s ft2, ft5, ft3, ft2      // + dy (...): u
    flw     ft3, 216(a0)            // V3
    flw     ft0, 220(a0)            // V4
    fmadd.s ft3, ft3, ft4, fs10     // V1 + V3 dx
    fmadd.s ft3, ft0, ft5, ft3      // + V4 dy
    flw     ft0, 224(a0)            // V5
    fmadd.s ft0, ft0, ft5, fs11     // V2 + V5 dy
    fmadd.s ft3, ft4, ft3, fa3      // V0 + dx (...)
    fmadd.s ft5, ft5, ft0, ft3      // + dy (...): v
    fmv.s   ft4, ft2                // u
.else
    fmadd.s ft4, ft4, ft1, fa2      // u
    fmadd.s ft5, ft5, ft1, fa3      // v
.endif
    fmul.s  ft6, ft1, \rf           // the distance
    fmadd.s ft4, ft4, fa6, fa7      // the map's column, and 1024
    fmadd.s ft5, ft5, ft8, ft9      // its row, and 1024
    flt.s   t1, fa4, ft1            // the depth over zLo...
    fle.s   t2, ft1, fa5            // ...and to zHi: the map's level
    fcvt.w.s t3, ft4, rtz
    fcvt.w.s t4, ft5, rtz
    and     t1, t1, t2
    addi    t3, t3, -1024           // the 1024 off: a floor, not a cast's truncation
    addi    t4, t4, -1024
    beqz    t1, \hand               // another level's: the long way
    bgeu    t3, s11, \hand          // off the map's part
    bgeu    t4, a1, \hand
    mul     t4, t4, a2
    add     t4, t4, t3
    add     t4, t4, s10
    lbu     t1, 0(t4)               // the cell
    fmv.x.w t2, ft6
    srli    t2, t2, 21
    add     t2, t2, s9
    lbu     t2, 0(t2)               // the haze for the distance
    addi    a7, a7, 1               // a sample
    sltiu   t3, t1, 16
    bnez    t3, \none               // nothing there: the rows' own colours
    slli    t2, t2, 8
    or      t1, t1, t2              // cell | haze << 8
.endm

// The pair of pixels for key t1 - cell | haze << 8 - on a row of the parity
// whose dithers are dLo and dHi, worked out as scene3d.cpp's pairInt() does
// and kept in the cache at t3, its slot, in t0, its parity's: the cell's lit
// colour hazed `haze` << 3 of 256 towards the haze's, each channel; packed
// to 5:6:5 twice, dithered dLo for the pair's first pixel and dHi for its
// second - x = c top + dither, (x + 1 + (x >> 8)) >> 8, no more than top -
// bytes swapped for the panel where it wants them. The pair in `out`. s9,
// s10, s11 and a1 kept on the stack while it works.
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
.macro PACK565 dither, out
    LEVEL   t0, 31, \dither, a1, s9
    slli    \out, a1, 11
    LEVEL   t2, 63, \dither, a1, s9
    slli    a1, a1, 5
    or      \out, \out, a1
    LEVEL   t4, 31, \dither, a1, s9
    or      \out, \out, a1
    lw      a1, 172(a0)             // swapped for the panel?
    beqz    a1, 96f
    srli    a1, \out, 8
    slli    \out, \out, 24
    srli    \out, \out, 16
    or      \out, \out, a1
96:
.endm
.macro MAKE_PAIR dLo, dHi, out
    sw      s9, 80(sp)
    sw      s10, 84(sp)
    sw      s11, 88(sp)
    sw      a1, 92(sp)
    slli    t4, t3, 1
    add     t4, t4, t0
    sh      t1, 4(t4)               // the key, kept
    slli    t3, t3, 2
    add     t3, t3, t0              // &pair[slot] - 260
    andi    t4, t1, 255             // the cell
    srli    s9, t1, 8
    slli    s9, s9, 3               // the haze, of 256
    lw      s10, 164(a0)            // coverRgb
    slli    t2, t4, 1
    add     t2, t2, t4
    add     s10, s10, t2            // the cell's lit colour
    lw      s11, 168(a0)            // the haze's colour
    lbu     t0, 0(s10)
    lbu     t2, 1(s10)
    lbu     t4, 2(s10)
    lw      a1, 0(s11)
    sub     a1, a1, t0
    mul     a1, a1, s9
    srai    a1, a1, 8
    add     t0, t0, a1              // red, hazed
    lw      a1, 4(s11)
    sub     a1, a1, t2
    mul     a1, a1, s9
    srai    a1, a1, 8
    add     t2, t2, a1              // green
    lw      a1, 8(s11)
    sub     a1, a1, t4
    mul     a1, a1, s9
    srai    a1, a1, 8
    add     t4, t4, a1              // blue
    PACK565 \dLo, s10
    PACK565 \dHi, s11
    slli    s11, s11, 16
    or      \out, s10, s11
    sw      \out, 260(t3)           // the pair, kept
    lw      a1, 176(a0)
    addi    a1, a1, 1
    sw      a1, 176(a0)             // pairs worked out
    lw      s9, 80(sp)
    lw      s10, 84(sp)
    lw      s11, 88(sp)
    lw      a1, 92(sp)
.endm

.macro TERRAIN_BAND_FN name, round
    .text
    .align  2
    .global \name
    .type   \name, @function
\name:
    addi    sp, sp, -112
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
.if \round
    fsw     fs8, 96(sp)
    fsw     fs9, 100(sp)
    fsw     fs10, 104(sp)
    fsw     fs11, 108(sp)
.endif
    lw      s8, 12(a0)              // yb
    lw      s5, 20(a0)              // stride
.if \round
    flw     fa2, 180(a0)            // U0
    flw     fa0, 184(a0)            // U1
    flw     fa1, 188(a0)            // U2
    flw     ft11, 192(a0)           // U3
    flw     fs8, 196(a0)            // U4
    flw     fs9, 200(a0)            // U5
    flw     fa3, 204(a0)            // V0
    flw     fs10, 208(a0)           // V1
    flw     fs11, 212(a0)           // V2
.else
    flw     fa2, 32(a0)             // ue
    flw     fa3, 36(a0)             // ve
.endif
    lw      s9, 56(a0)              // the haze table
    flw     fa4, 60(a0)             // zLo
    flw     fa5, 64(a0)             // zHi
    flw     fa6, 68(a0)             // au
    flw     fa7, 72(a0)             // bu
    flw     ft8, 76(a0)             // av
    flw     ft9, 80(a0)             // bv
    lw      s10, 84(a0)             // the map's cells
    lw      s11, 88(a0)             // its columns
    lw      a1, 92(a0)              // its rows
    lw      a2, 96(a0)              // its stride
    lw      a3, 100(a0)             // the parities' last keys and pairs
    lw      a4, 104(a0)
    lw      a5, 108(a0)
    lw      a6, 112(a0)
    lw      a7, 116(a0)             // samples
    lw      s3, 132(a0)             // the band's columns
    lw      s6, 156(a0)             // a column's bytes of 1 / depths
    li      t0, 0x3f800000
    fmv.w.x ft10, t0                // 1
    lw      t6, 148(a0)             // the row
    lw      s2, 128(a0)             // the column
.Lb_row\@x:  // row t6 - a pair's even, or an odd row alone - from column s2
    bge     t6, s8, .Lb_all\@x
    slli    t1, t6, 2
    lw      t0, 40(a0)
    add     t0, t0, t1
    flw     fs0, 0(t0)              // rowU[y]
    flw     fs4, 4(t0)              // and the row under's
    lw      t0, 44(a0)
    add     t0, t0, t1
    flw     fs1, 0(t0)              // rowV
    flw     fs5, 4(t0)
    lw      t0, 48(a0)
    add     t0, t0, t1
    flw     fs2, 0(t0)              // rowFar
    flw     fs6, 4(t0)
    lw      t0, 52(a0)
    add     t0, t0, t1
    flw     fs3, 0(t0)              // flatZ
    flw     fs7, 4(t0)
    lw      t1, 136(a0)             // y0
    sub     t1, t6, t1
    lw      s0, 160(a0)             // color
    mul     t2, t1, s5
    add     s0, s0, t2
    slli    t2, s2, 3
    add     s0, s0, t2              // px: the column's four pixels on the row
    lw      s1, 152(a0)             // zs
    mul     t2, s2, s6
    add     s1, s1, t2
    slli    t1, t1, 1
    add     s1, s1, t1              // its 1 / depth there
    lw      t0, 140(a0)             // topFlat
    slli    t1, s2, 2
    add     s4, t0, t1
    lw      t0, 144(a0)             // colUV
    slli    t1, s2, 3
    add     s7, t0, t1
    andi    t0, t6, 1
    bnez    t0, .Lb_single\@x          // an odd row: on its own
.Lb_col\@x:  // column s2 on the pair t6, t6 + 1
    bge     s2, s3, .Lb_pair_done\@x
    lw      t0, 0(s4)
    slli    t1, t0, 16
    srai    t1, t1, 16              // the column's highest row of land
    addi    t3, t6, 1
    blt     t3, t1, .Lb_next\@x        // below the pair
    bne     t3, t1, 6f
    blt     t3, s8, .Lb_odd_first\@x   // from the odd row...
    j       .Lb_next\@x                // ...which is past the band's last
6:
    TERRAIN_SAMPLE fs0, fs1, fs2, fs3, 0, t6, .Lb_next\@x, .Lb_hand_even\@x, \round
    addi    t3, t6, 1
    bge     t3, s8, .Lb_even_last\@x   // the band's last row: the even alone
    beq     t1, a3, 1f
    jal     t5, .Lb_lookE\@x           // not the even rows' last
1:
    beq     t1, a5, 2f
    jal     t5, .Lb_lookO\@x           // nor the odd's
2:
    sw      a4, 0(s0)               // the even row's four pixels
    sw      a4, 4(s0)
    add     t0, s0, s5
    sw      a6, 0(t0)               // and the odd's
    sw      a6, 4(t0)
.Lb_next\@x:  // the next column
    addi    s0, s0, 8
    add     s1, s1, s6
    addi    s4, s4, 4
    addi    s7, s7, 8
    addi    s2, s2, 1
    j       .Lb_col\@x
.Lb_even_last\@x:
    beq     t1, a3, 3f
    jal     t5, .Lb_lookE\@x
3:
    sw      a4, 0(s0)
    sw      a4, 4(s0)
    j       .Lb_next\@x
.Lb_odd_first\@x:  // the column's land from the odd row: its own sample there
    addi    t3, t6, 1
    TERRAIN_SAMPLE fs4, fs5, fs6, fs7, 2, t3, .Lb_next\@x, .Lb_hand_odd\@x, \round
    beq     t1, a5, 4f
    jal     t5, .Lb_lookO\@x
4:
    add     t0, s0, s5
    sw      a6, 0(t0)
    sw      a6, 4(t0)
    j       .Lb_next\@x
.Lb_pair_done\@x:
    addi    t6, t6, 2
    li      s2, 0
    j       .Lb_row\@x
.Lb_single\@x:  // an odd row alone, the band's first: every column with land on it
    bge     s2, s3, .Lb_single_done\@x
    lw      t0, 0(s4)
    slli    t1, t0, 16
    srai    t1, t1, 16
    blt     t6, t1, .Lb_single_next\@x
    TERRAIN_SAMPLE fs0, fs1, fs2, fs3, 0, t6, .Lb_single_next\@x, .Lb_hand_single\@x, \round
    beq     t1, a5, 5f
    jal     t5, .Lb_lookO\@x
5:
    sw      a6, 0(s0)
    sw      a6, 4(s0)
.Lb_single_next\@x:
    addi    s0, s0, 8
    add     s1, s1, s6
    addi    s4, s4, 4
    addi    s7, s7, 8
    addi    s2, s2, 1
    j       .Lb_single\@x
.Lb_single_done\@x:
    addi    t6, t6, 1
    li      s2, 0
    j       .Lb_row\@x

.Lb_lookE\@x:  // the even rows' pair for key t1, from their cache: slot (cell ^ haze 37) & 127
    andi    t3, t1, 255
    srli    t4, t1, 8
    slli    t2, t4, 5
    add     t2, t2, t4
    slli    t4, t4, 2
    add     t4, t4, t2              // haze 37
    xor     t3, t3, t4
    andi    t3, t3, 127
    lw      t0, 120(a0)             // pairs[0]
    slli    t4, t3, 1
    add     t4, t4, t0
    lhu     t4, 4(t4)               // its key
    bne     t4, t1, 1f              // not in it: worked out
    slli    t4, t3, 2
    add     t4, t4, t0
    lw      a4, 260(t4)             // its pair
    mv      a3, t1
    jr      t5
1:
    MAKE_PAIR 0, 127, a4            // the even rows' dithers
    mv      a3, t1
    jr      t5
.Lb_lookO\@x:  // the odd rows'
    andi    t3, t1, 255
    srli    t4, t1, 8
    slli    t2, t4, 5
    add     t2, t2, t4
    slli    t4, t4, 2
    add     t4, t4, t2
    xor     t3, t3, t4
    andi    t3, t3, 127
    lw      t0, 124(a0)             // pairs[1]
    slli    t4, t3, 1
    add     t4, t4, t0
    lhu     t4, 4(t4)
    bne     t4, t1, 1f
    slli    t4, t3, 2
    add     t4, t4, t0
    lw      a6, 260(t4)
    mv      a5, t1
    jr      t5
1:
    MAKE_PAIR 191, 63, a6           // the odd rows'
    mv      a5, t1
    jr      t5
.Lb_all\@x:  // the band done
    li      t5, 0
    sw      a7, 116(a0)
    j       .Lb_out\@x
.Lb_hand_pair\@x:  // a pair of pixels not in the cache: the sample, counted, to C - at the row t3 its link left
    addi    a7, a7, -1
    lw      t0, 0(s4)               // which row: the pair's even, unless the column's land starts on the odd
    slli    t1, t0, 16
    srai    t1, t1, 16
    andi    t2, t6, 1
    bnez    t2, .Lb_hand_single\@x     // an odd row alone
    addi    t3, t6, 1
    beq     t3, t1, .Lb_hand_odd\@x
.Lb_hand_even\@x:  // the long way at the pair's even row
    mv      t3, t6
    mv      t4, s0
    j       .Lb_hand\@x
.Lb_hand_odd\@x:  // at the odd row, the column's first
    addi    t3, t6, 1
    add     t4, s0, s5
    j       .Lb_hand\@x
.Lb_hand_single\@x:  // at the odd row alone
    mv      t3, t6
    mv      t4, s0
.Lb_hand\@x:  // back to C at row t3, its pixels t4, with what it needs; on from the next column
    sw      t4, 0(a0)               // px
    slli    t0, t6, 1
    sub     t0, s1, t0
    sw      t0, 4(a0)               // zc, by row
    sw      t3, 8(a0)               // y
    lw      t0, 0(s4)
    srai    t0, t0, 16
    sw      t0, 16(a0)              // the flat top
    flw     ft2, 0(s7)
    flw     ft3, 4(s7)
    fsw     ft2, 24(a0)             // cu
    fsw     ft3, 28(a0)             // cv
    sw      a3, 100(a0)
    sw      a4, 104(a0)
    sw      a5, 108(a0)
    sw      a6, 112(a0)
    sw      a7, 116(a0)
    addi    t0, s2, 1
    sw      t0, 128(a0)             // on from the next column
    sw      t6, 148(a0)             // on this row
    li      t5, 1
.Lb_out\@x:
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
    flw     fs0, 48(sp)
    flw     fs1, 52(sp)
    flw     fs2, 56(sp)
    flw     fs3, 60(sp)
    flw     fs4, 64(sp)
    flw     fs5, 68(sp)
    flw     fs6, 72(sp)
    flw     fs7, 76(sp)
.if \round
    flw     fs8, 96(sp)
    flw     fs9, 100(sp)
    flw     fs10, 104(sp)
    flw     fs11, 108(sp)
.endif
    addi    sp, sp, 112
    ret
    .size   \name, .-\name
.endm

    TERRAIN_BAND_FN scene3d_terrain_band, 0
    TERRAIN_BAND_FN scene3d_terrain_band_round, 1

// int scene3d_terrain_quad(TerrainCol *t), scene3d_terrain_quad_round:
//
// scene3d_terrain_band's work four rows a sample, where the governor has
// the land skimping - and the cells a level coarser then, as scene3d.cpp's
// levelFor() reads them, so that one is four rows tall where it is four
// pixels wide. Units of four rows from a multiple of four, across every
// column: a sample on the unit's first row the band has, the even and odd
// rows from it with their parities' pairs; a column whose land starts in
// the unit sampled on its first row, the unit's rows from there. Four rows,
// eight stores; fewer, at a band's edge, a row at a time. What it can't do
// it leaves for C as the pairs' routine does, with t->rowAt the unit's
// first row in the band.

// Rows t2 to t3 of the column from its pixels at t0, each its parity's pair.
.macro QUAD_ROWS
97:
    andi    t4, t2, 1
    bnez    t4, 98f
    sw      a4, 0(t0)
    sw      a4, 4(t0)
    j       99f
98:
    sw      a6, 0(t0)
    sw      a6, 4(t0)
99:
    add     t0, t0, s5
    addi    t2, t2, 1
    blt     t2, t3, 97b
.endm

// The unit's end in the band, min((t6 & -4) + 4, yb), in \out.
.macro QUAD_END out
    andi    \out, t6, -4
    addi    \out, \out, 4
    blt     \out, s8, 96f
    mv      \out, s8
96:
.endm

.macro TERRAIN_QUAD_FN name, round
    .text
    .align  2
    .global \name
    .type   \name, @function
\name:
    addi    sp, sp, -112
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
.if \round
    fsw     fs8, 96(sp)
    fsw     fs9, 100(sp)
    fsw     fs10, 104(sp)
    fsw     fs11, 108(sp)
.endif
    lw      s8, 12(a0)              // yb
    lw      s5, 20(a0)              // stride
.if \round
    flw     fa2, 180(a0)            // U0
    flw     fa0, 184(a0)            // U1
    flw     fa1, 188(a0)            // U2
    flw     ft11, 192(a0)           // U3
    flw     fs8, 196(a0)            // U4
    flw     fs9, 200(a0)            // U5
    flw     fa3, 204(a0)            // V0
    flw     fs10, 208(a0)           // V1
    flw     fs11, 212(a0)           // V2
.else
    flw     fa2, 32(a0)             // ue
    flw     fa3, 36(a0)             // ve
.endif
    lw      s9, 56(a0)              // the haze table
    flw     fa4, 60(a0)             // zLo
    flw     fa5, 64(a0)             // zHi
    flw     fa6, 68(a0)             // au
    flw     fa7, 72(a0)             // bu
    flw     ft8, 76(a0)             // av
    flw     ft9, 80(a0)             // bv
    lw      s10, 84(a0)             // the map's cells
    lw      s11, 88(a0)             // its columns
    lw      a1, 92(a0)              // its rows
    lw      a2, 96(a0)              // its stride
    lw      a3, 100(a0)             // the parities' last keys and pairs
    lw      a4, 104(a0)
    lw      a5, 108(a0)
    lw      a6, 112(a0)
    lw      a7, 116(a0)             // samples
    lw      s3, 132(a0)             // the band's columns
    lw      s6, 156(a0)             // a column's bytes of 1 / depths
    li      t0, 0x3f800000
    fmv.w.x ft10, t0                // 1
    lw      t6, 148(a0)             // the unit's first row in the band
    lw      s2, 128(a0)             // the column
.Lq_row\@:  // the unit from row t6, from column s2
    bge     t6, s8, .Lq_all\@
    slli    t1, t6, 2
    lw      t0, 40(a0)
    add     t0, t0, t1
    flw     fs0, 0(t0)              // its first row's rowU (round, rowX)
    lw      t0, 44(a0)
    add     t0, t0, t1
    flw     fs1, 0(t0)              // rowV (rowY)
    lw      t0, 48(a0)
    add     t0, t0, t1
    flw     fs2, 0(t0)              // rowFar
    lw      t0, 52(a0)
    add     t0, t0, t1
    flw     fs3, 0(t0)              // flatZ
    lw      t1, 136(a0)             // y0
    sub     t1, t6, t1
    lw      s0, 160(a0)             // color
    mul     t2, t1, s5
    add     s0, s0, t2
    slli    t2, s2, 3
    add     s0, s0, t2              // px: the column's four pixels on the row
    lw      s1, 152(a0)             // zs
    mul     t2, s2, s6
    add     s1, s1, t2
    slli    t1, t1, 1
    add     s1, s1, t1              // its 1 / depth there
    lw      t0, 140(a0)             // topFlat
    slli    t1, s2, 2
    add     s4, t0, t1
    lw      t0, 144(a0)             // colUV
    slli    t1, s2, 3
    add     s7, t0, t1
.Lq_col\@:  // column s2 on the unit
    bge     s2, s3, .Lq_unit_done\@
    lw      t0, 0(s4)
    slli    t1, t0, 16
    srai    t1, t1, 16              // the column's highest row of land
    QUAD_END t3
    bge     t1, t3, .Lq_next\@      // none in the unit
    blt     t6, t1, .Lq_first\@     // from a row in it
    TERRAIN_SAMPLE fs0, fs1, fs2, fs3, 0, t6, .Lq_next\@, .Lq_hand_lo\@, \round
    beq     t1, a3, 1f
    jal     t5, .Lq_lookE\@
1:
    beq     t1, a5, 2f
    jal     t5, .Lq_lookO\@
2:
    QUAD_END t3
    sub     t2, t3, t6
    li      t4, 4
    bne     t2, t4, 3f
    sw      a4, 0(s0)               // a whole unit: even, odd, even, odd
    sw      a4, 4(s0)
    add     t0, s0, s5
    sw      a6, 0(t0)
    sw      a6, 4(t0)
    add     t0, t0, s5
    sw      a4, 0(t0)
    sw      a4, 4(t0)
    add     t0, t0, s5
    sw      a6, 0(t0)
    sw      a6, 4(t0)
.Lq_next\@:  // the next column
    addi    s0, s0, 8
    add     s1, s1, s6
    addi    s4, s4, 4
    addi    s7, s7, 8
    addi    s2, s2, 1
    j       .Lq_col\@
3:  // fewer rows, at the band's edge
    mv      t0, s0
    mv      t2, t6
    QUAD_ROWS
    j       .Lq_next\@
.Lq_first\@:  // the column's land from row t1 in the unit: its row's values, its own sample
    sub     t2, t1, t6
    slli    t2, t2, 1
    add     t5, s1, t2              // its 1 / depth's place
    slli    t4, t1, 2
    lw      t0, 40(a0)
    add     t0, t0, t4
    flw     fs4, 0(t0)
    lw      t0, 44(a0)
    add     t0, t0, t4
    flw     fs5, 0(t0)
    lw      t0, 48(a0)
    add     t0, t0, t4
    flw     fs6, 0(t0)
    lw      t0, 52(a0)
    add     t0, t0, t4
    flw     fs7, 0(t0)
    lw      t0, 0(s4)
    TERRAIN_SAMPLE fs4, fs5, fs6, fs7, 0, t1, .Lq_next\@, .Lq_hand_first\@, \round, t5
    beq     t1, a3, 4f
    jal     t5, .Lq_lookE\@
4:
    beq     t1, a5, 5f
    jal     t5, .Lq_lookO\@
5:
    lw      t0, 0(s4)
    slli    t2, t0, 16
    srai    t2, t2, 16              // its first row
    QUAD_END t3
    sub     t4, t2, t6
    mul     t4, t4, s5
    add     t0, s0, t4              // its pixels there
    QUAD_ROWS
    j       .Lq_next\@
.Lq_unit_done\@:
    andi    t6, t6, -4
    addi    t6, t6, 4               // the next unit's first row
    li      s2, 0
    j       .Lq_row\@
.Lq_lookE\@:  // the even rows' pair for key t1, from their cache
    andi    t3, t1, 255
    srli    t4, t1, 8
    slli    t2, t4, 5
    add     t2, t2, t4
    slli    t4, t4, 2
    add     t4, t4, t2              // haze 37
    xor     t3, t3, t4
    andi    t3, t3, 127
    lw      t0, 120(a0)             // pairs[0]
    slli    t4, t3, 1
    add     t4, t4, t0
    lhu     t4, 4(t4)               // its key
    bne     t4, t1, 6f              // not in it: worked out
    slli    t4, t3, 2
    add     t4, t4, t0
    lw      a4, 260(t4)             // its pair
    mv      a3, t1
    jr      t5
6:
    MAKE_PAIR 0, 127, a4
    mv      a3, t1
    jr      t5
.Lq_lookO\@:  // the odd rows'
    andi    t3, t1, 255
    srli    t4, t1, 8
    slli    t2, t4, 5
    add     t2, t2, t4
    slli    t4, t4, 2
    add     t4, t4, t2
    xor     t3, t3, t4
    andi    t3, t3, 127
    lw      t0, 124(a0)             // pairs[1]
    slli    t4, t3, 1
    add     t4, t4, t0
    lhu     t4, 4(t4)
    bne     t4, t1, 7f
    slli    t4, t3, 2
    add     t4, t4, t0
    lw      a6, 260(t4)
    mv      a5, t1
    jr      t5
7:
    MAKE_PAIR 191, 63, a6
    mv      a5, t1
    jr      t5
.Lq_all\@:  // the band done
    li      t5, 0
    sw      a7, 116(a0)
    j       .Lq_out\@
.Lq_hand_lo\@:  // the long way, the sample on the unit's first row
    mv      t3, t6
    mv      t4, s0
    j       .Lq_hand\@
.Lq_hand_first\@:  // on the column's first row
    lw      t0, 0(s4)
    slli    t3, t0, 16
    srai    t3, t3, 16
    sub     t4, t3, t6
    mul     t4, t4, s5
    add     t4, t4, s0
.Lq_hand\@:  // back to C at row t3, its pixels t4; on from the next column
    sw      t4, 0(a0)               // px
    slli    t0, t6, 1
    sub     t0, s1, t0
    sw      t0, 4(a0)               // zc, by row
    sw      t3, 8(a0)               // y
    lw      t0, 0(s4)
    srai    t0, t0, 16
    sw      t0, 16(a0)              // the flat top
    flw     ft2, 0(s7)
    flw     ft3, 4(s7)
    fsw     ft2, 24(a0)             // cu
    fsw     ft3, 28(a0)             // cv
    sw      a3, 100(a0)
    sw      a4, 104(a0)
    sw      a5, 108(a0)
    sw      a6, 112(a0)
    sw      a7, 116(a0)
    addi    t0, s2, 1
    sw      t0, 128(a0)             // on from the next column
    sw      t6, 148(a0)             // in this unit
    li      t5, 1
.Lq_out\@:
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
    flw     fs0, 48(sp)
    flw     fs1, 52(sp)
    flw     fs2, 56(sp)
    flw     fs3, 60(sp)
    flw     fs4, 64(sp)
    flw     fs5, 68(sp)
    flw     fs6, 72(sp)
    flw     fs7, 76(sp)
.if \round
    flw     fs8, 96(sp)
    flw     fs9, 100(sp)
    flw     fs10, 104(sp)
    flw     fs11, 108(sp)
.endif
    addi    sp, sp, 112
    ret
    .size   \name, .-\name
.endm

    TERRAIN_QUAD_FN scene3d_terrain_quad, 0
    TERRAIN_QUAD_FN scene3d_terrain_quad_round, 1

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
