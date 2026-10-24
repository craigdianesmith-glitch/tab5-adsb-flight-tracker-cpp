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
