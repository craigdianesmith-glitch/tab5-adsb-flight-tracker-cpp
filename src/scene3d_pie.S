// The innermost loop of the 3D view's drawing (see scene3d.cpp's span()),
// eight pixels at a time with the ESP32-P4's PIE vector instructions: 128-bit
// registers q0-q7 holding eight 16-bit lanes each.
//
// void scene3d_span8(uint16_t *c, uint16_t *z, int chunks, int32_t at, int32_t step8,
//                    const uint16_t *ramp, const uint16_t *color);
//
// c and z - a row's colours and depths from a pixel whose x is a multiple of
// 8 - and ramp and color are 16-byte aligned, as the vector loads and stores
// need. For each chunk of eight: the triangle's depth in lane i is
// (at >> 16) + ramp[i]; where that is greater - nearer - than the depth there,
// it and the colour replace what is there. Then at goes on by step8. The
// lanes are signed: a ramp goes down as often as up, and depths stay under
// 32768.
//
// Plain C does a pixel in some six or seven instructions, a compare and a
// branch among them; this does eight in sixteen, the loop's own three among
// them, without a branch.
//
// A PIE load or store takes its address only from x8-x15 or x24-x31 - s0,
// s1, a0-a5, s8-s11, t3-t6 - so the colour's pointer, which comes in a6, and
// the broadcast's, are moved to t3 and t4.

    .text
    .align  2
    .global scene3d_span8
    .type   scene3d_span8, @function
scene3d_span8:
    blez    a2, 2f                  // none, or fewer
    esp.vld.128.ip  q4, a5, 0       // the ramp
    mv      t3, a6
    esp.vld.128.ip  q6, t3, 0       // the colour, in every lane
    addi    sp, sp, -16             // somewhere to broadcast each chunk's depth from
    mv      t4, sp
1:
    srai    t0, a3, 16
    sh      t0, 0(t4)
    esp.vldbc.16.ip q3, t4, 0       // the chunk's depth in every lane...
    esp.vadd.s16    q3, q3, q4      // ...plus each lane's share of the slope
    esp.vld.128.ip  q0, a1, 0       // the depths there
    esp.vcmp.gt.s16 q1, q3, q0      // all ones where this is nearer
    esp.vmax.s16    q0, q3, q0      // the nearer depth, either way
    esp.vst.128.ip  q0, a1, 16
    esp.vld.128.ip  q2, a0, 0       // the colours there
    esp.xorq        q5, q2, q6      // where they differ from this colour...
    esp.andq        q5, q5, q1      // ...where it is nearer...
    esp.xorq        q2, q2, q5      // ...flipped to it: three, not and-not-and-or's four
    esp.vst.128.ip  q2, a0, 16
    add     a3, a3, a4
    addi    a2, a2, -1
    bnez    a2, 1b
    addi    sp, sp, 16
2:
    ret
    .size   scene3d_span8, .-scene3d_span8

// void scene3d_fill8(uint16_t *row, int chunks, const uint32_t *pattern);
//
// Fills eight pixels a store, chunks times over, from a 16-byte aligned place
// in a row: the sky and grass behind each row, and the ground's long spans.
// `pattern` is the eight pixels - a dithered pair of colours four times over -
// 16-byte aligned. Four stores to a turn of the loop, then what is left.
    .align  2
    .global scene3d_fill8
    .type   scene3d_fill8, @function
scene3d_fill8:
    blez    a1, 3f                  // none, or fewer
    esp.vld.128.ip  q0, a2, 0
    srai    t0, a1, 2
    andi    a1, a1, 3
    beqz    t0, 2f
1:
    esp.vst.128.ip  q0, a0, 16
    esp.vst.128.ip  q0, a0, 16
    esp.vst.128.ip  q0, a0, 16
    esp.vst.128.ip  q0, a0, 16
    addi    t0, t0, -1
    bnez    t0, 1b
2:
    beqz    a1, 3f
    esp.vst.128.ip  q0, a0, 16
    addi    a1, a1, -1
    j       2b
3:
    ret
    .size   scene3d_fill8, .-scene3d_fill8

// void scene3d_darken8(uint16_t *c, const uint16_t *m, int chunks, const uint16_t *constants);
//
// The shadows: where the mask `m` isn't nought, a pixel's red, green and blue
// each times k/256, eight pixels at a time - the sums of scene3d.cpp's
// darken(), whole numbers rounded down. Each channel is masked out where it
// is, multiplied by k with the product shifted down by 8 (SAR), and masked
// again, which is (r*k>>8)<<11 and the rest exactly, having no 16-bit shifts
// to take it out and put it back with. The pixels are high byte first, as
// the panel takes them: turned round to sum on, by a byte unzip and zip, and
// back again. `c` and `m` 16-byte aligned; `constants` four rows of eight
// lanes, 16-byte aligned: k, then the masks of red, green and blue.
    .align  2
    .global scene3d_darken8
    .type   scene3d_darken8, @function
scene3d_darken8:
    blez    a2, 2f                  // none, or fewer
    esp.vld.128.ip  q7, a3, 16      // k
    esp.vld.128.ip  q4, a3, 16      // red's mask
    esp.vld.128.ip  q5, a3, 16      // green's
    esp.vld.128.ip  q6, a3, 0       // blue's
    li      t3, 8                   // (a PIE register, as the addresses are)
    esp.movx.w.sar  t3              // the products shifted down by 8: times k/256
    esp.zero.q      q1
1:
    esp.vld.128.ip  q0, a0, 0       // the pixels, high byte first
    esp.vunzip.8    q0, q1          // q1 nought: q0 their high bytes, q1 their low
    esp.vzip.8      q1, q0          // q1 the pixels low byte first, q0 nought
    esp.andq        q2, q1, q4      // red...
    esp.andq        q3, q1, q5      // ...green...
    esp.vmul.u16    q2, q2, q7
    esp.vmul.u16    q3, q3, q7
    esp.andq        q1, q1, q6      // ...blue, the pixels done with
    esp.andq        q2, q2, q4
    esp.vmul.u16    q1, q1, q7
    esp.andq        q3, q3, q5
    esp.orq         q2, q2, q3
    esp.andq        q1, q1, q6
    esp.orq         q2, q2, q1      // darkened, low byte first
    esp.vunzip.8    q2, q0          // q0 nought: q2 the low bytes, q0 the high
    esp.vzip.8      q0, q2          // q0 darkened, high byte first; q2 nought
    esp.vld.128.ip  q3, a1, 16      // the mask...
    esp.vcmp.eq.u16 q3, q3, q2      // ...all ones where it is nought, to be left as they were
    esp.vld.128.ip  q1, a0, 0       // the pixels again, as they were
    esp.xorq        q1, q1, q0      // where they differ from the darkened...
    esp.andq        q1, q1, q3      // ...and are to be left...
    esp.xorq        q0, q0, q1      // ...put back
    esp.vst.128.ip  q0, a0, 16
    esp.zero.q      q1
    addi    a2, a2, -1
    bnez    a2, 1b
2:
    ret
    .size   scene3d_darken8, .-scene3d_darken8

// void scene3d_tint8(uint16_t *c, const uint16_t *z, int chunks, int32_t at, int32_t step8,
//                    const uint16_t *constants, const uint16_t *head, const uint16_t *tail);
//
// The see-through - contrails, and blades' blur - eight pixels at a time,
// every span however short: where it isn't behind what is drawn, a pixel's
// red, green and blue each keep (8 - e)/8 of what they were and take e/8 of
// the tint's, whole numbers rounded down - scene3d.cpp's tintC() exactly.
// `c` and `z` at the span's first chunk of eight, 16-byte aligned; `at` the
// depth at that chunk's first pixel, in 16.16 fixed point, going on by
// step8 a chunk; lane i's depth (at >> 16) + ramp[i], as scene3d_span8 has
// a solid's - drawn where that isn't less than the depth there, which is
// left as it was. `head` the first chunk's lanes before the span, all ones,
// `tail` the last chunk's after it: left as they are too.
//
// Each channel is masked out where it is, multiplied by 8 - e with the
// product shifted down by 3 (SAR), and masked again; the tint's share of
// each, worked out once for the triangle, added after - each channel's sum
// within its own bits. The pixels high byte first, as the panel takes them:
// turned round by a byte unzip and zip, and back. `constants` six rows of
// eight lanes, 16-byte aligned: the ramp, 8 - e, the masks of red, green
// and blue, and the tint's share. Thirty-eight instructions a chunk, the
// loop's three among them, four more for the last; C took some thirty a
// pixel, the old way a float's depth and a division by 8 three times over.
//
// A PIE load or store takes its address only from x8-x15 or x24-x31: the
// count, depth and step to t0-t2, and a2 and a3 the edges' lanes.
    .align  2
    .global scene3d_tint8
    .type   scene3d_tint8, @function
scene3d_tint8:
    blez    a2, 9f                  // none, or fewer
    mv      t0, a2                  // chunks
    mv      t1, a3                  // the depth
    mv      t2, a4                  // its step a chunk
    mv      a2, a6                  // the first chunk's lanes before the span
    mv      a3, a7                  // the last's after it
    esp.vld.128.ip  q6, a5, 16      // the ramp
    esp.vld.128.ip  q7, a5, 16      // 8 - e; a5 then at red's mask
    li      t3, 3                   // (a PIE register, as the addresses are)
    esp.movx.w.sar  t3              // the products shifted down by 3: times (8 - e)/8
    addi    sp, sp, -32
    mv      t4, sp                  // the chunk's depth, to broadcast from
    addi    t6, sp, 16              // the chunk's lanes outside the span
    esp.vld.128.ip  q0, a2, 0
    esp.vst.128.ip  q0, t6, 0
1:
    li      a4, 1
    bne     t0, a4, 3f
    esp.vld.128.ip  q0, t6, 0       // the last chunk: its lanes after the span too
    esp.vld.128.ip  q1, a3, 0
    esp.orq         q0, q0, q1
    esp.vst.128.ip  q0, t6, 0
3:
    srai    a4, t1, 16
    sh      a4, 0(t4)
    esp.vldbc.16.ip q4, t4, 0       // the chunk's depth in every lane...
    esp.vadd.s16    q4, q4, q6      // ...plus each lane's share of the slope
    esp.vld.128.ip  q5, a1, 16      // the depths there
    esp.vcmp.lt.s16 q5, q4, q5      // all ones where it is behind them...
    esp.vld.128.ip  q4, t6, 0
    esp.orq         q5, q5, q4      // ...or outside the span: left as they are
    esp.vld.128.ip  q0, a0, 0       // the pixels, high byte first
    esp.zero.q      q1
    esp.vunzip.8    q0, q1          // q1 nought: q0 their high bytes, q1 their low
    esp.vzip.8      q1, q0          // q1 the pixels low byte first, q0 nought
    esp.vld.128.ip  q4, a5, 16      // red's mask
    esp.andq        q2, q1, q4
    esp.vmul.u16    q2, q2, q7
    esp.andq        q2, q2, q4
    esp.vld.128.ip  q4, a5, 16      // green's
    esp.andq        q3, q1, q4
    esp.vmul.u16    q3, q3, q7
    esp.andq        q3, q3, q4
    esp.orq         q2, q2, q3
    esp.vld.128.ip  q4, a5, 16      // blue's
    esp.andq        q3, q1, q4
    esp.vmul.u16    q3, q3, q7
    esp.andq        q3, q3, q4
    esp.orq         q2, q2, q3
    esp.vld.128.ip  q4, a5, -48     // the tint's share; a5 back at red's mask
    esp.vadd.u16    q2, q2, q4      // tinted, low byte first
    esp.vunzip.8    q2, q0          // q0 nought: q2 the low bytes, q0 the high
    esp.vzip.8      q0, q2          // q0 tinted, high byte first
    esp.vld.128.ip  q1, a0, 0       // the pixels again, as they were
    esp.xorq        q1, q1, q0      // where they differ from the tinted...
    esp.andq        q1, q1, q5      // ...and are to be left...
    esp.xorq        q0, q0, q1      // ...put back
    esp.vst.128.ip  q0, a0, 16
    esp.zero.q      q4              // past the first chunk, none outside but the last's
    esp.vst.128.ip  q4, t6, 0
    add     t1, t1, t2
    addi    t0, t0, -1
    bnez    t0, 1b
    addi    sp, sp, 32
9:
    ret
    .size   scene3d_tint8, .-scene3d_tint8

// void scene3d_solid(const Trapezoid *t);
//
// A solid triangle's rows in a band - or a trapezoid of them, its top corner
// to its middle one, or the middle to the bottom: scene3d.cpp's
// solidRowsC() exactly. Each row's first pixel xl >> 16, and the one after
// its last xr >> 16, the edges stepped down a row at a time in 16.16 fixed
// point; its depth at the first, 16.16, fused-multiplied from the depth at
// x nought, in float, which steps down too. Then the row's pixels as
// span() has them: where that depth is greater - nearer - than the depth
// there, it and the colour replace what is there, a pixel at a time - or,
// 24 and more and PIE wanted, a pixel at a time to a multiple of 8, eight at
// a time as scene3d_span8 does, then the rest. A row's overhead some twenty
// instructions; GCC's raster() and span() round the same loops, a float
// edge each side and a call, took ten times that.
//
// The Trapezoid: c and z, row 0's colours and depths at x nought; rows; w;
// xl, dl, xr, dr; the depth at x nought, its step a row and a pixel (float);
// the depth's step a pixel, 16.16; the colour; PIE's ramp and colours,
// 16-byte aligned; whether PIE. A PIE load or store takes its address only
// from x8-x15 or x24-x31: s0, s1 and s9, saved, and t3 and t4.
    .align  2
    .global scene3d_solid
    .type   scene3d_solid, @function
scene3d_solid:
    lw      a1, 8(a0)               // rows
    blez    a1, 9f
    addi    sp, sp, -32
    sw      s0, 0(sp)
    sw      s1, 4(sp)
    sw      s8, 8(sp)
    sw      s9, 12(sp)
    mv      s8, a0
    lw      s0, 0(s8)               // the row's colours, at x nought
    lw      s1, 4(s8)               // its depths
    lw      a2, 12(s8)              // w
    lw      a3, 16(s8)              // xl
    lw      a4, 20(s8)              // dl
    lw      a5, 24(s8)              // xr
    lw      a6, 28(s8)              // dr
    flw     fa0, 32(s8)             // the depth at x nought
    flw     fa1, 36(s8)             // its step a row
    flw     fa2, 40(s8)             // and a pixel
    lw      a7, 44(s8)              // the depth's step a pixel, 16.16
    lhu     t0, 48(s8)              // the colour
    addi    s9, sp, 16              // a chunk's depth, to broadcast from
    lw      t2, 60(s8)
    beqz    t2, 1f
    lw      t3, 52(s8)
    esp.vld.128.ip  q4, t3, 0       // the ramp
    lw      t3, 56(s8)
    esp.vld.128.ip  q6, t3, 0       // the colour, in every lane
1:  // a row
    srai    t1, a3, 16              // its first pixel...
    srai    t5, a5, 16              // ...and the one after its last
    bgez    t1, 2f
    li      t1, 0
2:  ble     t5, a2, 3f
    mv      t5, a2
3:  bge     t1, t5, 8f              // none
    fcvt.s.w ft0, t1
    fmadd.s ft0, fa2, ft0, fa0
    fcvt.w.s t6, ft0, rtz           // the depth at the first, 16.16
    bgez    t6, 4f
    li      t6, 0
4:  slli    a0, t1, 1
    add     t3, s0, a0              // the colour there
    add     t4, s1, a0              // the depth there
    slli    a0, t5, 1
    add     a0, s1, a0              // the depths' end
    sub     t5, t5, t1              // how many
    lw      t2, 60(s8)
    beqz    t2, 6f
    li      t2, 24
    blt     t5, t2, 6f
    // Long, with PIE: to a multiple of 8 a pixel at a time...
    sw      a0, 24(sp)
    addi    a0, t1, 7
    andi    a0, a0, -8
    slli    a0, a0, 1
    add     a0, s1, a0              // the depths' at the multiple of 8
    beq     t4, a0, 52f
51: lhu     t5, 0(t4)
    srli    t1, t6, 16
    add     t6, t6, a7
    bgeu    t5, t1, 511f
    sh      t1, 0(t4)
    sh      t0, 0(t3)
511:
    addi    t3, t3, 2
    addi    t4, t4, 2
    bne     t4, a0, 51b
52: // ...eight at a time...
    lw      a0, 24(sp)              // the depths' end
    sub     t2, a0, t4
    srai    t2, t2, 4               // chunks
    beqz    t2, 6f
    slli    t1, a7, 3               // the step eight at a time
53: srai    t5, t6, 16
    sh      t5, 0(s9)
    esp.vldbc.16.ip q3, s9, 0       // the chunk's depth in every lane...
    esp.vadd.s16    q3, q3, q4      // ...plus each lane's share of the slope
    esp.vld.128.ip  q0, t4, 0       // the depths there
    esp.vcmp.gt.s16 q1, q3, q0      // all ones where this is nearer
    esp.vmax.s16    q0, q3, q0      // the nearer depth, either way
    esp.vst.128.ip  q0, t4, 16
    esp.vld.128.ip  q2, t3, 0       // the colours there
    esp.xorq        q5, q2, q6      // where they differ from this colour...
    esp.andq        q5, q5, q1      // ...where it is nearer...
    esp.xorq        q2, q2, q5      // ...flipped to it
    esp.vst.128.ip  q2, t3, 16
    add     t6, t6, t1
    addi    t2, t2, -1
    bnez    t2, 53b
6:  // ...and the rest - or all of a short row - a pixel at a time
    beq     t4, a0, 8f
7:  lhu     t5, 0(t4)
    srli    t1, t6, 16
    add     t6, t6, a7
    bgeu    t5, t1, 71f
    sh      t1, 0(t4)
    sh      t0, 0(t3)
71: addi    t3, t3, 2
    addi    t4, t4, 2
    bne     t4, a0, 7b
8:  // the next row
    add     a3, a3, a4
    add     a5, a5, a6
    fadd.s  fa0, fa0, fa1
    slli    t1, a2, 1
    add     s0, s0, t1
    add     s1, s1, t1
    addi    a1, a1, -1
    bnez    a1, 1b
    lw      s0, 0(sp)
    lw      s1, 4(sp)
    lw      s8, 8(sp)
    lw      s9, 12(sp)
    addi    sp, sp, 32
9:  ret
    .size   scene3d_solid, .-scene3d_solid

// void scene3d_hills8(uint16_t *row, const int16_t *top, const int16_t *foot, const uint16_t *hill, int y,
//                     int chunks);
//
// A range of hills along the skyline over row y, eight pixels at a time:
// where a column's top is at or above the row and its foot below it, the
// hill's colour there - scene3d.cpp's hillsRowC() exactly. `row`, `top`,
// `foot` and `hill` 16-byte aligned. Two compares and a select for the
// eight, where C tests each pixel on its own.
    .align  2
    .global scene3d_hills8
    .type   scene3d_hills8, @function
scene3d_hills8:
    blez    a5, 9f                  // none, or fewer
    addi    sp, sp, -16
    sh      a4, 0(sp)
    addi    t0, a4, 1
    sh      t0, 2(sp)
    mv      t3, sp
    esp.vldbc.16.ip q6, t3, 2       // y in every lane...
    esp.vldbc.16.ip q7, t3, 0       // ...and y + 1
1:
    esp.vld.128.ip  q0, a1, 16      // the columns' tops
    esp.vld.128.ip  q1, a2, 16      // and feet
    esp.vcmp.gt.s16 q2, q7, q0      // y + 1 > top: at or below the top...
    esp.vcmp.gt.s16 q3, q1, q6      // ...and foot > y: above the foot
    esp.andq        q2, q2, q3
    esp.vld.128.ip  q4, a0, 0       // the row as drawn
    esp.vld.128.ip  q5, a3, 16      // the hills' colours
    esp.xorq        q5, q5, q4      // where they differ...
    esp.andq        q5, q5, q2      // ...and the hill is...
    esp.xorq        q4, q4, q5      // ...the hill's
    esp.vst.128.ip  q4, a0, 16
    addi    a5, a5, -1
    bnez    a5, 1b
    addi    sp, sp, 16
9:
    ret
    .size   scene3d_hills8, .-scene3d_hills8

// void scene3d_hills8d(uint16_t *row, uint16_t *depth, const int16_t *top, const int16_t *foot,
//                      const uint16_t *hill, const uint16_t *hillDepth, int y, int chunks);
//
// scene3d_hills8, and the hill's depth put in the row's depth where its
// colour is put in the row - the same mask selecting both - for what is
// drawn after to be hidden behind it: hillsRowC() with its depth. All the
// pointers 16-byte aligned, and in x8-x15, as PIE takes its addresses.
    .align  2
    .global scene3d_hills8d
    .type   scene3d_hills8d, @function
scene3d_hills8d:
    blez    a7, 9f                  // none, or fewer
    addi    sp, sp, -16
    sh      a6, 0(sp)
    addi    t0, a6, 1
    sh      t0, 2(sp)
    mv      t3, sp
    esp.vldbc.16.ip q6, t3, 2       // y in every lane...
    esp.vldbc.16.ip q7, t3, 0       // ...and y + 1
1:
    esp.vld.128.ip  q0, a2, 16      // the columns' tops
    esp.vld.128.ip  q1, a3, 16      // and feet
    esp.vcmp.gt.s16 q2, q7, q0      // y + 1 > top: at or below the top...
    esp.vcmp.gt.s16 q3, q1, q6      // ...and foot > y: above the foot
    esp.andq        q2, q2, q3
    esp.vld.128.ip  q4, a0, 0       // the row as drawn
    esp.vld.128.ip  q5, a4, 16      // the hills' colours
    esp.xorq        q5, q5, q4      // where they differ...
    esp.andq        q5, q5, q2      // ...and the hill is...
    esp.xorq        q4, q4, q5      // ...the hill's
    esp.vst.128.ip  q4, a0, 16
    esp.vld.128.ip  q0, a1, 0       // the row's depth
    esp.vld.128.ip  q1, a5, 16      // and the hills'
    esp.xorq        q1, q1, q0
    esp.andq        q1, q1, q2
    esp.xorq        q0, q0, q1
    esp.vst.128.ip  q0, a1, 16
    addi    a7, a7, -1
    bnez    a7, 1b
    addi    sp, sp, 16
9:
    ret
    .size   scene3d_hills8d, .-scene3d_hills8d
