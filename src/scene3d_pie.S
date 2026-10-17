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
