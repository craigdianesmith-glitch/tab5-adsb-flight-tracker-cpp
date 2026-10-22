// The 3D view's land in what covers it (see scene3d.cpp's coverSpan()): the
// innermost loops, which coverCellsC() and coverWriteC() are the C of.
//
// void scene3d_cover_cells(const uint8_t *cells, int stride, int32_t cu, int32_t cv,
//                          int32_t dcu, int32_t dcv, int n, uint8_t *out, uint8_t *seen);
//
// n of a tile's cells, `stride` to its row, along a line from (cu, cv) in
// 16.16 fixed point stepping (dcu, dcv): cell i, cells[(cv >> 16) * stride +
// (cu >> 16)] there, to out[i], and seen[cell] marked - with the stride, never
// nought, saving a register for the 1 C writes.
//
// Sixteen or more, eight at a time with PIE: eight lanes of cu and of cv,
// 32 bits each, shifted down 16 for where in the tile each is, the eight
// columns and the eight rows then unzipped into 16-bit lanes, the rows
// multiplied by the stride and the columns added - every cell's place, in
// eleven instructions for the eight - then each read with a load of its own,
// PIE having nothing to gather with. Eight and a half instructions a cell,
// against GCC's thirteen; the same cells exactly, the sums being the same
// sums. What is left, and fewer than sixteen, one or two at a time: eleven
// instructions a cell and the loop's three a turn of two, each multiply and
// load put where its result isn't wanted for a cycle or two.
//
// A PIE load takes its address only from x8-x15 or x24-x31, a lane is moved
// out only to one of them, and SAR is set from one of them too: t3 the
// broadcasts' address - each load moving it on to the next, and the last
// back to the first - t4 and t5 the lanes, a2 and a3 the shifts: 16 for the
// positions, nought for the multiply.

    .text
    .align  2
    .global scene3d_cover_cells
    .type   scene3d_cover_cells, @function
scene3d_cover_cells:
    lw      t6, 0(sp)               // seen
    li      t0, 16
    blt     a6, t0, 5f
    // The lanes from the first cell on, then the steps eight at a time and
    // the stride, on the stack to load from.
    addi    sp, sp, -80
    mv      t0, a2
    mv      t1, a3
    sw      t0, 0(sp)
    sw      t1, 32(sp)
    add     t0, t0, a4
    add     t1, t1, a5
    sw      t0, 4(sp)
    sw      t1, 36(sp)
    add     t0, t0, a4
    add     t1, t1, a5
    sw      t0, 8(sp)
    sw      t1, 40(sp)
    add     t0, t0, a4
    add     t1, t1, a5
    sw      t0, 12(sp)
    sw      t1, 44(sp)
    add     t0, t0, a4
    add     t1, t1, a5
    sw      t0, 16(sp)
    sw      t1, 48(sp)
    add     t0, t0, a4
    add     t1, t1, a5
    sw      t0, 20(sp)
    sw      t1, 52(sp)
    add     t0, t0, a4
    add     t1, t1, a5
    sw      t0, 24(sp)
    sw      t1, 56(sp)
    add     t0, t0, a4
    add     t1, t1, a5
    sw      t0, 28(sp)
    sw      t1, 60(sp)
    slli    t0, a4, 3
    slli    t1, a5, 3
    sh      a1, 64(sp)
    sw      t0, 68(sp)
    sw      t1, 72(sp)
    mv      t3, sp
    esp.vld.128.ip  q0, t3, 16      // cu, the first four's
    esp.vld.128.ip  q1, t3, 16      // and the next four's
    esp.vld.128.ip  q2, t3, 16      // cv
    esp.vld.128.ip  q3, t3, 16      // t3 at the stride, then the steps
    li      a2, 16
    li      a3, 0
    esp.movx.w.sar  a2
    srai    t2, a6, 3               // turns of eight
    andi    a6, a6, 7               // and what is left after them
4:
    esp.vsr.s32     q4, q0          // the columns, 32 bits a lane...
    esp.vsr.s32     q5, q1
    esp.vsr.s32     q6, q2          // ...and the rows
    esp.vsr.s32     q7, q3
    esp.vunzip.16   q4, q5          // the eight columns, 16 bits a lane
    esp.vunzip.16   q6, q7          // the eight rows
    esp.vldbc.16.ip q5, t3, 4       // the stride, in every lane
    esp.movx.w.sar  a3
    esp.vmul.u16    q6, q6, q5      // the rows by the stride...
    esp.movx.w.sar  a2
    esp.vadd.s16    q6, q6, q4      // ...and the columns: each cell's place
    esp.vldbc.32.ip q4, t3, 4       // on eight
    esp.vldbc.32.ip q5, t3, -8      // t3 back at the stride
    esp.vadd.s32    q0, q0, q4
    esp.vadd.s32    q1, q1, q4
    esp.vadd.s32    q2, q2, q5
    esp.vadd.s32    q3, q3, q5
    esp.movi.16.a   q6, t4, 0
    esp.movi.16.a   q6, t5, 1
    add     t4, t4, a0
    add     t5, t5, a0
    lbu     t4, 0(t4)
    lbu     t5, 0(t5)
    sb      t4, 0(a7)
    sb      t5, 1(a7)
    add     t4, t4, t6
    add     t5, t5, t6
    sb      a1, 0(t4)
    sb      a1, 0(t5)
    esp.movi.16.a   q6, t4, 2
    esp.movi.16.a   q6, t5, 3
    add     t4, t4, a0
    add     t5, t5, a0
    lbu     t4, 0(t4)
    lbu     t5, 0(t5)
    sb      t4, 2(a7)
    sb      t5, 3(a7)
    add     t4, t4, t6
    add     t5, t5, t6
    sb      a1, 0(t4)
    sb      a1, 0(t5)
    esp.movi.16.a   q6, t4, 4
    esp.movi.16.a   q6, t5, 5
    add     t4, t4, a0
    add     t5, t5, a0
    lbu     t4, 0(t4)
    lbu     t5, 0(t5)
    sb      t4, 4(a7)
    sb      t5, 5(a7)
    add     t4, t4, t6
    add     t5, t5, t6
    sb      a1, 0(t4)
    sb      a1, 0(t5)
    esp.movi.16.a   q6, t4, 6
    esp.movi.16.a   q6, t5, 7
    add     t4, t4, a0
    add     t5, t5, a0
    lbu     t4, 0(t4)
    lbu     t5, 0(t5)
    sb      t4, 6(a7)
    sb      t5, 7(a7)
    add     t4, t4, t6
    add     t5, t5, t6
    sb      a1, 0(t4)
    sb      a1, 0(t5)
    addi    a7, a7, 8
    addi    t2, t2, -1
    bnez    t2, 4b
    // Where the lanes got to: the next cell's place, for what is left.
    esp.movi.32.a   q0, a2, 0
    esp.movi.32.a   q2, a3, 0
    addi    sp, sp, 80
5:
    andi    t0, a6, 1
    beqz    t0, 1f
    // An odd one, first.
    srai    t0, a3, 16
    srai    t1, a2, 16
    mul     t0, t0, a1
    add     t1, t1, a0
    add     a2, a2, a4
    add     a3, a3, a5
    add     t0, t0, t1
    lbu     t0, 0(t0)
    addi    a7, a7, 1
    addi    a6, a6, -1
    sb      t0, -1(a7)
    add     t0, t0, t6
    sb      a1, 0(t0)
1:
    srai    a6, a6, 1               // turns
    beqz    a6, 3f
    add     t4, a2, a4              // the second's place
    add     t5, a3, a5
    slli    a4, a4, 1               // the steps two at a time
    slli    a5, a5, 1
2:
    srai    t0, a3, 16              // the first's row
    srai    t2, t5, 16              // the second's
    srai    t1, a2, 16              // their columns
    srai    t3, t4, 16
    mul     t0, t0, a1
    mul     t2, t2, a1
    add     t1, t1, a0
    add     t3, t3, a0
    add     a2, a2, a4              // on two
    add     a3, a3, a5
    add     t0, t0, t1
    add     t2, t2, t3
    lbu     t0, 0(t0)
    lbu     t2, 0(t2)
    add     t4, t4, a4
    add     t5, t5, a5
    sb      t0, 0(a7)
    sb      t2, 1(a7)
    add     t0, t0, t6
    add     t2, t2, t6
    sb      a1, 0(t0)
    sb      a1, 0(t2)
    addi    a7, a7, 2
    addi    a6, a6, -1
    bnez    a6, 2b
3:
    ret
    .size   scene3d_cover_cells, .-scene3d_cover_cells

// void scene3d_cover_write(uint16_t *row, const uint8_t *cells, const uint32_t *pairs, int n);
//
// n samples of four pixels, from `row` - 8-byte aligned - each four the
// colour pair of its cell, pairs[cells[i]], twice over: with PIE, the pair
// loaded into every lane of a register and the low 64 bits of it stored, an
// instruction each, where C loads it and stores it twice. Four samples -
// sixteen pixels - a turn of the loop, their loads first, so none waits on
// the one before; then what is left, one at a time. A PIE load or store
// takes its address only from x8-x15 or x24-x31: a0 and t3-t6.

    .align  2
    .global scene3d_cover_write
    .type   scene3d_cover_write, @function
scene3d_cover_write:
    blez    a3, 9f
    andi    t2, a3, 3
    srai    a3, a3, 2
    beqz    a3, 2f
1:
    lbu     t0, 0(a1)
    lbu     t1, 1(a1)
    lbu     a4, 2(a1)
    lbu     a5, 3(a1)
    slli    t0, t0, 2
    slli    t1, t1, 2
    slli    a4, a4, 2
    slli    a5, a5, 2
    add     t3, a2, t0
    add     t4, a2, t1
    add     t5, a2, a4
    add     t6, a2, a5
    esp.vldbc.32.ip q0, t3, 0
    esp.vldbc.32.ip q1, t4, 0
    esp.vldbc.32.ip q2, t5, 0
    esp.vldbc.32.ip q3, t6, 0
    esp.vst.l.64.ip q0, a0, 8
    esp.vst.l.64.ip q1, a0, 8
    esp.vst.l.64.ip q2, a0, 8
    esp.vst.l.64.ip q3, a0, 8
    addi    a1, a1, 4
    addi    a3, a3, -1
    bnez    a3, 1b
2:
    beqz    t2, 9f
3:
    lbu     t0, 0(a1)
    slli    t0, t0, 2
    add     t3, a2, t0
    esp.vldbc.32.ip q0, t3, 0
    esp.vst.l.64.ip q0, a0, 8
    addi    a1, a1, 1
    addi    t2, t2, -1
    bnez    t2, 3b
9:
    ret
    .size   scene3d_cover_write, .-scene3d_cover_write

// void scene3d_cover_write8(uint16_t *row, const uint8_t *cells, const uint32_t *pairs, int n);
//
// scene3d_cover_write for samples of eight pixels - where the near land's
// cells are wide enough for a sample every eight: each the pair broadcast to
// every lane and all 128 bits of it stored, two instructions for eight
// pixels. `row` 16-byte aligned. Four samples a turn, their loads first.
    .align  2
    .global scene3d_cover_write8
    .type   scene3d_cover_write8, @function
scene3d_cover_write8:
    blez    a3, 9f
    andi    t2, a3, 3
    srai    a3, a3, 2
    beqz    a3, 2f
1:
    lbu     t0, 0(a1)
    lbu     t1, 1(a1)
    lbu     a4, 2(a1)
    lbu     a5, 3(a1)
    slli    t0, t0, 2
    slli    t1, t1, 2
    slli    a4, a4, 2
    slli    a5, a5, 2
    add     t3, a2, t0
    add     t4, a2, t1
    add     t5, a2, a4
    add     t6, a2, a5
    esp.vldbc.32.ip q0, t3, 0
    esp.vldbc.32.ip q1, t4, 0
    esp.vldbc.32.ip q2, t5, 0
    esp.vldbc.32.ip q3, t6, 0
    esp.vst.128.ip  q0, a0, 16
    esp.vst.128.ip  q1, a0, 16
    esp.vst.128.ip  q2, a0, 16
    esp.vst.128.ip  q3, a0, 16
    addi    a1, a1, 4
    addi    a3, a3, -1
    bnez    a3, 1b
2:
    beqz    t2, 9f
3:
    lbu     t0, 0(a1)
    slli    t0, t0, 2
    add     t3, a2, t0
    esp.vldbc.32.ip q0, t3, 0
    esp.vst.128.ip  q0, a0, 16
    addi    a1, a1, 1
    addi    t2, t2, -1
    bnez    t2, 3b
9:
    ret
    .size   scene3d_cover_write8, .-scene3d_cover_write8
