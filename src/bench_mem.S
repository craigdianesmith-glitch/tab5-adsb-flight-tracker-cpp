// What the P4's loads do when they miss - for the bench (##iM, bench3d's
// 'M'), compiled only into its builds: how long a line takes from each
// level of memory, whether the core stops at a load that misses or only at
// the first instruction that wants its value, and whether two or four
// misses can be on their way at once. Each walks chains of pointers - one
// to a cache line, in an order made up so that nothing guesses the next -
// timed with the cycle counter. And the latencies of the instructions the
// terrain's loops wait on (##iT). The loops are written out here, not left
// to the compiler, so that what is measured is exactly what is written.

#if defined(BENCH3D)

// uint32_t bench_mem_chase(void **p, int n): n loads, each the address of
// the next - the latency of one, n times over.
    .text
    .align  2
    .global bench_mem_chase
    .type   bench_mem_chase, @function
bench_mem_chase:
    csrr    t0, mcycle
1:
    lw      a0, 0(a0)
    addi    a1, a1, -1
    bnez    a1, 1b
    csrr    t1, mcycle
    sub     a0, t1, t0
    ret
    .size   bench_mem_chase, .-bench_mem_chase

// uint32_t bench_mem_chase2(void **p, void **q, int n): two chains at once,
// a load of each a turn, neither waiting on the other's.
    .align  2
    .global bench_mem_chase2
    .type   bench_mem_chase2, @function
bench_mem_chase2:
    csrr    t0, mcycle
1:
    lw      a0, 0(a0)
    lw      a1, 0(a1)
    addi    a2, a2, -1
    bnez    a2, 1b
    csrr    t1, mcycle
    sub     a0, t1, t0
    ret
    .size   bench_mem_chase2, .-bench_mem_chase2

// uint32_t bench_mem_chase4(void **p[4], int n): four chains at once.
    .align  2
    .global bench_mem_chase4
    .type   bench_mem_chase4, @function
bench_mem_chase4:
    lw      a2, 0(a0)
    lw      a3, 4(a0)
    lw      a4, 8(a0)
    lw      a5, 12(a0)
    csrr    t0, mcycle
1:
    lw      a2, 0(a2)
    lw      a3, 0(a3)
    lw      a4, 0(a4)
    lw      a5, 0(a5)
    addi    a1, a1, -1
    bnez    a1, 1b
    csrr    t1, mcycle
    sub     a0, t1, t0
    ret
    .size   bench_mem_chase4, .-bench_mem_chase4

// uint32_t bench_mem_shadow(void **p, int n): a chain's load, then 128
// instructions that don't want it, then the next load, which does: if the
// core stops at the load until the line comes, each turn takes the line's
// time and the 128 more; if it stops only when the value is wanted, the 128
// go on under the wait.
    .align  2
    .global bench_mem_shadow
    .type   bench_mem_shadow, @function
bench_mem_shadow:
    li      t2, 0
    csrr    t0, mcycle
1:
    lw      a0, 0(a0)
    .rept   128
    addi    t2, t2, 1
    .endr
    addi    a1, a1, -1
    bnez    a1, 1b
    csrr    t1, mcycle
    sub     a0, t1, t0
    ret
    .size   bench_mem_shadow, .-bench_mem_shadow

// uint32_t bench_mem_work(int n): the 128 instructions and the loop alone,
// for the shadow's sums.
    .align  2
    .global bench_mem_work
    .type   bench_mem_work, @function
bench_mem_work:
    li      t2, 0
    csrr    t0, mcycle
1:
    .rept   128
    addi    t2, t2, 1
    .endr
    addi    a0, a0, -1
    bnez    a0, 1b
    csrr    t1, mcycle
    sub     a0, t1, t0
    ret
    .size   bench_mem_work, .-bench_mem_work

// void scene3d_latencies(uint32_t out[12]);
//
// The P4's latencies and throughputs the terrain's loops wait on (##iT's
// first line), each the best of four runs of 32 turns of eight - the
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

#endif
