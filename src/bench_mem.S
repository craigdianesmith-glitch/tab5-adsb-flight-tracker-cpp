// What the P4's loads do when they miss - for the bench (##iM, bench3d's
// 'M'), compiled only into its builds: how long a line takes from each
// level of memory, whether the core stops at a load that misses or only at
// the first instruction that wants its value, and whether two or four
// misses can be on their way at once. Each walks chains of pointers - one
// to a cache line, in an order made up so that nothing guesses the next -
// timed with the cycle counter. The loops are written out here, not left
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

#endif
