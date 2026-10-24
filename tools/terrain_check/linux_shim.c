// Newlib's hooks for ESP's riscv32-esp-elf toolchain, as Linux riscv32 system
// calls - so that a program built with it runs under qemu-riscv32 (user mode)
// on the PC: the terrain's assembly checked against its C without the Tab5.
#include <errno.h>
#include <reent.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#undef errno
extern int errno;

static long sys(long n, long a, long b, long c, long d, long e) {
    register long a0 asm("a0") = a, a1 asm("a1") = b, a2 asm("a2") = c, a3 asm("a3") = d, a4 asm("a4") = e;
    register long a7 asm("a7") = n;
    asm volatile("ecall" : "+r"(a0) : "r"(a1), "r"(a2), "r"(a3), "r"(a4), "r"(a7) : "memory");
    return a0;
}
static int ret(long r) {
    if (r < 0 && r > -4096) {
        errno = (int)-r;
        return -1;
    }
    return (int)r;
}
extern struct _reent *_impure_ptr;
struct _reent *__getreent(void) { return _impure_ptr; }

int _open(const char *path, int flags, int mode) {
    // newlib's O_ flags are not Linux's: map the ones fopen uses.
    int lf = 0;
    if ((flags & 3) == 1) lf |= 1;
    if ((flags & 3) == 2) lf |= 2;
    if (flags & 0x0008) lf |= 02000;   // O_APPEND
    if (flags & 0x0200) lf |= 0100;    // O_CREAT
    if (flags & 0x0400) lf |= 01000;   // O_TRUNC
    if (flags & 0x0800) lf |= 0200;    // O_EXCL
    return ret(sys(56, -100, (long)path, lf, mode, 0));  // openat(AT_FDCWD, ...)
}
int _close(int fd) { return ret(sys(57, fd, 0, 0, 0, 0)); }
int _read(int fd, void *buf, size_t n) { return ret(sys(63, fd, (long)buf, (long)n, 0, 0)); }
int _write(int fd, const void *buf, size_t n) { return ret(sys(64, fd, (long)buf, (long)n, 0, 0)); }
off_t _lseek(int fd, off_t off, int whence) {
    long long result = 0;
    long long o = off;
    int r = ret(sys(62, fd, (long)(o >> 32), (long)(o & 0xFFFFFFFF), (long)&result, whence));  // llseek
    return r < 0 ? (off_t)-1 : (off_t)result;
}
int _fstat(int fd, struct stat *st) {
    memset(st, 0, sizeof *st);
    st->st_mode = fd < 3 ? S_IFCHR : S_IFREG;
    st->st_blksize = 65536;
    return 0;
}
int _isatty(int fd) { return fd < 3; }
int _stat(const char *path, struct stat *st) {
    int fd = _open(path, 0, 0);
    if (fd < 0) return -1;
    _fstat(fd, st);
    _close(fd);
    return 0;
}
int _unlink(const char *path) { return ret(sys(35, -100, (long)path, 0, 0, 0)); }  // unlinkat
int _mkdir(const char *path, int mode) { return ret(sys(34, -100, (long)path, mode, 0, 0)); }  // mkdirat
int mkdir(const char *path, mode_t mode) { return _mkdir(path, mode); }
void *_sbrk(ptrdiff_t incr) {
    static long cur = 0;
    if (cur == 0) cur = sys(214, 0, 0, 0, 0, 0);
    long want = cur + incr, got = sys(214, want, 0, 0, 0, 0);
    if (got < want) {
        errno = ENOMEM;
        return (void *)-1;
    }
    long old = cur;
    cur = want;
    return (void *)old;
}
void _exit(int code) {
    sys(94, code, 0, 0, 0, 0);  // exit_group
    for (;;) {
    }
}
int _kill(int pid, int sig) { return ret(sys(129, pid, sig, 0, 0, 0)); }
int _getpid(void) { return (int)sys(172, 0, 0, 0, 0, 0); }
int _gettimeofday(struct timeval *tv, void *tz) {
    long long ts[2];  // clock_gettime64: 64-bit seconds, then nanoseconds
    sys(403, 0, (long)ts, 0, 0, 0);
    tv->tv_sec = (time_t)ts[0];
    tv->tv_usec = (suseconds_t)(ts[1] / 1000);
    return 0;
}
int clock_gettime(clockid_t id, struct timespec *tp) {
    long long ts[2];
    int r = ret(sys(403, id, (long)ts, 0, 0, 0));
    tp->tv_sec = (time_t)ts[0];
    tp->tv_nsec = (long)ts[1];
    return r;
}
int _link(const char *a, const char *b) { errno = EMLINK; return -1; }
int _fork(void) { errno = EAGAIN; return -1; }
int _wait(int *s) { errno = ECHILD; return -1; }
int _execve(const char *n, char *const a[], char *const e[]) { errno = ENOMEM; return -1; }
int _times(void *b) { return -1; }

// crt0 passes main no environment: Linux leaves it on the stack after argv's
// NULL, so the program's main is renamed (-Dmain=preview_main) and this one
// sets environ first. Statics are made before it runs: a getenv() in one's
// initialiser finds nothing.
extern char **environ;
int _Z12preview_mainiPPc(int argc, char **argv);  // C++'s name for preview_main(int, char **)
int main(int argc, char **argv) {
    environ = argv + argc + 1;
    return _Z12preview_mainiPPc(argc, argv);
}
