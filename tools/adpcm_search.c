// The IMA ADPCM encoder tools/voice_adpcm.py uses: each sample's four bits are
// chosen for the least error over that sample and the next two - every pair
// of codes for the next, the one after that the nearest - rather than for the
// nearest to it alone, as the standard encoder does. The decoder is the
// standard one, unchanged (tools/ima_adpcm.py, src/sound.cpp). On the voices'
// squeezed words that is 26dB of signal to the noise it adds, against 20.
//
//   cc -O2 -o adpcm_search tools/adpcm_search.c
//   adpcm_search < clips > codes
//
// In: clips one after another, each a little-endian uint32 count and then
// that many little-endian int16 samples. Out: each clip's codes, two to a
// byte, the first in the low nibble, starting afresh - a predictor of 0, a
// step index of 0 - at each clip.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static const int STEP[89] = {
    7,     8,     9,     10,    11,    12,    13,    14,    16,    17,    19,    21,    23,    25,    28,
    31,    34,    37,    41,    45,    50,    55,    60,    66,    73,    80,    88,    97,    107,   118,
    130,   143,   157,   173,   190,   209,   230,   253,   279,   307,   337,   371,   408,   449,   494,
    544,   598,   658,   724,   796,   876,   963,   1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
    2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,  7132,  7845,  8630,
    9493,  10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};
static const int INDEX[8] = {-1, -1, -1, -1, 2, 4, 6, 8};

static void next(int code, int *pred, int *index) {
    int step = STEP[*index], delta = step >> 3;
    if (code & 4) delta += step;
    if (code & 2) delta += step >> 1;
    if (code & 1) delta += step >> 2;
    int p = (code & 8) ? *pred - delta : *pred + delta;
    *pred = p < -32768 ? -32768 : p > 32767 ? 32767 : p;
    int i = *index + INDEX[code & 7];
    *index = i < 0 ? 0 : i > 88 ? 88 : i;
}

// The standard encoder's code: the nearest the next prediction can get.
static int nearest(int s, int pred, int index) {
    int step = STEP[index], diff = s - pred, code = 0;
    if (diff < 0) {
        code = 8;
        diff = -diff;
    }
    if (diff >= step) {
        code |= 4;
        diff -= step;
    }
    if (diff >= step >> 1) {
        code |= 2;
        diff -= step >> 1;
    }
    if (diff >= step >> 2) {
        code |= 1;
    }
    return code;
}

static double sq(int a, int b) { return (double)(a - b) * (a - b); }

static void encodeClip(const int16_t *s, uint32_t n) {
    int pred = 0, index = 0;
    unsigned char byte = 0;
    for (uint32_t i = 0; i < n; i++) {
        int best = 0;
        double bestErr = 1e300;
        for (int c = 0; c < 16; c++) {
            int p = pred, x = index;
            next(c, &p, &x);
            double e = sq(s[i], p);
            if (i + 1 < n) {
                double ahead = 1e300;
                for (int c2 = 0; c2 < 16; c2++) {
                    int p2 = p, x2 = x;
                    next(c2, &p2, &x2);
                    double e2 = sq(s[i + 1], p2);
                    if (i + 2 < n) {
                        next(nearest(s[i + 2], p2, x2), &p2, &x2);
                        e2 += sq(s[i + 2], p2);
                    }
                    if (e2 < ahead) ahead = e2;
                }
                e += ahead;
            }
            if (e < bestErr) {
                bestErr = e;
                best = c;
            }
        }
        next(best, &pred, &index);
        if (i & 1) {
            fputc(byte | best << 4, stdout);
        } else {
            byte = (unsigned char)best;
        }
    }
    if (n & 1) fputc(byte, stdout);
}

int main(void) {
    uint32_t n;
    while (fread(&n, 4, 1, stdin) == 1) {
        int16_t *s = malloc(n * sizeof(int16_t) + 2);
        if (s == NULL || fread(s, 2, n, stdin) != n) return 1;
        encodeClip(s, n);
        free(s);
    }
    return 0;
}
