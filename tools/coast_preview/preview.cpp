// Draws the map outlines (src/coast.cpp) round a point on the PC, exactly as
// the radar draws them on the device - the same code, onto a canvas the same
// size, with the scope's rings over them - and saves it as a PPM, to look at
// what a change to them or to tools/gen_coast.py has done without flashing
// anything. Build and run, from the repo's root:
//
//   g++ -std=c++17 -O2 -Iinclude tools/coast_preview/preview.cpp src/coast.cpp -o /tmp/coast_preview
//   /tmp/coast_preview 55.8719 -4.4331 20 /tmp/glasgow.ppm
//
// The arguments are the latitude and longitude of the middle of the plot, its
// range in nautical miles, and where to save it.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "coast.h"

// The radar's own: its canvas, its plot, and its colours as RGB565.
constexpr int W = 1280, H = 720, CENTER_X = 640, CENTER_Y = 362, RADIUS = 342;
constexpr uint16_t rgb(int r, int g, int b) { return (uint16_t)((r >> 3) << 11 | (g >> 2) << 5 | b >> 3); }
constexpr uint16_t BG = rgb(0x04, 0x12, 0x0A), RING = rgb(0x1C, 0x5A, 0x38), HOME = rgb(0xEA, 0xFF, 0xF0);
constexpr uint16_t COAST = rgb(0x26, 0x50, 0x48);

std::vector<uint16_t> canvas(W * H, BG);

void ring(int r, uint16_t color) {
    for (int i = 0; i < 8 * r; i++) {
        float a = i * (float)M_PI / (4 * r);
        canvas[(CENTER_Y - (int)lroundf(cosf(a) * r)) * W + CENTER_X + (int)lroundf(sinf(a) * r)] = color;
    }
}

int main(int argc, char **argv) {
    if (argc != 5) {
        fprintf(stderr, "usage: %s LAT LON RANGE_NM OUT.ppm\n", argv[0]);
        return 1;
    }
    float range = atof(argv[3]);
    CoastPlot plot = {canvas.data(), W, CENTER_X, CENTER_Y, RADIUS, atof(argv[1]), atof(argv[2]), range, COAST};
    auto t0 = std::chrono::steady_clock::now();
    int strokes = coastDraw(plot);
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
    printf("%d strokes in %ldus\n", strokes, (long)us);

    // Rings as the radar spaces them, the outer one doubled, and the centre's cross.
    int step = range <= 5 ? 1 : range <= 15 ? 5 : range <= 30 ? 10 : range <= 60 ? 20 : 50;
    for (int nm = step; nm < range; nm += step) {
        ring((int)lroundf(nm / range * RADIUS), RING);
    }
    ring(RADIUS, RING);
    ring(RADIUS - 1, RING);
    for (int i = -9; i <= 9; i++) {
        canvas[CENTER_Y * W + CENTER_X + i] = canvas[(CENTER_Y + i) * W + CENTER_X] = HOME;
    }

    FILE *f = fopen(argv[4], "wb");
    if (!f) {
        perror(argv[4]);
        return 1;
    }
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (uint16_t v : canvas) {
        int r = v >> 11, g = (v >> 5) & 0x3F, b = v & 0x1F;
        fputc(r << 3 | r >> 2, f);
        fputc(g << 2 | g >> 4, f);
        fputc(b << 3 | b >> 2, f);
    }
    fclose(f);
    return 0;
}
