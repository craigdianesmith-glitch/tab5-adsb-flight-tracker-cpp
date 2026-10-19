#include "geoid.h"

#include <math.h>
#include <stdint.h>

namespace {
#include "geoid_table.inc"
}  // namespace

float geoidM(double lat, double lon) {
    double y = fmin(180.0, fmax(0.0, lat + 90.0)), x = fmod(fmod(lon + 180.0, 360.0) + 360.0, 360.0);
    int j = (int)fmin(179.0, floor(y)), i = (int)fmin(359.0, floor(x));
    float fy = (float)(y - j), fx = (float)(x - i);
    float a = GEOID_M[j][i] + (GEOID_M[j][i + 1] - GEOID_M[j][i]) * fx;
    float b = GEOID_M[j + 1][i] + (GEOID_M[j + 1][i + 1] - GEOID_M[j + 1][i]) * fx;
    return a + (b - a) * fy;
}
