// Checks the 3D view's land and sea and towns high up on the PC -
// src/scene3d.cpp's groundAndCoasts(), which tells them apart a row of the
// screen at a time by which way each coast and town's outline crosses it -
// pixel by pixel, against maps of where the land is and where the towns are. The aircraft cruises at a place, with the country's coasts
// loaded about it as view3d loads them, and the ORBIT camera is dragged
// round it as a finger would: right round at each of several heights, up and
// down at each of several bearings, at three zooms - a frame each step, so
// what the view keeps from frame to frame is kept here too.
//
// The maps: rasters 400m a cell, 800km about the place, each filled a row at
// a time - the land's from the coasts and lakes' shores loaded in full detail
// to 1,700km, further than the view loads them, so no row of it runs off the
// end of them; the towns' from their outlines, as the view has them. A pixel is judged where its ray meets the round earth: not within
// 1.2km of a coast (4.5km where the view draws them thinned), and not past
// 225km off, where the haze has taken four fifths of the colour and land
// and sea look alike - and where, at so low an angle, the map's sphere and
// the view's ellipsoid put a coast kilometres apart.
//
// Build and run, from the repo's root:
//
//   g++ -std=c++17 -O2 -Iinclude tools/landsea_check/check.cpp src/coast.cpp src/scenery.cpp src/airfield_pack.cpp
//       -o /tmp/landsea_check
//   /tmp/landsea_check all
//   /tmp/landsea_check Clyde Finland
//   /tmp/landsea_check 55.58 -5.25 0 0 205 36100 /tmp/frames
//
// (the first all one line): every place in PLACES below, some of them, or
// one given as its latitude and longitude, the aircraft that many km east
// and north of it, its course and its height in feet - and a folder to save
// the frames with anything wrong in as PPMs, the wrong pixels red where the
// view drew land or a town on the sea and yellow where it drew anything else
// wrong. NO_TOWNS= leaves the towns out. SHOT=
// "azimuth,elevation,zoom" checks that one frame; LEVEL= and FULL_KM= load
// the coasts as thinned as coastLines() can (2, 90 was the view's before
// 19 Oct) to see what that does.
#include "../../src/scene3d.cpp"  // for the frame drawn, `d`

#include <cctype>
#include <chrono>
#include <string>
#include <strings.h>

#include "coast.h"
#include "scenery.h"

using namespace scene3d;

namespace {

// Metres east, north and up through WGS84's earth-centred frame, as the
// view's own Enu.
struct Enu {
    double lat0, lon0, x0, y0, z0;
    static void ecef(double lat, double lon, double h, double &x, double &y, double &z) {
        constexpr double A = 6378137.0, E2 = 6.69437999014e-3;
        double p = lat * M_PI / 180, l = lon * M_PI / 180, n = A / sqrt(1 - E2 * sin(p) * sin(p));
        x = (n + h) * cos(p) * cos(l);
        y = (n + h) * cos(p) * sin(l);
        z = (n * (1 - E2) + h) * sin(p);
    }
    Enu(double lat, double lon) : lat0(lat), lon0(lon) { ecef(lat, lon, 0, x0, y0, z0); }
    Vec3 at(double lat, double lon, double h) const {
        double x, y, z;
        ecef(lat, lon, h, x, y, z);
        double dx = x - x0, dy = y - y0, dz = z - z0;
        double p = lat0 * M_PI / 180, l = lon0 * M_PI / 180;
        double e = -sin(l) * dx + cos(l) * dy;
        double n = -sin(p) * cos(l) * dx - sin(p) * sin(l) * dy + cos(p) * dz;
        double u = cos(p) * cos(l) * dx + cos(p) * sin(l) * dy + sin(p) * dz;
        return {(float)e, (float)n, (float)u};
    }
};

struct Sink {
    const Enu *enu;
    std::vector<GroundLine> *out;
};

void sinkTown(void *context, SceneryKind kind, const double *latLon, int points) {
    if (kind != SceneryKind::TOWN) {
        return;
    }
    Sink &s = *(Sink *)context;
    Vec3 a = s.enu->at(latLon[0], latLon[1], 0);
    for (int i = 0; i + 1 < points; i++) {
        Vec3 b = s.enu->at(latLon[2 * i + 2], latLon[2 * i + 3], 0);
        s.out->push_back({a, b, GroundKind::TOWN});
        a = b;
    }
}

void sinkLine(void *context, OutlineKind kind, const double *latLon, int points) {
    Sink &s = *(Sink *)context;
    Vec3 a = s.enu->at(latLon[0], latLon[1], 0);
    for (int i = 0; i + 1 < points; i++) {
        Vec3 b = s.enu->at(latLon[2 * i + 2], latLon[2 * i + 3], 0);
        s.out->push_back({a, b, (GroundKind)kind});
        a = b;
    }
}

// Where the land is: each cell land or sea, and whether it is too near a
// coast to judge.
struct Map {
    double half, cell;
    int n;
    std::vector<uint8_t> land, near;

    // 0 sea, 1 land, -1 not to be judged.
    int at(double x, double y) const {
        int i = (int)floor((x + half) / cell), j = (int)floor((y + half) / cell);
        if (i < 0 || j < 0 || i >= n || j >= n || near[(size_t)j * n + i]) {
            return -1;
        }
        return land[(size_t)j * n + i];
    }
};

// Each row of cells filled from the crossings of it, as the view fills a
// row of the screen: before the first, what that one leaves; after each,
// what it leads onto - going east, a coast going south leads onto land.
void buildMap(Map &map, const std::vector<GroundLine> &lines, double half, double cell, double fullKm, bool towns) {
    map.half = half;
    map.cell = cell;
    map.n = (int)(2 * half / cell);
    int n = map.n;
    map.land.assign((size_t)n * n, 0);
    map.near.assign((size_t)n * n, 0);
    std::vector<std::vector<std::pair<float, bool>>> rows(n);
    for (const GroundLine &l : lines) {
        if (towns ? l.kind != GroundKind::TOWN : l.kind != GroundKind::COAST && l.kind != GroundKind::LAKE) {
            continue;
        }
        double ay = l.a.y, by = l.b.y;
        if (ay != by) {
            double lo = std::min(ay, by), hi = std::max(ay, by);
            int j0 = std::max(0, (int)ceil((lo + half) / cell - 0.5)),
                j1 = std::min(n, (int)ceil((hi + half) / cell - 0.5));
            for (int j = j0; j < j1; j++) {
                double y = -half + (j + 0.5) * cell;
                rows[j].push_back({(float)(l.a.x + (y - ay) * (l.b.x - l.a.x) / (by - ay)), by < ay});
            }
        }
        // The cells it runs through, for how far each cell is from a coast.
        double len = sqrt(pow(l.b.x - l.a.x, 2) + pow(l.b.y - l.a.y, 2));
        int steps = (int)(len / (cell / 2)) + 1;
        for (int k = 0; k <= steps; k++) {
            double x = l.a.x + (l.b.x - l.a.x) * k / steps, y = l.a.y + (l.b.y - l.a.y) * k / steps;
            int i = (int)floor((x + half) / cell), j = (int)floor((y + half) / cell);
            if (i >= 0 && j >= 0 && i < n && j < n) {
                map.near[(size_t)j * n + i] = 1;
            }
        }
    }
    for (int j = 0; j < n; j++) {
        auto &r = rows[j];
        if (r.empty()) {
            continue;  // no coast anywhere along it: sea, as the map starts
        }
        std::sort(r.begin(), r.end());
        bool land = !r[0].second;
        size_t k = 0;
        for (int i = 0; i < n; i++) {
            double x = -half + (i + 0.5) * cell;
            for (; k < r.size() && r[k].first < x; k++) {
                land = r[k].second;
            }
            map.land[(size_t)j * n + i] = land;
        }
    }
    // How far each cell is from a coast, in thirds of a cell (a 3-4
    // chamfer, two passes), and so whether it is too near to judge.
    std::vector<uint16_t> far((size_t)n * n);
    for (size_t k = 0; k < far.size(); k++) {
        far[k] = map.near[k] ? 0 : 60000;
    }
    auto relax = [&](int i, int j, int di, int dj, int w) {
        int ii = i + di, jj = j + dj;
        if (ii >= 0 && jj >= 0 && ii < n && jj < n) {
            uint16_t &d = far[(size_t)j * n + i];
            d = (uint16_t)std::min<int>(d, far[(size_t)jj * n + ii] + w);
        }
    };
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            relax(i, j, -1, 0, 3), relax(i, j, 0, -1, 3), relax(i, j, -1, -1, 4), relax(i, j, 1, -1, 4);
        }
    }
    for (int j = n - 1; j >= 0; j--) {
        for (int i = n - 1; i >= 0; i--) {
            relax(i, j, 1, 0, 3), relax(i, j, 0, 1, 3), relax(i, j, 1, 1, 4), relax(i, j, -1, 1, 4);
        }
    }
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            double x = -half + (i + 0.5) * cell, y = -half + (j + 0.5) * cell;
            double within = towns || sqrt(x * x + y * y) < fullKm * 1000 - 5000 ? 1200 : 4500;
            map.near[(size_t)j * n + i] = far[(size_t)j * n + i] / 3.0 * cell < within;
        }
    }
}

struct Place {
    const char *name;
    double lat, lon;
    float eastKm, northKm, course, feet;
};

// Coasts of every kind - fjords, skerries, lochs and isles, lakes by the
// thousand, archipelagos, open ocean with an island or two - each with the
// aircraft over the place and off to one side of it.
const Place PLACES[] = {
    {"Clyde", 55.58, -5.25, 0, 0, 205, 36100},       {"Clyde", 55.58, -5.25, 60, -100, 30, 36100},
    {"Bergen", 60.4, 5.0, 0, 0, 205, 36100},         {"Bergen", 60.4, 5.0, -140, 40, 300, 38000},
    {"Aegean", 37.5, 25.0, 60, -100, 205, 36100},    {"Aegean", 37.5, 25.0, -80, 50, 120, 30000},
    {"Denmark", 55.5, 11.0, 0, 0, 205, 36100},       {"Denmark", 55.5, 11.0, -140, 40, 60, 34000},
    {"Finland", 62.0, 26.0, 60, -100, 205, 36100},   {"Finland", 62.0, 26.0, -80, 50, 120, 36100},
    {"Hebrides", 57.3, -6.5, 60, -100, 205, 36100},  {"Hebrides", 57.3, -6.5, -80, 50, 120, 25000},
    {"Biscay", 45.5, -4.0, 60, -100, 205, 36100},    {"Canaries", 28.3, -15.5, 60, -100, 205, 36100},
    {"Tromso", 69.5, 18.5, -80, 50, 120, 36100},     {"Tokyo", 35.5, 139.8, -80, 50, 120, 38000},
    {"Caribbean", 18.3, -64.8, -80, 50, 120, 33000}, {"Chesapeake", 38.5, -76.3, -80, 50, 120, 24000},
    {"Indonesia", -6.0, 106.0, -80, 50, 120, 36000}, {"Iceland", 64.1, -21.9, -80, 50, 120, 35000},
    {"Rotterdam", 51.9, 4.4, -80, 50, 120, 20000},   {"Vanern", 58.9, 13.3, -80, 50, 120, 36000},
    {"Ladoga", 61.0, 31.5, -80, 50, 120, 36000},     {"Greece", 39.0, 23.0, 100, 20, 250, 37000},
    {"Philippines", 11.0, 123.0, -60, -40, 10, 39000},
    // Where Natural Earth's lakes overlapped - Saginaw Bay a lake of its own
    // inside Huron's - before tools/gen_coast.py joined them.
    {"Huron", 45.0, -83.0, -80, 50, 120, 41000},      {"Erie", 42.2, -81.0, -80, 50, 120, 36000},
};

constexpr int VIEW_W = 1008, VIEW_H = 560, BAND = 16;  // as view3d's

struct Shot {
    float azimuth, elevation, zoom;
};

void savePpm(const std::string &path, const std::vector<uint16_t> &image, const std::vector<uint8_t> &wrong) {
    FILE *f = fopen(path.c_str(), "wb");
    if (f == nullptr) {
        return;
    }
    fprintf(f, "P6\n%d %d\n255\n", VIEW_W, VIEW_H);
    for (int i = 0; i < VIEW_W * VIEW_H; i++) {
        uint16_t p = image[i];
        uint8_t rgb[3] = {(uint8_t)((p >> 11) << 3), (uint8_t)(((p >> 5) & 63) << 2), (uint8_t)((p & 31) << 3)};
        if (wrong[i] == 1) {
            rgb[0] = 255, rgb[1] = 40, rgb[2] = 40;
        } else if (wrong[i] == 2) {
            rgb[0] = 255, rgb[1] = 200, rgb[2] = 0;
        }
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}

// One place: true if no frame had more than 200 pixels wrong.
bool check(const Place &place, const std::string &out) {
    Enu enu(place.lat, place.lon);
    float fullKm = getenv("FULL_KM") ? (float)atof(getenv("FULL_KM")) : 520;
    int level = getenv("LEVEL") ? atoi(getenv("LEVEL")) : 0;
    std::vector<GroundLine> lines, wide;
    Sink sink = {&enu, &lines}, wideSink = {&enu, &wide};
    coastLines(place.lat, place.lon, 520, fullKm, level, sinkLine, &sink);
    if (getenv("NO_TOWNS") == nullptr) {
        sceneryLines(place.lat, place.lon, 520, sinkTown, &sink);
    }
    setGroundLines(lines.data(), (int)lines.size());
    coastLines(place.lat, place.lon, 1700, 1700, 0, sinkLine, &wideSink);
    Map map, towns;
    buildMap(map, wide, 800000, 400, fullKm, false);
    buildMap(towns, lines, 800000, 400, fullKm, true);

    double lat = place.lat + place.northKm / 111.32,
           lon = place.lon + place.eastKm / (111.32 * cos(place.lat * M_PI / 180));
    Pose pose = {enu.at(lat, lon, place.feet * 0.3048), (90 - place.course) * DEG, 0, 0};
    Look look = lookFor("A21N", 0x1B2A55);
    float size = sizeOf(look);
    Scene scene;
    scene.followed = {pose, look, {}};
    scene.followed.config.gear = false;
    scene.earthRadius = 6371000;  // as view3d high up
    scene.visibility = 140000;
    scene.stemsAbove = 0;
    scene.trailsAbove = 8000;
    scene.landAndSea = true;
    scene.landingLights = false;
    scene.sun = sunFor(TimeOfDay::DAY);

    std::vector<Shot> shots;
    if (const char *one = getenv("SHOT")) {
        Shot s{0, 0, 1};
        sscanf(one, "%f,%f,%f", &s.azimuth, &s.elevation, &s.zoom);
        shots.push_back(s);
    } else {
        for (float zoom : {1.0f, 0.3f, 4.0f}) {
            for (float el : {-10.0f, 0.0f, 12.0f, 30.0f, 50.0f, 80.0f}) {
                for (float az = 0; az < 360; az += 3) {
                    shots.push_back({az, el, zoom});
                }
            }
            for (float az = 0; az < 360; az += 45) {
                for (float el = -10; el <= 80; el += 2) {
                    shots.push_back({az, el, zoom});
                }
                for (float el = 80; el >= -10; el -= 2) {
                    shots.push_back({az, el, zoom});
                }
            }
        }
    }
    std::vector<uint8_t> scratch(scratchBytes(VIEW_W, BAND) + 64);
    void *band = (void *)(((uintptr_t)scratch.data() + 63) & ~(uintptr_t)63);
    std::vector<uint16_t> image((size_t)VIEW_W * VIEW_H);
    std::vector<uint8_t> wrong((size_t)VIEW_W * VIEW_H);
    long judged = 0, wrongPixels = 0;
    int badFrames = 0, saved = 0;
    double worst = 0, countryUs = 0;
    for (const Shot &s : shots) {
        Camera cam = orbitCamera(pose, s.azimuth, s.elevation, std::max(28 * size, 80 * size / s.zoom), 50);
        prepare(VIEW_W, VIEW_H, BAND, false, scene, cam);
        countryUs += d.counts.stepUs[STEP_COUNTRY];
        if (!d.landSea) {
            continue;
        }
        // Each pixel, every third of every third row: what the frame says,
        // walked along its row as drawBandInto() walks it, and what the map
        // says where its ray meets the earth.
        std::fill(wrong.begin(), wrong.end(), 0);
        long frameJudged = 0, frameWrong = 0;
        const double R = scene.earthRadius;
        for (int y = 0; y < VIEW_H; y += 3) {
            if (!d.rowGround[y]) {
                continue;
            }
            bool on[2] = {(d.rowFirst[y] & 1) != 0, (d.rowFirst[y] & 2) != 0};
            uint32_t i = d.rowStart[y];
            for (int x = 0; x < VIEW_W; x++) {
                for (; i < d.rowStart[y + 1] && d.rowCross[i].x <= x; i++) {
                    on[d.rowCross[i].layer] = d.rowCross[i].on;
                }
                int drawn = !on[0] ? 0 : on[1] ? 2 : 1;  // sea, land, town
                if (x % 3 != 0) {
                    continue;
                }
                Vec3 dir = d.forward + d.right * ((x + 0.5f - d.cx) / d.f) + d.down * ((y + 0.5f - d.cy) / d.f);
                double ex = d.eye.x, ey = d.eye.y, ez = d.eye.z + R;
                double a = (double)dir.x * dir.x + (double)dir.y * dir.y + (double)dir.z * dir.z,
                       b = 2 * (ex * dir.x + ey * dir.y + ez * dir.z), c = ex * ex + ey * ey + ez * ez - R * R;
                double disc = b * b - 4 * a * c;
                if (disc < 0) {
                    continue;
                }
                double t = (-b - sqrt(disc)) / (2 * a);
                if (t <= 0 || t * sqrt(a) > 225000) {
                    continue;
                }
                int truth = map.at(ex + dir.x * t, ey + dir.y * t), town = towns.at(ex + dir.x * t, ey + dir.y * t);
                if (truth < 0 || (truth == 1 && town < 0)) {
                    continue;
                }
                truth = truth == 0 ? 0 : town == 1 ? 2 : 1;
                frameJudged++;
                if (drawn != truth) {
                    frameWrong++;
                    wrong[(size_t)y * VIEW_W + x] = drawn != 0 && truth == 0 ? 1 : 2;
                }
            }
        }
        judged += frameJudged;
        wrongPixels += frameWrong;
        if (frameWrong > 200) {
            double share = (double)frameWrong / frameJudged;
            worst = std::max(worst, share);
            if (badFrames++ < 10) {
                printf("    azimuth %5.1f elevation %5.1f zoom %.1f: %.1f%% wrong\n", s.azimuth, s.elevation, s.zoom,
                       100 * share);
            }
            if (!out.empty() && saved < 20) {
                for (int k = 0; k < bandCount(); k++) {
                    uint16_t *rows = drawBand(k, band);
                    memcpy(&image[(size_t)k * BAND * VIEW_W], rows,
                           sizeof(uint16_t) * VIEW_W * std::min(BAND, VIEW_H - k * BAND));
                }
                char name[200];
                snprintf(name, sizeof name, "%s/%s_%03d_az%.0f_el%.0f_z%.1f.ppm", out.c_str(), place.name, saved++,
                         s.azimuth, s.elevation, s.zoom);
                savePpm(name, image, wrong);
            }
        }
    }
    printf("%-11s %5.0f,%5.0fkm FL%03.0f: %zu frames, %d with over 200 pixels wrong (the worst %.1f%%); %.4f%% of %ld "
           "pixels; %zu lines, the country %.0fus a frame here\n",
           place.name, place.eastKm, place.northKm, place.feet / 100, shots.size(), badFrames, 100 * worst,
           judged > 0 ? 100.0 * wrongPixels / judged : 0.0, judged, lines.size(), countryUs / shots.size());
    return badFrames == 0;
}

}  // namespace

int main(int argc, char **argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    setClock([]() {
        return (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    });
    if (argc < 2) {
        fprintf(stderr, "usage: %s all | PLACE... | lat lon eastKm northKm course feet [folder]\n", argv[0]);
        return 2;
    }
    std::vector<Place> places;
    std::string out;
    if (argc >= 7 && (isdigit((unsigned char)argv[1][0]) || argv[1][0] == '-')) {
        places.push_back({"here", atof(argv[1]), atof(argv[2]), (float)atof(argv[3]), (float)atof(argv[4]),
                          (float)atof(argv[5]), (float)atof(argv[6])});
        out = argc > 7 ? argv[7] : "";
    } else {
        for (int i = 1; i < argc; i++) {
            bool found = false;
            for (const Place &p : PLACES) {
                if (strcmp(argv[i], "all") == 0 || strcasecmp(argv[i], p.name) == 0) {
                    places.push_back(p);
                    found = true;
                }
            }
            if (!found) {
                out = argv[i];  // not a place: the folder for the frames
            }
        }
    }
    int failed = 0;
    for (const Place &p : places) {
        failed += !check(p, out);
    }
    printf("%zu checked, %d with frames wrong\n", places.size(), failed);
    return failed > 0;
}
