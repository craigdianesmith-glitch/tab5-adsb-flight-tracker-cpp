// Flies the 3D landing (src/scene3d.cpp) on the PC and saves its frames as
// PPMs, to see what it looks like without flashing anything - the same code
// the device runs, drawn a band at a time as it would be, with labels on the
// aircraft and the beacon in the device's own font. Build (once the firmware
// has been, for the font it fetches) and run, from the repo's root:
//
//   g++ -std=c++17 -O2 -Iinclude -I.pio/libdeps/tab5/M5GFX/src/lgfx/Fonts tools/landing3d_preview/preview.cpp src/scene3d.cpp src/airfield_pack.cpp src/labels3d.cpp src/coast.cpp src/scenery.cpp src/ground_pack.cpp -lz -o /tmp/landing3d
//   /tmp/landing3d chase /tmp/frames 15
//   ffmpeg -framerate 15 -i /tmp/frames/%04d.ppm -pix_fmt yuv420p /tmp/landing.mp4
//
// The arguments are the view - spotter, chase, tower, orbit (the camera going
// round it, as a drag would), vor, apron or cinema - where to save the
// frames, and how many a second; then, optionally, a time in seconds to save
// just that one frame ("-" for them all), day, dusk or night, and an airport
// by its IATA code with the runway landed on and the one departed from -
// "LHR 27L 27R" - built from the firmware's own runway and navaid tables,
// and its real taxiways, stands and buildings with AIRFIELDS=airfields.bin;
// what covers the land round it, and its hills, with GROUND=ground.bin.
#include <cctype>
#include <chrono>
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "airfield_pack.h"
#include "coast.h"
#include "ground_pc.h"
#include "scene3d.h"
#include "scenery.h"

// The firmware's tables, as it compiles them.
struct RunwayEnd {
    char ident[4];
    float lat, lon;
};
struct Runway {
    char code[4];
    uint16_t widthFt;
    int16_t elevationFt;
    bool closed;
    RunwayEnd end[2];
};
const Runway RUNWAYS[] = {
#include "../../src/runways_table.inc"
};
enum class NavaidType : uint8_t { VOR, VOR_DME, VORTAC, TACAN, DME, NDB_DME };
struct Navaid {
    char ident[5];
    NavaidType type;
    char channel[5];
    uint32_t freqKhz;
    float lat, lon;
    const char *name;
    char airport[5];
};
const Navaid NAVAIDS[] = {
#include "../../src/navaids_table.inc"
};

#define PROGMEM
#include "glcdfont.h"  // Font0's letters, a byte a column
#include "labels3d.h"

using namespace scene3d;

constexpr int W = 1280, H = 720;
// The bands' rows: 16, as the Tab5 has them with room - or BAND_ROWS, 15
// say, as it has them without.
const int BAND = getenv("BAND_ROWS") ? atoi(getenv("BAND_ROWS")) : 16;
std::vector<uint16_t> image(W *H);

void text(const char *s, int x, int y, int size, uint16_t color) {
    for (; *s; s++, x += 6 * size) {
        for (int col = 0; col < 5; col++) {
            uint8_t bits = font[(uint8_t)*s * 5 + col];
            for (int row = 0; row < 8; row++) {
                if (bits >> row & 1) {
                    for (int py = 0; py < size; py++) {
                        for (int px = 0; px < size; px++) {
                            int X = x + col * size + px, Y = y + row * size + py;
                            if (X >= 0 && X < W && Y >= 0 && Y < H) {
                                image[Y * W + X] = color;
                            }
                        }
                    }
                }
            }
        }
    }
}

void lineTo(int x0, int y0, int x1, int y1, uint16_t color) {
    int n = std::max(abs(x1 - x0), abs(y1 - y0));
    for (int i = 0; i <= n; i++) {
        int x = x0 + (x1 - x0) * i / std::max(1, n), y = y0 + (y1 - y0) * i / std::max(1, n);
        if (x >= 0 && x < W && y >= 0 && y < H) {
            image[y * W + x] = color;
        }
    }
}

// A label as the radar has them: a tick from the thing to its words, over
// the picture. Returns where the words are, for other labels to keep off.
labels3d::Rect label(Vec3 at, const char *line1, const char *line2) {
    float x, y;
    if (!projectPoint(at, x, y) || x < -200 || x > W + 200 || y < -100 || y > H + 100) {
        return {0, 0, 0, 0};
    }
    constexpr uint16_t INK = 0xFFFF, GREEN = 0x3F4F;  // white, and the radar's blip green
    int lx = (int)x + 24, ly = (int)y - 60;
    lineTo((int)x, (int)y, lx, ly + 34, GREEN);
    int w = 12 * (int)std::max(strlen(line1), strlen(line2)) + 10;
    text(line1, lx + 5, ly, 2, INK);
    text(line2, lx + 5, ly + 17, 2, GREEN);
    return {lx, ly - 4, lx + w, ly + 34};
}

// The traffic's labels as the 3D view has them: as many, and saying as
// much, as the view has room for, stamped into the picture (labels3d.h).
labels3d::Labeller g_labeller;

void trafficLabels(const labels3d::Candidate *candidates, int count, labels3d::Rect *keepOut, int keepOutCount,
                   float dt) {
    float x, y, reach;
    if (followedOnScreen(x, y, reach)) {
        keepOut[keepOutCount++] = {(int)(x - reach), (int)(y - reach * 0.45f), (int)(x + reach), (int)(y + reach * 0.45f)};
    }
    static labels3d::Placed placed[labels3d::MOST];
    int n = g_labeller.place(candidates, count, keepOut, keepOutCount, W, H, dt, placed, labels3d::MOST);
    labels3d::stamp(image.data(), 0, H, W, placed, n, 0xFFFF, 0x3F4F);  // white, and the radar's blip green
}

// TSTATS=1: each frame's terrain - the march's time, samples, rows filled
// and blocks skipped; the drawing's time, samples and cells found the long
// way - on this PC.
void terrainLine(int frame) {
    if (!getenv("TSTATS")) {
        return;
    }
    uint64_t t[8];
    terrainStats(t, true);
    printf("tstats %04d: march %.0fus %llu samples %llu rows %llu skips; band %.0fus %llu samples %llu long; digest %08x\n",
           frame, t[0] / 1000.0, (unsigned long long)t[1], (unsigned long long)t[2], (unsigned long long)t[3],
           t[4] / 1000.0, (unsigned long long)t[5], (unsigned long long)t[6], (unsigned)terrainDigest());
}

void save(const std::string &path) {
    if (getenv("NOSAVE")) {
        // NOSAVE=1: the frame's FNV-1a in the log instead of its file - to
        // set one build's frames against another's without the gigabytes.
        uint32_t h = 2166136261u;
        for (uint16_t v : image) {
            h = (h ^ (v & 255)) * 16777619u;
            h = (h ^ (v >> 8)) * 16777619u;
        }
        printf("image %s %08x\n", path.substr(path.find_last_of('/') + 1).c_str(), (unsigned)h);
        return;
    }
    FILE *f = fopen(path.c_str(), "wb");
    if (!f) {
        perror(path.c_str());
        exit(1);
    }
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (uint16_t v : image) {
        int r = v >> 11, g = (v >> 5) & 0x3F, b = v & 0x1F;
        fputc(r << 3 | r >> 2, f);
        fputc(g << 2 | g >> 4, f);
        fputc(b << 3 | b >> 2, f);
    }
    fclose(f);
}

// Speed and climb from the path either side of t.
void rates(float t, float &knots, float &fpm) {
    constexpr float DT = 0.1f;
    Pose a = landingPose(t - DT), b = landingPose(t + DT);
    float dx = b.at.x - a.at.x, dy = b.at.y - a.at.y;
    knots = sqrtf(dx * dx + dy * dy) / (2 * DT) * 1.944f;
    fpm = (b.at.z - a.at.z) / (2 * DT) * 196.85f;
}

// A broadcast's lower third: who, what, how fast and high - and once down, how
// hard it touched.
void lowerThird(float t) {
    float knots, fpm;
    rates(t, knots, fpm);
    Pose pose = landingPose(t);
    char line[96];
    float touch = landingTouchdown();
    if (t >= touch && t < touch + 6) {
        float k, landedFpm;
        rates(touch - 0.15f, k, landedFpm);
        snprintf(line, sizeof line, "TOUCHDOWN  %.0f fpm  %.0f kt", -landedFpm, k);
    } else if (pose.at.z > 0) {
        snprintf(line, sizeof line, "%3.0f kt  %4.0f ft  %+5.0f fpm", knots, pose.at.z * 3.28f, fpm);
    } else {
        snprintf(line, sizeof line, "%3.0f kt  on the ground", knots);
    }
    int y = H - 74;
    for (int yy = y; yy < y + 56; yy++) {
        for (int xx = 40; xx < 640; xx++) {
            uint16_t v = image[yy * W + xx];
            image[yy * W + xx] = (v >> 1 & 0x7BEF) >> 1 & 0x7BEF;
        }
    }
    for (int yy = y; yy < y + 56; yy++) {
        for (int xx = 40; xx < 46; xx++) {
            image[yy * W + xx] = 0x07E0 | 0x0010;
        }
    }
    text("BAW27K  A320  RWY 09", 58, y + 8, 2, 0xFFFF);
    text(line, 58, y + 32, 2, 0xC618);
}

// The towns' outlines and the rivers, as view3d loads them, each point put
// by `place`.
struct ScenerySink {
    std::function<Vec3(double, double)> place;
    std::vector<GroundLine> *lines;
    std::vector<RiverLine> rivers;
};

void sinkScenery(void *context, SceneryKind kind, const double *latLon, int n) {
    ScenerySink &s = *(ScenerySink *)context;
    Vec3 a = s.place(latLon[0], latLon[1]);
    for (int i = 0; i + 1 < n; i++) {
        Vec3 b = s.place(latLon[2 * i + 2], latLon[2 * i + 3]);
        if (kind == SceneryKind::TOWN) {
            if (!getenv("NO_TOWNS")) s.lines->push_back({a, b, GroundKind::TOWN});
        } else {
            s.rivers.push_back({a, b, riverWidth(kind)});
        }
        a = b;
    }
}

// The airport's runways in metres round the middle of their ends, its open
// ones only, and its VOR/DMEs within 3km of that; the runway ends named set
// for the landing and the departures. False if it has no runways.
double g_lat = 55.8719, g_lon = -4.4331;  // the made-up airfield's: Glasgow's
float g_fieldM = 8;  // and its height above the sea
int g_landRunway = 0, g_landEnd = 0, g_departRunway = -1, g_departEnd = 0;
std::vector<scene3d::Vec3> g_beaconsAt;
std::vector<std::string> g_beaconNames;
// With STAND naming one of the real layout's stands - "412" - the camera on
// it, from ahead of the stop and to one side, up a little.
bool g_standCam = false;
scene3d::Camera g_standView;

bool useAirport(const std::string &code, const std::string &land, const std::string &depart) {
    std::vector<const Runway *> found;
    for (const Runway &r : RUNWAYS) {
        if (code == r.code && !r.closed) {
            found.push_back(&r);
        }
    }
    if (found.empty()) {
        return false;
    }
    double lat0 = 0, lon0 = 0;
    for (const Runway *r : found) {
        lat0 += (r->end[0].lat + r->end[1].lat) / (2.0 * found.size());
        lon0 += (r->end[0].lon + r->end[1].lon) / (2.0 * found.size());
    }
    g_lat = lat0;
    g_lon = lon0;
    g_fieldM = found[0]->elevationFt * 0.3048f;
    auto local = [&](double lat, double lon) {
        return scene3d::Vec3{(float)((lon - lon0) * 111320.0 * cos(lat0 * M_PI / 180)), (float)((lat - lat0) * 110574.0),
                             0};
    };
    std::vector<RunwaySpec> specs;
    int landRunway = 0, landEnd = 0, departRunway = -1, departEnd = 0;
    for (const Runway *r : found) {
        RunwaySpec spec{};
        for (int e = 0; e < 2; e++) {
            spec.end[e] = local(r->end[e].lat, r->end[e].lon);
            snprintf(spec.name[e], sizeof spec.name[e], "%s", r->end[e].ident);
            if (land == r->end[e].ident) {
                landRunway = (int)specs.size(), landEnd = e;
            }
            if (depart == r->end[e].ident) {
                departRunway = (int)specs.size(), departEnd = e;
            }
        }
        spec.width = r->widthFt * 0.3048f;
        specs.push_back(spec);
        printf("  runway %s/%s, %.0fm by %.0fm\n", spec.name[0], spec.name[1],
               sqrtf(powf(spec.end[1].x - spec.end[0].x, 2) + powf(spec.end[1].y - spec.end[0].y, 2)), spec.width);
    }
    for (const Navaid &n : NAVAIDS) {
        scene3d::Vec3 at = local(n.lat, n.lon);
        if ((n.type == NavaidType::VOR_DME || n.type == NavaidType::VOR || n.type == NavaidType::VORTAC) &&
            sqrtf(at.x * at.x + at.y * at.y) < 3000) {
            g_beaconsAt.push_back(at);
            g_beaconNames.push_back(n.ident);
            printf("  beacon %s at %.0f, %.0f\n", n.ident, at.x, at.y);
        }
    }
    // Its real layout, with AIRFIELDS naming the file tools/gen_airfields.py
    // built - as the Tab5 reads it off its card.
    static airfield::Layout layout;
    bool real = false;
    if (const char *pack = getenv("AIRFIELDS")) {
        FILE *f = fopen(pack, "rb");
        std::vector<uint8_t> file;
        for (int c; f != nullptr && (c = fgetc(f)) != EOF;) {
            file.push_back((uint8_t)c);
        }
        if (f != nullptr) {
            fclose(f);
        }
        if (file.size() >= 16 && memcmp(file.data(), "OVAF", 4) == 0) {
            unsigned count = file[6] | file[7] << 8, index = file[8] | file[9] << 8 | file[10] << 16 | file[11] << 24;
            for (unsigned i = 0; i < count && !real; i++) {
                const uint8_t *row = &file[index + i * 12];
                if (strncmp((const char *)row, code.c_str(), 4) != 0) {
                    continue;
                }
                unsigned at = row[4] | row[5] << 8 | row[6] << 16 | (unsigned)row[7] << 24;
                unsigned size = row[8] | row[9] << 8 | row[10] << 16 | (unsigned)row[11] << 24;
                real = airfield::parse(&file[at], size, layout);
            }
        }
        if (real) {
            scene3d::Vec3 o = local(layout.lat0, layout.lon0);
            layout.moveBy(o.x, o.y);
            for (const airfield::Stand &st : layout.stands) {
                const char *want = getenv("STAND");
                if (want == nullptr || strcmp(st.ref, want) != 0 || st.lead.size() < 2) {
                    continue;
                }
                const airfield::Point &a = st.lead[st.lead.size() - 2], &b = st.lead.back();
                float dx = b.x - a.x, dy = b.y - a.y, d = std::max(0.01f, hypotf(dx, dy));
                dx /= d, dy /= d;
                scene3d::Vec3 stop = {b.x, b.y, 0};
                g_standView = {{stop.x - dx * 75 - dy * 45, stop.y - dy * 75 + dx * 45, 30},
                               {stop.x - dx * 8, stop.y - dy * 8, 4}, 55};
                g_standCam = true;
                printf("  stand %s's stop at %.0f, %.0f\n", st.ref, stop.x, stop.y);
            }
            printf("  its real layout: %zu ways, %zu holds, %zu stands, %zu surfaces, %zu buildings%s\n",
                   layout.ways.size(), layout.holds.size(), layout.stands.size(), layout.surfaces.size(),
                   layout.buildings.size(), layout.hasTower ? ", a tower" : "");
        } else {
            printf("  no real layout for it in %s\n", pack);
        }
    }
    // The coasts and lakes' shores within 60km, laid out flat as the airfield
    // is, as view3d loads them: the sea beside it drawn as water.
    struct Shores {
        decltype(local) *to;
        std::vector<GroundLine> lines;
    } shores = {&local, {}};
    coastLines(lat0, lon0, 60, 60, 0,
               [](void *context, OutlineKind kind, const double *latLon, int n) {
                   Shores &s = *(Shores *)context;
                   if (kind != OutlineKind::COAST && kind != OutlineKind::LAKE) {
                       return;
                   }
                   for (int i = 0; i + 1 < n; i++) {
                       s.lines.push_back({(*s.to)(latLon[2 * i], latLon[2 * i + 1]),
                                          (*s.to)(latLon[2 * i + 2], latLon[2 * i + 3]), (GroundKind)kind});
                   }
               },
               &shores);
    ScenerySink scenery = {local, &shores.lines, {}};
    sceneryLines(lat0, lon0, 60, sinkScenery, &scenery);
    setGroundLines(shores.lines.data(), (int)shores.lines.size());
    setRivers(scenery.rivers.data(), (int)scenery.rivers.size());
    printf("  %zu lines of coast, shore and towns within 60km, %zu of rivers\n", shores.lines.size(),
           scenery.rivers.size());
    auto built0 = std::chrono::steady_clock::now();
    setAirfield(specs.data(), (int)specs.size(), g_beaconsAt.data(), (int)g_beaconsAt.size(), real ? &layout : nullptr);
    auto lit0 = std::chrono::steady_clock::now();
    lightTowns(4000, 120, 6000, false);
    if (getenv("PROFILE")) {
        auto end = std::chrono::steady_clock::now();
        printf("  setAirfield: %.2f ms, then lightTowns %.2f ms\n",
               std::chrono::duration<double, std::milli>(lit0 - built0).count(),
               std::chrono::duration<double, std::milli>(end - lit0).count());
    }
    printf("  the tower's camera %.0fm over its cab\n", towerRaised());
    setLanding(landRunway, landEnd);
    setDeparture(departRunway, departEnd);
    g_landRunway = landRunway, g_landEnd = landEnd, g_departRunway = departRunway, g_departEnd = departEnd;
    return true;
}


// Every family's model, each in two frames: standing on the runway, seen
// from ahead and to its left; and flying, banked into a right turn with its
// controls over, its gear up, seen from behind and to its right - saved as
// fleet/NN-ground.ppm and fleet/NN-air.ppm.
void fleet(const std::string &dir, const std::string &time) {
    static const char *const NAMES[] = {
        "A320  narrowbody",   "KC-135  707",        "E-3 SENTRY",         "787-9  widebody",
        "A340  four engines", "747-400",            "A380",               "CRJ900  regional",
        "G650  business jet", "ATR 72  turboprop",  "SAAB 340",           "CESSNA 172",
        "PIPER CHEROKEE",     "C-17 GLOBEMASTER",   "C-130 HERCULES",     "A400M ATLAS",
        "C295",               "BAE 146",            "TYPHOON",            "F-35",
        "F-16",               "HAWK",               "AW139",              "H145",
        "CHINOOK"};
    static const char *const PAINT[] = {"BAW", "",    "",    "VIR", "DLH", "BAW", "UAE", "DLH", "",
                                        "",    "LOG", "",    "",    "",    "",    "",    "",    "",
                                        "",    "",    "",    "",    "",    "",    ""};
    static const bool MILITARY[] = {false, true,  true,  false, false, false, false, false, false,
                                    false, false, false, false, true,  true,  true,  true,  false,
                                    true,  true,  true,  true,  false, false, true};
    std::vector<uint8_t> scratch(scratchBytes(W, BAND) + 64);
    void *band = (void *)(((uintptr_t)scratch.data() + 63) & ~(uintptr_t)63);
    for (int f = 0; f < (int)Family::COUNT; f++) {
        for (int shot = 0; shot < 2; shot++) {
            Scene scene;
            Look look = lookFor(typeOf((Family)f), 0x1B2A55, MILITARY[f]);
            paintAirline(look, PAINT[f]);
            float size = sizeOf(look);
            Pose pose = {{600, 0, 0}, 0, 0};
            Config config;
            if (shot == 1) {
                pose = {{600, 0, 600}, 0, 3 * 3.14159f / 180, 25 * 3.14159f / 180};
                config.gear = false;
                config.aileron = 0.8f;
                config.elevator = 0.6f;
                config.rudder = 0.5f;
            } else {
                config.flaps = true;
            }
            scene.followed = {pose, look, config};
            scene.seconds = 0.4f;
            scene.landingLights = false;
            scene.sun = sunFor(time == "night" ? TimeOfDay::NIGHT : time == "dusk" ? TimeOfDay::DUSK : TimeOfDay::DAY);
            float reach = std::max(14.0f, 37.6f * size);
            Camera cam = shot == 0 ? orbitCamera(pose, 140, 9, reach * 1.55f, 40)
                                   : orbitCamera(pose, 35, 14, reach * 1.45f, 40);
            prepare(W, H, BAND, false, scene, cam);
            for (int b = 0; b < bandCount(); b++) {
                uint16_t *rows = drawBand(b, band);
                int y = b * BAND;
                memcpy(&image[(size_t)y * W], rows, sizeof(uint16_t) * W * std::min(BAND, H - y));
            }
            text(NAMES[f], 40, 30, 4, 0xFFFF);
            char name[64];
            snprintf(name, sizeof name, "/%02d-%s.ppm", f, shot == 0 ? "ground" : "air");
            save(dir + name);
        }
    }
    printf("models: %u bytes\n", (unsigned)buildModels());
}


// --- high up ---------------------------------------------------------------------

// Where a point on the earth - latitude, longitude, height in metres - is in
// metres east, north and up of another on the ground: through WGS84's
// earth-centred frame, so the earth's curve is in it.
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
    scene3d::Vec3 at(double lat, double lon, double h) const {
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

struct LineSink {
    const Enu *enu;
    std::vector<GroundLine> *out;
    double lat, lon;
};

void sinkLine(void *context, OutlineKind kind, const double *latLon, int n) {
    LineSink &s = *(LineSink *)context;
    for (int i = 0; i + 1 < n; i++) {
        Vec3 a = s.enu->at(latLon[2 * i], latLon[2 * i + 1], 0), b = s.enu->at(latLon[2 * i + 2], latLon[2 * i + 3], 0);
        s.out->push_back({a, b, (GroundKind)kind});
    }
}

// Traffic about a cruise: what it is, its callsign, where it starts against
// the one followed - metres east and north - its height in feet, its course
// and speed, and its climb.
struct Other {
    const char *type, *callsign;
    float east, north, feet, course, knots, fpm;
    bool military;
};

// High up, between airports: an easyJet A320 at FL350 over the Firth of
// Clyde, heading south-west and turning, with traffic about at other heights,
// over the coasts, lakes and borders of the firmware's own map and the
// runways of its own table - the earth curving away to a horizon two hundred
// miles off. Frames from t seconds into it.
void enroute(const std::string &dir, float fps, float from, float to, const std::string &time) {
    const double LAT0 = 55.58, LON0 = -5.25;
    Enu enu(LAT0, LON0);
    std::vector<GroundLine> lines;
    LineSink sink = {&enu, &lines, LAT0, LON0};
    coastLines(LAT0, LON0, 520, 520, 0, sinkLine, &sink);
    ScenerySink scenery = {[&](double la, double lo) { return enu.at(la, lo, 0); }, &lines, {}};
    sceneryLines(LAT0, LON0, 520, sinkScenery, &scenery);
    setGroundLines(lines.data(), (int)lines.size());
    setRivers(scenery.rivers.data(), (int)scenery.rivers.size());
    std::vector<RunwaySpec> runways;
    for (const Runway &r : RUNWAYS) {
        double dn = (r.end[0].lat - LAT0) * 111.0, de = (r.end[0].lon - LON0) * 111.0 * cos(LAT0 * M_PI / 180);
        if (!r.closed && dn * dn + de * de < 400.0 * 400.0) {
            RunwaySpec spec{};
            for (int e = 0; e < 2; e++) {
                spec.end[e] = enu.at(r.end[e].lat, r.end[e].lon, r.elevationFt * 0.3048);
            }
            spec.width = r.widthFt * 0.3048f;
            runways.push_back(spec);
        }
    }
    clearAirfield();
    setFarRunways(runways.data(), (int)runways.size());
    auto lit0 = std::chrono::steady_clock::now();
    lightTowns(6000, 500, 60000, true);
    if (getenv("PROFILE")) {
        printf("  lightTowns %.2f ms\n",
               std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lit0).count());
    }
    printf("  %zu lines on the ground, %zu runways\n", lines.size(), runways.size());

    static const Other OTHERS[] = {
        {"A388", "UAE18", -9000, 14000, 38000, 30, 485, 0, false},
        {"B738", "RYR4LH", 7000, -2000, 21000, 160, 330, 2200, false},
        {"AT76", "LOG6KG", -14000, -9000, 13000, 225, 260, 0, false},
        {"A320", "BAW1497", -5000, -22000, 33000, 205, 440, 0, false},
        {"C17", "RCH457", 26000, -14000, 30000, 95, 420, 0, true},
        {"EUFI", "TARTN1", -11000, -4000, 20000, 175, 480, -1500, true},
        {"A343", "DLH4KN", 16000, -26000, 36000, 300, 470, 0, false},
        {"B789", "VIR35", 30000, 12000, 39000, 285, 490, 0, false},
    };
    constexpr int OTHER_COUNT = sizeof(OTHERS) / sizeof(OTHERS[0]);
    static Aircraft traffic[OTHER_COUNT];

    std::vector<uint8_t> scratch(scratchBytes(W, BAND) + 64);
    void *band = (void *)(((uintptr_t)scratch.data() + 63) & ~(uintptr_t)63);
    Look followedLook = lookFor("A320", 0xFF6600);
    paintAirline(followedLook, "EZY");
    Pilot pilot;
    // The flight: 450 knots at FL350 on 205, turning right from 20s to 50s
    // at a degree a second, onto 235.
    // FEET=: how high it flies; GROUND=ground.bin and TERRAIN=1 the land
    // under it, at its height.
    const float SPEED = 450 * 0.5144f, FEET = getenv("FEET") ? (float)atof(getenv("FEET")) : 35000;
    static PcGround ground;
    bool covered = false;
    if (const char *path = getenv("GROUND")) {
        covered = ground.load(path, LAT0, LON0, 7);
        printf(covered ? "ground: %s round %.4f %.4f\n" : "ground: %s didn't load\n", path, LAT0, LON0);
    }
    auto courseAt = [](float t) { return 205 + std::max(0.0f, std::min(30.0f, t - 20)); };
    // Its track - where it is at t - stepped along from 0.
    auto positionAt = [&](float t, float &e, float &n) {
        e = n = 0;
        for (float u = 0; u < t; u += 0.05f) {
            float c = courseAt(u) * (float)M_PI / 180;
            e += sinf(c) * SPEED * 0.05f;
            n += cosf(c) * SPEED * 0.05f;
        }
    };
    double totalMs = 0;
    int frame = 0;
    for (float t = 0; t <= to + 1e-3f; t += 1 / fps) {
        // The pilot flies the turn whatever frames are saved, so its bank
        // is where it would be.
        float turnRate = (courseAt(t + 0.5f) - courseAt(t - 0.5f)) * -(float)M_PI / 180;  // a heading's way: left +
        pilot.fly(1 / fps, Family::NARROWBODY, turnRate, 450, 0, false);
        if (t < from - 1e-3f) {
            continue;
        }
        float e, n;
        positionAt(t, e, n);
        double lat = LAT0 + n / 111320.0, lon = LON0 + e / (111320.0 * cos(LAT0 * M_PI / 180));
        Vec3 at = enu.at(lat, lon, FEET * 0.3048);
        Pose pose = {at, (90 - courseAt(t)) * (float)M_PI / 180, pilot.pitch, pilot.roll};
        Scene scene;
        Config config;
        config.gear = false;
        config.aileron = pilot.aileron;
        config.elevator = pilot.elevator;
        config.rudder = pilot.rudder;
        scene.followed = {pose, followedLook, config};
        for (int i = 0; i < OTHER_COUNT; i++) {
            const Other &o = OTHERS[i];
            float c = o.course * (float)M_PI / 180, v = o.knots * 0.5144f;
            float oe = o.east + sinf(c) * v * t, on = o.north + cosf(c) * v * t;
            double olat = LAT0 + on / 111320.0, olon = LON0 + oe / (111320.0 * cos(LAT0 * M_PI / 180));
            float feet = o.feet + o.fpm * t / 60;
            Look look = lookFor(o.type, 0x1B2A55, o.military);
            paintAirline(look, o.callsign);
            Config oc;
            oc.gear = false;
            float climb = atanf(o.fpm * 0.00508f / v);
            traffic[i] = {{enu.at(olat, olon, feet * 0.3048f), (90 - o.course) * (float)M_PI / 180, climb + 0.04f}, look, oc};
        }
        scene.traffic = traffic;
        scene.trafficCount = OTHER_COUNT;
        scene.seconds = t;
        scene.earthRadius = 6371000;
        scene.visibility = 140000;
        if (covered) {
            scene.cover = roundCover(ground.window, LAT0, LON0);
            scene.terrain = getenv("TERRAIN") != nullptr;
            // TERRAIN_EFFORT=: the governor's skimping, 0 to 1, as the device's.
            scene.terrainEffort = getenv("TERRAIN_EFFORT") ? (float)atof(getenv("TERRAIN_EFFORT")) : 0;
            scene.terrainGrid = &ground.grid;
        }
        scene.stemsAbove = 0;
        scene.trailsAbove = 8000;
        scene.landAndSea = getenv("NO_LANDSEA") == nullptr;
        scene.landingLights = false;
        if (time == "now" || isdigit((unsigned char)time[0])) {
            int64_t when = (int64_t)::time(nullptr);
            if (time != "now") {
                when = when / 86400 * 86400 + (int64_t)(atof(time.c_str()) * 3600);
            }
            scene.sun = sunAt(when, lat, lon);
        } else {
            scene.sun = sunFor(time == "night" ? TimeOfDay::NIGHT : time == "dusk" ? TimeOfDay::DUSK : TimeOfDay::DAY);
        }
        // The shots: close behind; wide, to see the country and the traffic;
        // and from ahead and below, the trails against the sky.
        float size = sizeOf(followedLook);
        Camera cam;
        const char *shot;
        float phase = fmodf(t, 30);
        if (getenv("TOPDOWN")) {
            shot = "top down";
            cam = {{pose.at.x, pose.at.y - 1, pose.at.z + (float)atof(getenv("TOPDOWN"))}, pose.at, 70};
        } else if (getenv("ORBIT")) {
            // ORBIT="yaw pitch spans fov": one shot held, as a finger would
            // leave the orbit camera.
            float yaw = 0, up = 10, spans = 70, fov = 48;
            sscanf(getenv("ORBIT"), "%f %f %f %f", &yaw, &up, &spans, &fov);
            shot = "orbit";
            cam = orbitCamera(pose, yaw, up, spans * size, fov);
        } else if (phase < 10) {
            shot = "chase";
            cam = orbitCamera(pose, 18 + phase * 2, 9, 70 * size, 48);
        } else if (phase < 20) {
            shot = "wide";
            cam = orbitCamera(pose, 160 - (phase - 10) * 4, 22, 900, 62);
        } else {
            shot = "from below";
            cam = orbitCamera(pose, 200 + (phase - 20) * 3, -18, 120 * size, 52);
        }
        auto start = std::chrono::steady_clock::now();
        prepare(W, H, BAND, false, scene, cam);
        if (getenv("PROFILE")) {  // each step of the preparing, averaged, at the end
            static double sums[STEPS] = {};
            static int frames = 0;
            Counts pc = counts();
            for (int i = 0; i < STEPS; i++) {
                sums[i] += pc.stepUs[i];
            }
            frames++;
            if (t + 1 / fps > to + 1e-3f) {
                printf("preparing, a frame on average over %d (%d triangles last):", frames, pc.triangles);
                for (int i = 0; i < STEPS; i++) {
                    if (sums[i] / frames >= 1) {
                        printf(" %s %.0fus", stepName(i), sums[i] / frames);
                    }
                }
                printf("\n");
            }
        }
        for (int b = 0; b < bandCount(); b++) {
            uint16_t *rows = drawBand(b, band);
            int y = b * BAND;
            memcpy(&image[(size_t)y * W], rows, sizeof(uint16_t) * W * std::min(BAND, H - y));
        }
        totalMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        terrainLine(frame);
        char line[96];
        snprintf(line, sizeof line, "EZY64RT  A320  FL350  450kt  %03.0f  bank %2.0f", courseAt(t),
                 fabsf(pilot.roll) * 180 / (float)M_PI);
        // The traffic's labels: callsign, and height as a flight level with
        // an arrow for climbing or descending, and type.
        Label anchors[16];
        int count = labelAnchors(anchors, 16);
        labels3d::Candidate candidates[16];
        char heights[16][16];
        for (int i = 0; i < count; i++) {
            const Other &o = OTHERS[anchors[i].index];
            snprintf(heights[i], sizeof heights[i], "FL%03d%s", (int)lroundf((o.feet + o.fpm * t / 60) / 100),
                     o.fpm > 300 ? "^" : o.fpm < -300 ? "v" : "");
            candidates[i] = {anchors[i].x, anchors[i].y, anchors[i].sizePx, (uint32_t)anchors[i].index + 1,
                             labels3d::Kind::AIRBORNE, o.callsign, heights[i], o.type};
        }
        labels3d::Rect keep[3] = {{34, H - 44, 40 + 12 * (int)strlen(line) + 6, H - 20},
                                  {34, 26, 40 + 12 * (int)strlen(shot) + 6, 50}};
        trafficLabels(candidates, count, keep, 2, 1 / fps);
        text(line, 40, H - 40, 2, 0xFFFF);
        text(shot, 40, 30, 2, 0xFFFF);
        char name[32];
        snprintf(name, sizeof name, "/%04d.ppm", frame++);
        save(dir + name);
    }
    Counts c = counts();
    printf("%d frames, %.2f ms a frame on this PC; last: %d triangles, %d lines, %d lights, aircraft %d full %d "
           "simple %d tiny %d dots\n",
           frame, totalMs / std::max(1, frame), c.triangles, c.lines, c.lights, c.aircraftFull, c.aircraftSimple,
           c.aircraftTiny, c.aircraftDots);
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s spotter|chase|tower|orbit|vor|apron|cinema outdir fps [seconds] [day|dusk|night]\n",
                argv[0]);
        return 1;
    }
    std::string view = argv[1], dir = argv[2];
    setClock([]() {
        return (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    });
    if (view == "fleet") {
        fleet(dir, argc > 5 ? argv[5] : "day");
        return 0;
    }
    if (view == "enroute") {
        float fps = (float)atof(argv[3]);
        float at = argc > 4 && strcmp(argv[4], "-") != 0 ? (float)atof(argv[4]) : -1;
        enroute(dir, fps, at < 0 ? 0 : at, at < 0 ? 60 : at, argc > 5 ? argv[5] : "day");
        return 0;
    }
    float fps = (float)atof(argv[3]);
    setClock([]() {
        return (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    });
    std::vector<uint8_t> scratch(scratchBytes(W, BAND) + 64);
    void *band = (void *)(((uintptr_t)scratch.data() + 63) & ~(uintptr_t)63);

    float from = 0, to = landingSeconds();
    bool departureView = std::string(argv[1]) == "departure";
    if (argc > 4 && strcmp(argv[4], "-") != 0) {
        from = to = (float)atof(argv[4]);
    }
    std::string time = argc > 5 ? argv[5] : "day";
    bool busy = argc > 6;
    if (busy && !useAirport(argv[6], argc > 7 ? argv[7] : "", argc > 8 ? argv[8] : "")) {
        fprintf(stderr, "no runways at %s\n", argv[6]);
        return 1;
    }
    if (argc <= 4 || strcmp(argv[4], "-") == 0) {
        to = departureView ? 95 : landingSeconds();
    }
    // The ground pack round the airport, all of it read at once.
    static PcGround ground;
    bool covered = false;
    if (const char *path = getenv("GROUND")) {
        covered = ground.load(path, g_lat, g_lon, 7);
        printf(covered ? "ground: %s round %.4f %.4f\n" : "ground: %s didn't load\n", path, g_lat, g_lon);
    }
    static Aircraft parked[400];
    int stands = parkedAtStands(parked, 400), parkedCount = parkedAtStands(parked, 22);
    // The one followed: a BA-blue A320 - or, with a type after the runways,
    // that type, "B77W".
    Look followedLook = lookFor(argc > 9 ? argv[9] : "A320", 0x1B2A55);

    int frame = 0, mostTriangles = 0, mostLines = 0;
    double totalMs = 0;
    for (float t = from; t <= to + 1e-3f; t += 1 / fps, frame++) {
        Scene scene;
        scene.followed = {landingPose(t), followedLook, {}};
        // The director's views fly the landing - or a departure, lined up for
        // twenty seconds first - as though it were live: the pose, and its
        // speed and climb from either side of it.
        static Director director;
        Flight flight;
        bool departing = view == "departure";
        auto poseAt = [&](float when) { return departing ? departurePose(when - 20) : landingPose(when); };
        // Set up as what it is doing calls for: gear, flaps, spoilers.
        {
            Pose a = poseAt(t - 0.1f), b = poseAt(t + 0.1f), now = poseAt(t);
            float knots = sqrtf(powf(b.at.x - a.at.x, 2) + powf(b.at.y - a.at.y, 2)) / 0.2f * 1.944f;
            bool ground = now.at.z <= 0.01f && (departing || t >= landingTouchdown() - 0.05f);
            scene.followed.config = configFor(now.at.z, knots, ground, departing);
            if (!departing) {
                // The spoilers springing up from the touchdown, as the view's.
                scene.followed.config.spoilerDeploy *= spoilersRising(t - landingTouchdown());
                scene.followed.config.spoilers = scene.followed.config.spoilerDeploy > 0.01f;
            }
        }
        if (view == "director" || departing) {
            Pose a = poseAt(t - 0.1f), b = poseAt(t + 0.1f), now = poseAt(t);
            scene.followed.pose = now;
            flight.aircraft = scene.followed;
            flight.speedKt = sqrtf(powf(b.at.x - a.at.x, 2) + powf(b.at.y - a.at.y, 2)) / 0.2f * 1.944f;
            flight.climbFpm = (b.at.z - a.at.z) / 0.2f * 196.85f;
            flight.onGround = now.at.z <= 0.01f && (departing || t >= landingTouchdown() - 0.05f);
            flight.departing = departing;
            flight.runway = departing ? g_departRunway : g_landRunway;
            flight.end = departing ? g_departEnd : g_landEnd;
        }
        if (busy) {
            // The made-up traffic's own departures fly the runway the one
            // followed is leaving by: none of them in the departure's view.
            if (view == "departure") {
                setDeparture(-1, 0);
            }
            parkedCount = busyTraffic(t, parked, 400);
            setDeparture(g_departRunway, g_departEnd);
        }
        scene.traffic = parked;
        scene.trafficCount = parkedCount;
        scene.seconds = t;
        scene.landAndSea = getenv("NO_LANDSEA") == nullptr;  // where there are shores, as view3d
        if (covered) {
            scene.cover = flatCover(ground.window, g_lat, g_lon);
            // TERRAIN=1: the land at its height.
            scene.terrain = getenv("TERRAIN") != nullptr;
            // TERRAIN_EFFORT=: the governor's skimping, 0 to 1, as the device's.
            scene.terrainEffort = getenv("TERRAIN_EFFORT") ? (float)atof(getenv("TERRAIN_EFFORT")) : 0;
            scene.groundUpM = g_fieldM;
            scene.terrainGrid = &ground.grid;
        }
        // "now" is the real sun over the airport at this moment; a number, the
        // sun that many hours after midnight UTC today.
        if (time == "now" || isdigit((unsigned char)time[0])) {
            int64_t when = (int64_t)::time(nullptr);
            if (time != "now") {
                when = when / 86400 * 86400 + (int64_t)(atof(time.c_str()) * 3600);
            }
            scene.sun = sunAt(when, g_lat, g_lon);
        } else {
            scene.sun = sunFor(time == "night" ? TimeOfDay::NIGHT : time == "dusk" ? TimeOfDay::DUSK : TimeOfDay::DAY);
        }
        float touch = landingTouchdown();
        if (t >= touch) {
            scene.smokeAge = t - touch;
            scene.touchdown = landingPose(touch);
        }
        const Pose &pose = scene.followed.pose;
        float size = sizeOf(followedLook);
        const char *shot = nullptr;
        Camera cam;
        if (view == "spotter") {
            cam = fixedCamera(pose, {560, -120, 1.8f}, 0.4f, size);
        } else if (view == "tower") {
            Vec3 eye = {1850, 292, 33};
            towerEye(eye);
            cam = fixedCamera(pose, eye, 0.3f, size);
        } else if (view == "overview") {
            cam = {{-2600, -3400, 2400}, {0, 0, 0}, 58};
        } else if (view == "vor") {
            cam = {{470, -300, 14}, {420, -360, 4}, 60};
        } else if (view == "apron") {
            cam = orbitCamera(pose, 15, 10, 90, 55);
        } else if (view == "orbit") {
            cam = orbitCamera(pose, 160 + t * 6, 10 + 8 * sinf(t / 5), 75, 50);
        } else if (view == "cinema") {
            cam = cinemaCamera(t, &shot, size);
        } else if (view == "director" || view == "departure") {
            Pose shown;
            bool replaying = false;
            cam = director.next(flight, t, &shot, &shown, &replaying);
            if (replaying) {
                scene.followed.pose = shown;
            }
        } else {
            cam = orbitCamera(pose, 20, 12, 80, 50);
        }
        auto start = std::chrono::steady_clock::now();
        if (g_standCam) {
            cam = g_standView;
        }
        if (getenv("OVERHEAD")) {  // straight down on the airfield's middle from OVERHEAD metres
            cam = {{0, -1, (float)atof(getenv("OVERHEAD"))}, {0, 0, 0}, 70};
        }
        prepare(W, H, BAND, false, scene, cam);
        if (getenv("PROFILE")) {  // each step of the preparing, averaged, at the end
            static double sums[STEPS] = {};
            static int frames = 0;
            Counts pc = counts();
            for (int i = 0; i < STEPS; i++) {
                sums[i] += pc.stepUs[i];
            }
            frames++;
            if (t + 1 / fps > to + 1e-3f) {
                printf("preparing, a frame on average over %d (%d triangles last):", frames, pc.triangles);
                for (int i = 0; i < STEPS; i++) {
                    if (sums[i] / frames >= 1) {
                        printf(" %s %.0fus", stepName(i), sums[i] / frames);
                    }
                }
                printf("\n");
            }
        }
        for (int b = 0; b < bandCount(); b++) {
            uint16_t *rows = drawBand(b, band);
            int y = b * BAND;
            memcpy(&image[(size_t)y * W], rows, sizeof(uint16_t) * W * std::min(BAND, H - y));
        }
        totalMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        terrainLine(frame);

        char height[32];
        snprintf(height, sizeof height, "%.0fft", pose.at.z * 3.28f);
        std::vector<labels3d::Rect> keep;
        keep.push_back(label({pose.at.x, pose.at.y, pose.at.z + 12}, "BAW27K", height));
        if (!busy) {
            keep.push_back(label({420, -360, 13}, "GOW", "115.40"));
        }
        for (size_t i = 0; i < g_beaconsAt.size(); i++) {
            keep.push_back(label({g_beaconsAt[i].x, g_beaconsAt[i].y, 13}, g_beaconNames[i].c_str(), "VOR/DME"));
        }
        keep.push_back({40, H - 74, 640, H - 18});  // the lower third
        if (shot) {
            keep.push_back({34, 26, 40 + 12 * (int)strlen(shot) + 6, 50});
        }
        // The traffic's labels: made-up callsigns, and the parked told apart
        // from the rest by being on the stands - busyTraffic()'s first.
        {
            static const char *const AIRLINES[] = {"BAW", "EZY", "RYR", "DLH", "AFR", "KLM",
                                                   "UAE", "QTR", "AAL", "UAL", "EIN", "VIR"};
            Label anchors[64];
            int n = labelAnchors(anchors, 64);
            labels3d::Candidate candidates[64];
            char calls[64][10], heights[64][16];
            for (int i = 0; i < n; i++) {
                int k = anchors[i].index;
                const Aircraft &a = parked[k];
                snprintf(calls[i], sizeof calls[i], "%s%d", AIRLINES[k % 12], 100 + k * 37 % 900);
                bool flying = a.pose.at.z > 1;
                if (flying) {
                    snprintf(heights[i], sizeof heights[i], "%dft", (int)lroundf(a.pose.at.z * 3.28f / 10) * 10);
                } else {
                    snprintf(heights[i], sizeof heights[i], "GND");
                }
                labels3d::Kind kind = k < stands ? labels3d::Kind::PARKED
                                      : flying   ? labels3d::Kind::AIRBORNE
                                                 : labels3d::Kind::MOVING;
                candidates[i] = {anchors[i].x, anchors[i].y, anchors[i].sizePx, (uint32_t)k + 1, kind, calls[i],
                                 heights[i], typeOf(a.look.family)};
            }
            keep.resize(keep.size() + 1);  // room for the one followed
            trafficLabels(candidates, n, keep.data(), (int)keep.size() - 1, 1 / fps);
        }
        lowerThird(t);
        if (shot) {
            text(shot, 40, 30, 2, 0xFFFF);
        }
        char name[16];
        snprintf(name, sizeof name, "/%04d.ppm", frame);
        save(dir + name);
        mostTriangles = std::max(mostTriangles, counts().triangles);
        mostLines = std::max(mostLines, counts().lines);
    }
    Counts c = counts();
    printf("%d frames, %.2f ms a frame on this PC; last: %d triangles, %d lines, %d lights, aircraft %d full %d simple "
           "%d tiny %d dots, of %d; most: %d triangles, %d lines\n",
           frame, totalMs / frame, c.triangles, c.lines, c.lights, c.aircraftFull, c.aircraftSimple, c.aircraftTiny,
           c.aircraftDots, parkedCount + 1, mostTriangles, mostLines);
    return 0;
}
