#include "airfield_pack.h"

#include <math.h>
#include <string.h>

#include <algorithm>

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <SD.h>

#include "recorder.h"
#endif

namespace airfield {

void Layout::clear() {
    lat0 = lon0 = 0;
    junctions.clear();
    ways.clear();
    holds.clear();
    stands.clear();
    surfaces.clear();
    buildings.clear();
    hasTower = false;
    towerHeight = 0;
    windsocks.clear();
}

void Layout::moveBy(float dx, float dy) {
    auto move = [&](Point &p) {
        p.x += dx;
        p.y += dy;
    };
    for (Point &p : junctions) {
        move(p);
    }
    for (Way &w : ways) {
        for (Point &p : w.between) {
            move(p);
        }
    }
    for (Hold &h : holds) {
        move(h.at);
    }
    for (Stand &s : stands) {
        move(s.at);
        for (Point &p : s.lead) {
            move(p);
        }
    }
    for (List<Area> *list : {&surfaces, &buildings}) {
        for (Area &a : *list) {
            for (Point &p : a.corners) {
                move(p);
            }
        }
    }
    move(tower);
    for (Point &p : windsocks) {
        move(p);
    }
}

float Layout::roomAhead(Point from, float heading, float reach) const {
    float dx = cosf(heading), dy = sinf(heading);
    float x0 = std::min(from.x, from.x + dx * reach), x1 = std::max(from.x, from.x + dx * reach);
    float y0 = std::min(from.y, from.y + dy * reach), y1 = std::max(from.y, from.y + dy * reach);
    auto cross = [](float ax, float ay, float bx, float by) { return ax * by - ay * bx; };
    float room = reach;
    for (const Area &b : buildings) {
        // Only those whose box the way ahead crosses.
        float bx0 = 1e30f, by0 = 1e30f, bx1 = -1e30f, by1 = -1e30f;
        for (const Point &c : b.corners) {
            bx0 = std::min(bx0, c.x), by0 = std::min(by0, c.y);
            bx1 = std::max(bx1, c.x), by1 = std::max(by1, c.y);
        }
        if (bx0 > x1 || bx1 < x0 || by0 > y1 || by1 < y0) {
            continue;
        }
        for (size_t t = 0; t + 2 < b.triangles.size(); t += 3) {
            const Point *p[3] = {&b.corners[b.triangles[t]], &b.corners[b.triangles[t + 1]],
                                 &b.corners[b.triangles[t + 2]]};
            // In it already: no room at all.
            float s[3];
            for (int k = 0; k < 3; k++) {
                const Point &a = *p[k], &c = *p[(k + 1) % 3];
                s[k] = cross(c.x - a.x, c.y - a.y, from.x - a.x, from.y - a.y);
            }
            if ((s[0] >= 0 && s[1] >= 0 && s[2] >= 0) || (s[0] <= 0 && s[1] <= 0 && s[2] <= 0)) {
                return 0;
            }
            // Where the way ahead first crosses one of its edges.
            for (int k = 0; k < 3; k++) {
                const Point &a = *p[k], &c = *p[(k + 1) % 3];
                float ex = c.x - a.x, ey = c.y - a.y, d = cross(dx, dy, ex, ey);
                if (fabsf(d) < 1e-6f) {
                    continue;
                }
                float wx = a.x - from.x, wy = a.y - from.y;
                float along = cross(wx, wy, ex, ey) / d, on = cross(wx, wy, dx, dy) / d;
                if (along >= 0 && along < room && on >= 0 && on <= 1) {
                    room = along;
                }
            }
        }
    }
    return room;
}

namespace {

constexpr float UNIT = 0.25f;  // metres a step, as tools/gen_airfields.py packs points

// Reading a block: bytes in order, and whether it ran out.
struct Reader {
    const uint8_t *at, *end;
    bool ok = true;

    uint8_t byte() {
        if (at >= end) {
            ok = false;
            return 0;
        }
        return *at++;
    }
    uint32_t varint() {
        uint32_t v = 0;
        for (int shift = 0; shift < 35; shift += 7) {
            uint8_t b = byte();
            v |= (uint32_t)(b & 0x7F) << shift;
            if (!(b & 0x80)) {
                return v;
            }
        }
        ok = false;
        return 0;
    }
    int32_t signedVarint() {
        uint32_t z = varint();
        return (int32_t)(z >> 1) ^ -(int32_t)(z & 1);
    }
    int32_t i32() {
        uint32_t v = byte();
        v |= (uint32_t)byte() << 8;
        v |= (uint32_t)byte() << 16;
        v |= (uint32_t)byte() << 24;
        return (int32_t)v;
    }
    void ref(char out[8]) {
        uint8_t n = byte();
        memset(out, 0, 8);
        for (uint8_t i = 0; i < n; i++) {
            char c = (char)byte();
            if (i < 7) {
                out[i] = c;
            }
        }
    }
};

// A section's points, each from the one before, the first from (0, 0).
struct Points {
    int32_t x = 0, y = 0;
    Point next(Reader &r) {
        x += r.signedVarint();
        y += r.signedVarint();
        return {x * UNIT, y * UNIT};
    }
};

void area(Reader &r, Points &p, Area &a) {
    uint32_t corners = r.varint();
    for (uint32_t i = 0; i < corners && r.ok; i++) {
        a.corners.push_back(p.next(r));
    }
    uint32_t triangles = r.varint();
    for (uint32_t i = 0; i < triangles * 3 && r.ok; i++) {
        uint32_t c = r.varint();
        if (c >= corners) {
            r.ok = false;
            return;
        }
        a.triangles.push_back((uint16_t)c);
    }
}

}  // namespace

bool parse(const uint8_t *block, size_t size, Layout &out) {
    out.clear();
    Reader r{block, block + size};
    out.lat0 = r.i32() / 1e7;
    out.lon0 = r.i32() / 1e7;
    while (r.ok && r.at < r.end) {
        uint8_t kind = r.byte();
        uint32_t length = r.varint();
        if (!r.ok || length > (uint32_t)(r.end - r.at)) {
            return false;
        }
        Reader s{r.at, r.at + length};
        r.at += length;
        Points p;
        if (kind == 7) {  // the tower: a point and its height, no count
            out.tower = p.next(s);
            out.towerHeight = s.byte() * 0.5f;
            out.hasTower = s.ok;
            continue;
        }
        uint32_t count = s.varint();
        for (uint32_t i = 0; i < count && s.ok; i++) {
            switch (kind) {
            case 1:
                out.junctions.push_back(p.next(s));
                break;
            case 2: {
                Way w{};
                w.kind = (WayKind)s.byte();
                w.width = s.byte() * 0.5f;
                s.ref(w.ref);
                w.from = (uint16_t)s.varint();
                w.to = (uint16_t)s.varint();
                uint32_t between = s.varint();
                for (uint32_t k = 0; k < between && s.ok; k++) {
                    w.between.push_back(p.next(s));
                }
                out.ways.push_back(std::move(w));
                break;
            }
            case 3: {
                Hold h{};
                h.way = (uint16_t)s.varint();
                h.at = p.next(s);
                h.kind = (HoldKind)s.byte();
                s.ref(h.ref);
                out.holds.push_back(h);
                break;
            }
            case 4: {
                Stand st{};
                st.at = p.next(s);
                s.ref(st.ref);
                uint32_t lead = s.varint();
                for (uint32_t k = 0; k < lead && s.ok; k++) {
                    st.lead.push_back(p.next(s));
                }
                out.stands.push_back(std::move(st));
                break;
            }
            case 5: {
                Area a{};
                a.kind = s.byte();
                area(s, p, a);
                out.surfaces.push_back(std::move(a));
                break;
            }
            case 6: {
                Area a{};
                a.kind = s.byte();
                a.height = s.byte() * 0.5f;
                area(s, p, a);
                out.buildings.push_back(std::move(a));
                break;
            }
            case 8:
                out.windsocks.push_back(p.next(s));
                break;
            default:
                i = count;  // a kind this doesn't know: passed over whole
                break;
            }
        }
        if (!s.ok) {
            return false;
        }
    }
    // Every way's junctions there, and every hold's way.
    for (const Way &w : out.ways) {
        if (w.from >= out.junctions.size() || w.to >= out.junctions.size()) {
            return false;
        }
    }
    for (const Hold &h : out.holds) {
        if (h.way >= out.ways.size()) {
            return false;
        }
    }
    return r.ok;
}

#ifdef ESP_PLATFORM
bool load(const char *code, Layout &out) {
    out.clear();
    if (!recorder::mount()) {
        return false;
    }
    recorder::CardLock lock;
    File f = SD.open("/overhead/airfields.bin", FILE_READ);
    if (!f) {
        return false;
    }
    uint8_t head[16];
    if (f.read(head, sizeof head) != sizeof head || memcmp(head, "OVAF", 4) != 0 || head[4] != 1) {
        Serial.println("[airfield] airfields.bin isn't one this firmware reads");
        f.close();
        return false;
    }
    uint16_t count = head[6] | head[7] << 8;
    uint32_t indexAt = head[8] | head[9] << 8 | head[10] << 16 | (uint32_t)head[11] << 24;
    // The index sorted by code: halved until found.
    char want[4] = {};
    strncpy(want, code, 4);
    int lo = 0, hi = count - 1;
    uint32_t at = 0, size = 0;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        uint8_t row[12];
        if (!f.seek(indexAt + (uint32_t)mid * 12) || f.read(row, 12) != 12) {
            break;
        }
        int c = memcmp(row, want, 4);
        if (c == 0) {
            at = row[4] | row[5] << 8 | row[6] << 16 | (uint32_t)row[7] << 24;
            size = row[8] | row[9] << 8 | row[10] << 16 | (uint32_t)row[11] << 24;
            break;
        }
        if (c < 0) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    if (size == 0 || size > 1024 * 1024) {
        f.close();
        return false;
    }
    uint8_t *block = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    bool ok = block != nullptr && f.seek(at) && f.read(block, size) == size;
    f.close();
    ok = ok && parse(block, size, out);
    heap_caps_free(block);
    if (!ok) {
        Serial.printf("[airfield] %s's layout is damaged in airfields.bin\n", code);
        out.clear();
    }
    return ok;
}
#endif

}  // namespace airfield
