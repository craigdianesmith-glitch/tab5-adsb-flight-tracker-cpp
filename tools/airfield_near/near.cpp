// Every OpenStreetMap object the airfield layouts are made from - aeroway
// other than the aerodrome itself, buildings, towers, windsocks - with its
// geometry, where it lies in the cells given: those over and round the
// aerodromes wanted, which tools/gen_airfields.py works out. One line of JSON
// each, as the Overpass API would give it, for gen_airfields.py to share out
// among its airports. For the planet: pyosmium places a way's nodes from an
// index of every node, ten billion of them, and on a machine without a
// hundred gigabytes of memory that index is read from disk at random for
// every way; this keeps only the nodes in the cells, a few tens of millions,
// and reads the file twice, straight through.
//
//   near FILE.osm.pbf CELLS > near.jsonl
//
// CELLS: the cells, one a line, "y x" - floor(latitude * 200) and
// floor(longitude * 200), so a 200th of a degree square.
//
// Build, with libosmium's and protozero's headers (github.com/osmcode/libosmium,
// github.com/mapbox/protozero) and zlib:
//
//   g++ -O2 -std=c++17 -I LIBOSMIUM/include -I PROTOZERO/include \
//       tools/airfield_near/near.cpp -o near -lz -lpthread

#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/osm.hpp>
#include <osmium/visitor.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

constexpr double PER_DEGREE = 200;

int64_t cellOf(double lat, double lon) {
    int64_t y = (int64_t)std::floor(lat * PER_DEGREE), x = (int64_t)std::floor(lon * PER_DEGREE);
    return y * 1000000 + x;
}

// What the layouts are made of, by its tags: the aerodrome's own outline is
// found elsewhere, and of man_made only a tower or a windsock is wanted.
bool wanted(const osmium::TagList &tags, bool node) {
    const char *aeroway = tags["aeroway"];
    if (aeroway != nullptr && std::string(aeroway) != "aerodrome") {
        return true;
    }
    if (tags["building"] != nullptr) {
        return true;
    }
    const char *manMade = tags["man_made"];
    if (manMade == nullptr) {
        return false;
    }
    std::string m = manMade;
    return m == "tower" || (node && m == "windsock");
}

void json(std::string &out, const char *s) {
    out += '"';
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
        switch (*p) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (*p < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof buf, "\\u%04x", *p);
                out += buf;
            } else {
                out += (char)*p;
            }
        }
    }
    out += '"';
}

void tagsOf(std::string &out, const osmium::TagList &tags) {
    out += "\"tags\":{";
    bool first = true;
    for (const osmium::Tag &tag : tags) {
        if (!first) {
            out += ',';
        }
        first = false;
        json(out, tag.key());
        out += ':';
        json(out, tag.value());
    }
    out += '}';
}

void point(std::string &out, osmium::Location l) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "{\"lat\":%.7f,\"lon\":%.7f}", l.lat(), l.lon());
    out += buf;
}

void geometry(std::string &out, const std::vector<osmium::Location> &line) {
    out += "\"geometry\":[";
    for (size_t i = 0; i < line.size(); i++) {
        if (i) {
            out += ',';
        }
        point(out, line[i]);
    }
    out += ']';
}

// The multipolygons that might be wanted - a building with a courtyard, an
// apron drawn as one - and the ways they are made of, from the first pass.
class Relations : public osmium::handler::Handler {
  public:
    std::vector<osmium::object_id_type> relations, memberWays;

    void relation(const osmium::Relation &r) {
        if (!wanted(r.tags(), false)) {
            return;
        }
        bool any = false;
        for (const osmium::RelationMember &m : r.members()) {
            if (m.type() == osmium::item_type::way) {
                memberWays.push_back(m.ref());
                any = true;
            }
        }
        if (any) {
            relations.push_back(r.id());
        }
    }
};

class Near : public osmium::handler::Handler {
  public:
    Near(const std::unordered_set<int64_t> &cells, std::vector<osmium::object_id_type> relations,
         std::vector<osmium::object_id_type> memberWays)
        : cells_(cells), relations_(std::move(relations)), memberWays_(std::move(memberWays)) {
        std::sort(relations_.begin(), relations_.end());
        std::sort(memberWays_.begin(), memberWays_.end());
        memberWays_.erase(std::unique(memberWays_.begin(), memberWays_.end()), memberWays_.end());
    }

    uint64_t nodesKept() const { return locations_.size(); }
    uint64_t written = 0;

    void node(const osmium::Node &n) {
        osmium::Location l = n.location();
        if (!l.valid() || cells_.count(cellOf(l.lat(), l.lon())) == 0) {
            return;
        }
        // The planet's nodes come in order of their ids: kept so, for a
        // binary search.
        locations_.emplace_back(n.id(), l);
        if (!n.tags().empty() && wanted(n.tags(), true)) {
            line_.clear();
            char buf[96];
            std::snprintf(buf, sizeof buf, "{\"type\":\"node\",\"id\":%lld,\"lat\":%.7f,\"lon\":%.7f,",
                          (long long)n.id(), l.lat(), l.lon());
            line_ += buf;
            tagsOf(line_, n.tags());
            line_ += "}\n";
            emit();
        }
    }

    void way(const osmium::Way &w) {
        bool want = wanted(w.tags(), false);
        bool member = std::binary_search(memberWays_.begin(), memberWays_.end(), w.id());
        if (!want && !member) {
            return;
        }
        // Only a way all of whose nodes are in the cells: one reaching out
        // of them is nowhere near an aerodrome wanted.
        points_.clear();
        for (const osmium::NodeRef &nr : w.nodes()) {
            auto it = std::lower_bound(locations_.begin(), locations_.end(), nr.ref(),
                                       [](const std::pair<osmium::object_id_type, osmium::Location> &a,
                                          osmium::object_id_type id) { return a.first < id; });
            if (it == locations_.end() || it->first != nr.ref()) {
                return;
            }
            points_.push_back(it->second);
        }
        if (points_.size() < 2) {
            return;
        }
        if (member) {
            memberGeometry_[w.id()] = points_;
        }
        if (!want) {
            return;
        }
        line_.clear();
        char buf[64];
        std::snprintf(buf, sizeof buf, "{\"type\":\"way\",\"id\":%lld,\"nodes\":[", (long long)w.id());
        line_ += buf;
        bool first = true;
        for (const osmium::NodeRef &nr : w.nodes()) {
            if (!first) {
                line_ += ',';
            }
            first = false;
            line_ += std::to_string(nr.ref());
        }
        line_ += "],";
        tagsOf(line_, w.tags());
        line_ += ',';
        geometry(line_, points_);
        line_ += "}\n";
        emit();
    }

    void relation(const osmium::Relation &r) {
        if (!std::binary_search(relations_.begin(), relations_.end(), r.id())) {
            return;
        }
        // All its ways in the cells, or none of it: half a building's outline
        // would make some other shape.
        for (const osmium::RelationMember &m : r.members()) {
            if (m.type() == osmium::item_type::way && memberGeometry_.count(m.ref()) == 0) {
                return;
            }
        }
        line_.clear();
        char buf[64];
        std::snprintf(buf, sizeof buf, "{\"type\":\"relation\",\"id\":%lld,", (long long)r.id());
        line_ += buf;
        tagsOf(line_, r.tags());
        line_ += ",\"members\":[";
        bool first = true;
        for (const osmium::RelationMember &m : r.members()) {
            if (m.type() != osmium::item_type::way) {
                continue;
            }
            if (!first) {
                line_ += ',';
            }
            first = false;
            std::snprintf(buf, sizeof buf, "{\"type\":\"way\",\"ref\":%lld,\"role\":", (long long)m.ref());
            line_ += buf;
            json(line_, m.role());
            line_ += ',';
            geometry(line_, memberGeometry_[m.ref()]);
            line_ += '}';
        }
        line_ += "]}\n";
        emit();
    }

  private:
    void emit() {
        std::fwrite(line_.data(), 1, line_.size(), stdout);
        written++;
    }

    const std::unordered_set<int64_t> &cells_;
    std::vector<osmium::object_id_type> relations_, memberWays_;
    std::vector<std::pair<osmium::object_id_type, osmium::Location>> locations_;
    std::unordered_map<osmium::object_id_type, std::vector<osmium::Location>> memberGeometry_;
    std::vector<osmium::Location> points_;
    std::string line_;
};

}  // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: near FILE.osm.pbf CELLS > near.jsonl\n");
        return 2;
    }
    std::unordered_set<int64_t> cells;
    std::ifstream in(argv[2]);
    long long y, x;
    while (in >> y >> x) {
        cells.insert(y * 1000000 + x);
    }
    std::fprintf(stderr, "near: %zu cells\n", cells.size());

    Relations rels;
    {
        osmium::io::Reader reader{argv[1], osmium::osm_entity_bits::relation};
        osmium::apply(reader, rels);
        reader.close();
    }
    std::fprintf(stderr, "near: %zu multipolygons that might be wanted, %zu ways in them\n", rels.relations.size(),
                 rels.memberWays.size());

    Near near(cells, std::move(rels.relations), std::move(rels.memberWays));
    {
        osmium::io::Reader reader{argv[1], osmium::osm_entity_bits::nwr};
        osmium::apply(reader, near);
        reader.close();
    }
    std::fprintf(stderr, "near: %llu nodes in the cells, %llu objects written\n",
                 (unsigned long long)near.nodesKept(), (unsigned long long)near.written);
    return 0;
}
