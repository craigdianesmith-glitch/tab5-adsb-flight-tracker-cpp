#include "filter.h"

#include "aircraft_db.h"
#include "alerts.h"

namespace {

TrafficFilterSet g_filter;

// Whether `value` begins with any of the words in `list`, separated by
// spaces or commas - or, with `orNone`, `list` has none.
bool matchesAny(const String &list, const String &value, bool orNone = true) {
    bool any = false;
    int from = 0, n = list.length();
    while (from < n) {
        while (from < n && (list[from] == ' ' || list[from] == ',')) {
            from++;
        }
        int to = from;
        while (to < n && list[to] != ' ' && list[to] != ',') {
            to++;
        }
        if (to > from) {
            any = true;
            if (value.startsWith(list.substring(from, to))) {
                return true;
            }
        }
        from = to;
    }
    return orNone && !any;
}

}  // namespace

bool TrafficFilterSet::active() const {
    return minFt > 0 || maxFt < FILTER_TOP_FT || minKt > 0 || maxKt < FILTER_TOP_KT || !civil || !cargo || !military ||
           airlines.length() || types.length() || hideTypes.length();
}

bool TrafficFilterSet::passes(const Aircraft &a) const {
    bool kind = a.military ? military : isCargo(a.callsign) ? cargo : civil;
    if (!kind) {
        return false;
    }
    // A height or speed not known passes only where none is asked for.
    if (minFt > 0 || maxFt < FILTER_TOP_FT) {
        if (a.altStr == "?") {
            return false;
        }
        int ft = a.altStr == "GND" ? 0 : a.altStr.toInt();
        if (ft < minFt || (maxFt < FILTER_TOP_FT && ft > maxFt)) {
            return false;
        }
    }
    if (minKt > 0 || maxKt < FILTER_TOP_KT) {
        if (a.speedStr == "?") {
            return false;
        }
        int kt = a.speedStr.toInt();
        if (kt < minKt || (maxKt < FILTER_TOP_KT && kt > maxKt)) {
            return false;
        }
    }
    String cs = a.callsign;
    cs.toUpperCase();
    String type = a.type;
    type.toUpperCase();
    // Left out by name before anything is let in by it: "GLID" hides the
    // gliders whatever the types asked for.
    if (type.length() && matchesAny(hideTypes, type, false)) {
        return false;
    }
    return matchesAny(airlines, cs) && matchesAny(types, type);
}

const TrafficFilterSet &trafficFilter() { return g_filter; }

String filterText(const TrafficFilterSet &f) {
    // The words with commas between rather than spaces, so each list is one
    // token; "-" for none.
    auto words = [](String w) {
        w.trim();
        w.replace(' ', ',');
        return w.length() ? w : String("-");
    };
    String kinds = String(f.civil ? 'C' : '-') + (f.cargo ? 'G' : '-') + (f.military ? 'M' : '-');
    return String(f.minFt) + " " + String(f.maxFt) + " " + String(f.minKt) + " " + String(f.maxKt) + " " + kinds +
           " " + words(f.airlines) + " " + words(f.types) + " " + words(f.hideTypes);
}

bool parseFilterText(const char *text, TrafficFilterSet &out) {
    int minFt, maxFt, minKt, maxKt;
    char kinds[4] = "", airlines[96] = "", types[96] = "", hide[96] = "-";
    int n = sscanf(text, "%d %d %d %d %3s %95s %95s %95s", &minFt, &maxFt, &minKt, &maxKt, kinds, airlines, types, hide);
    if (n < 7 || strlen(kinds) != 3) {
        return false;
    }
    out = TrafficFilterSet();
    out.minFt = minFt;
    out.maxFt = maxFt;
    out.minKt = minKt;
    out.maxKt = maxKt;
    out.civil = kinds[0] == 'C';
    out.cargo = kinds[1] == 'G';
    out.military = kinds[2] == 'M';
    auto words = [](const char *w) {
        String s = strcmp(w, "-") == 0 ? String() : String(w);
        s.replace(',', ' ');
        return s;
    };
    out.airlines = words(airlines);
    out.types = words(types);
    out.hideTypes = words(hide);
    return true;
}

String shownKinds(bool code) { return kindsName(g_filter, false, code); }

String kindsName(const TrafficFilterSet &f, bool militaryMode, bool code) {
    if (militaryMode) {
        return code ? "MIL" : "Military";
    }
    if (f.civil && f.cargo) {
        return code ? "CIV" : "Civilian";
    }
    if (f.civil && f.military) {
        return code ? "CIV+MIL" : "Civil & military";
    }
    if (f.cargo && f.military) {
        return code ? "CRG+MIL" : "Cargo & military";
    }
    if (f.cargo) {
        return code ? "CRG" : "Cargo";
    }
    if (f.military) {
        return code ? "MIL" : "Military";
    }
    return code ? "CIV" : "Civilian";
}

int radiusCeilingNm(bool filtered) { return filtered ? FILTERED_MAX_RADIUS_NM : MAX_RADIUS_NM; }

void setTrafficFilter(const TrafficFilterSet &f) {
    g_filter = f;
    g_filter.airlines.toUpperCase();
    g_filter.types.toUpperCase();
}

void applyTrafficFilter(std::vector<Aircraft> &list, std::vector<uint8_t> *isNew, const String &keepHex) {
    filterList(g_filter, list, isNew, keepHex);
}

void filterList(const TrafficFilterSet &f, std::vector<Aircraft> &list, std::vector<uint8_t> *isNew,
                const String &keepHex) {
    if (!f.active()) {
        return;
    }
    size_t out = 0;
    for (size_t i = 0; i < list.size(); i++) {
        // An emergency is never hidden, whatever the filter says.
        bool keep = (list[i].alert & ALERT_EMERGENCY) || (keepHex.length() && list[i].hex == keepHex);
        if (keep || f.passes(list[i])) {
            if (out != i) {
                list[out] = list[i];
                if (isNew && i < isNew->size()) {
                    (*isNew)[out] = (*isNew)[i];
                }
            }
            out++;
        }
    }
    list.resize(out);
    if (isNew && isNew->size() > out) {
        isNew->resize(out);
    }
}
