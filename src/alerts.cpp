#include "alerts.h"

namespace {

// ICAO type designators worth looking up for: the giants, the oddities and
// the old. Not exhaustive - the feed's own "interesting" flag catches a good
// many more - but these are the ones that turn heads at any airfield.
const char *const RARE_TYPES[] = {
    // very large or very unusual freighters and airliners
    "A388", "B741", "B742", "B743", "B744", "B748", "B74S", "B74R", "B74D", "A124", "A225", "AN22", "A3ST",
    "A337", "BLCF", "CONC",
    // military heavies and specials
    "C17", "C5", "C5M", "A400", "B52", "B1", "B2", "U2", "E3TF", "E3CF", "E6", "E8", "K35R", "V22", "A10",
    "F22", "F35", "EUFI", "RFAL", "TOR",
    // vintage and warbirds
    "SPIT", "HURI", "LANC", "P51", "DC3", "DC6", "B17", "B29", "AN2", "JU52", "CAT",
};

bool isRareType(const String &type) {
    for (const char *t : RARE_TYPES) {
        if (type == t) {
            return true;
        }
    }
    return false;
}

String withoutDashes(const String &s) {
    String out = s;
    out.replace("-", "");
    return out;
}

// A three-letter, all-alphabetic entry is an airline's ICAO prefix, matched
// against the front of a flight-number callsign (RYR -> RYR12AB). Anything
// else has to match a field exactly: callsign, registration (dashes
// optional, so G-ABCD and GABCD both work), type designator or ICAO hex.
bool matchesWatchlist(const Aircraft &ac, const std::vector<String> &watch) {
    if (watch.empty()) {
        return false;
    }
    String hex = ac.hex;
    hex.toUpperCase();
    String reg = withoutDashes(ac.reg);
    reg.toUpperCase();
    for (const String &token : watch) {
        if (token == ac.callsign || token == ac.type || token == hex) {
            return true;
        }
        if (reg.length() && withoutDashes(token) == reg) {
            return true;
        }
        if (token.length() == 3 && isAlpha(token[0]) && isAlpha(token[1]) && isAlpha(token[2]) &&
            ac.callsign.length() > 3 && ac.callsign.startsWith(token) && isDigit(ac.callsign[3])) {
            return true;
        }
    }
    return false;
}

}  // namespace

std::vector<String> parseWatchlist(const String &text) {
    std::vector<String> out;
    String token;
    for (size_t i = 0; i <= text.length(); i++) {
        char c = (i < text.length()) ? text[i] : ' ';
        if (c == ' ' || c == ',' || c == ';') {
            if (token.length()) {
                token.toUpperCase();
                out.push_back(token);
                token = "";
            }
        } else {
            token += c;
        }
    }
    return out;
}

String watchlistProblem(const String &text) {
    String bad;
    int count = 0;
    for (const String &token : parseWatchlist(text)) {
        bool ok = token.length() >= 2 && token.length() <= 10;
        for (size_t i = 0; ok && i < token.length(); i++) {
            char c = token[i];
            ok = isAlphaNumeric(c) || c == '-';
        }
        if (!ok) {
            if (count < 3) {
                bad += (bad.length() ? ", " : "") + token;
            }
            count++;
        }
    }
    if (count == 0) {
        return "";
    }
    if (count > 3) {
        bad += " and " + String(count - 3) + " more";
    }
    return "Can't match " + bad + " - use letters, digits and dashes, 2 to 10 long";
}

uint8_t evaluateAlert(const Aircraft &ac, const AlertRules &rules, bool civilMode) {
    uint8_t reasons = 0;
    if ((rules.enabled & ALERT_EMERGENCY) &&
        (ac.squawk == "7700" || ac.squawk == "7600" || ac.squawk == "7500")) {
        reasons |= ALERT_EMERGENCY;
    }
    if ((rules.enabled & ALERT_MILITARY) && civilMode && ac.military) {
        reasons |= ALERT_MILITARY;
    }
    if ((rules.enabled & ALERT_WATCHLIST) && matchesWatchlist(ac, rules.watch)) {
        reasons |= ALERT_WATCHLIST;
    }
    if ((rules.enabled & ALERT_RARE) && (ac.dbInteresting || isRareType(ac.type))) {
        reasons |= ALERT_RARE;
    }
    return reasons;
}

String alertReasonText(const Aircraft &ac, uint8_t reasons) {
    if (reasons & ALERT_EMERGENCY) {
        if (ac.squawk == "7500") return "HIJACK (7500)";
        if (ac.squawk == "7600") return "RADIO FAILURE (7600)";
        return "EMERGENCY (7700)";
    }
    // The one you asked for outranks the ones the device picked out for you.
    if (reasons & ALERT_WATCHLIST) return "WATCHLIST";
    if (reasons & ALERT_RARE) return "RARE TYPE";
    if (reasons & ALERT_MILITARY) return "MILITARY";
    return "";
}

bool alertOutranks(const Aircraft &a, const Aircraft &b) {
    bool ea = (a.alert & ALERT_EMERGENCY) != 0, eb = (b.alert & ALERT_EMERGENCY) != 0;
    if (ea != eb) {
        return ea;
    }
    float da = a.hasDist ? a.distNm : 1e9f;
    float db = b.hasDist ? b.distNm : 1e9f;
    return da < db;
}

namespace {

constexpr int FLASH_COUNT = 3;
constexpr uint32_t FLASH_PHASE_MS = 300;

std::vector<String> g_flashHexes;
uint32_t g_flashStartMs = 0;
uint32_t g_flashGen = 0;  // so back-to-back flashes never share a step

// Which half-flash is showing, or -1 when no flash is running.
int flashPhase() {
    if (g_flashHexes.empty()) {
        return -1;
    }
    uint32_t phase = (millis() - g_flashStartMs) / FLASH_PHASE_MS;
    if (phase >= FLASH_COUNT * 2) {
        g_flashHexes.clear();
        return -1;
    }
    return (int)phase;
}

}  // namespace

void alertFlashStart(const std::vector<String> &hexes) {
    g_flashHexes = hexes;
    g_flashStartMs = millis();
    g_flashGen++;
}

AlertFlash alertFlash(const String &hex) {
    int phase = flashPhase();
    if (phase < 0) {
        return AlertFlash::NONE;
    }
    for (const String &h : g_flashHexes) {
        if (h == hex) {
            return (phase % 2 == 0) ? AlertFlash::RED : AlertFlash::OFF;
        }
    }
    return AlertFlash::NONE;
}

uint32_t alertFlashStep() {
    int phase = flashPhase();
    return phase < 0 ? 0 : g_flashGen * (FLASH_COUNT * 2) + (uint32_t)phase + 1;
}
