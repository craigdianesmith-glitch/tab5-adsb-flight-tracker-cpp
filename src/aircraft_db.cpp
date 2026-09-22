#include "aircraft_db.h"

namespace {

struct TypeEntry {
    const char *code;
    const char *name;
};

// Common ICAO type designators seen in UK/European airspace, plus major
// long-haul and military types. Not exhaustive - unknown codes just fall
// back to showing the raw code, same as before.
const TypeEntry TYPES[] = {
    // Airbus
    {"A19N", "Airbus A319neo"}, {"A20N", "Airbus A320neo"}, {"A21N", "Airbus A321neo"},
    {"A318", "Airbus A318"}, {"A319", "Airbus A319"}, {"A320", "Airbus A320"}, {"A321", "Airbus A321"},
    {"A332", "Airbus A330-200"}, {"A333", "Airbus A330-300"}, {"A338", "Airbus A330-800neo"},
    {"A339", "Airbus A330-900neo"}, {"A342", "Airbus A340-200"}, {"A343", "Airbus A340-300"},
    {"A345", "Airbus A340-500"}, {"A346", "Airbus A340-600"}, {"A359", "Airbus A350-900"},
    {"A35K", "Airbus A350-1000"}, {"A388", "Airbus A380-800"}, {"A400", "Airbus A400M Atlas"},
    // Boeing
    {"B734", "Boeing 737-400"}, {"B735", "Boeing 737-500"}, {"B736", "Boeing 737-600"},
    {"B737", "Boeing 737-700"}, {"B738", "Boeing 737-800"}, {"B739", "Boeing 737-900"},
    {"B37M", "Boeing 737 MAX 7"}, {"B38M", "Boeing 737 MAX 8"}, {"B39M", "Boeing 737 MAX 9"},
    {"B3XM", "Boeing 737 MAX 10"}, {"B744", "Boeing 747-400"}, {"B748", "Boeing 747-8"},
    {"B752", "Boeing 757-200"}, {"B753", "Boeing 757-300"}, {"B762", "Boeing 767-200"},
    {"B763", "Boeing 767-300"}, {"B764", "Boeing 767-400"}, {"B772", "Boeing 777-200"},
    {"B773", "Boeing 777-300"}, {"B77L", "Boeing 777-200LR"}, {"B77W", "Boeing 777-300ER"},
    {"B788", "Boeing 787-8 Dreamliner"}, {"B789", "Boeing 787-9 Dreamliner"},
    {"B78X", "Boeing 787-10 Dreamliner"},
    // Embraer
    {"E135", "Embraer ERJ-135"}, {"E145", "Embraer ERJ-145"}, {"E170", "Embraer E170"},
    {"E175", "Embraer E175"}, {"E190", "Embraer E190"}, {"E195", "Embraer E195"},
    {"E290", "Embraer E190-E2"}, {"E295", "Embraer E195-E2"},
    // Bombardier / De Havilland Canada
    {"CRJ2", "Bombardier CRJ200"}, {"CRJ7", "Bombardier CRJ700"}, {"CRJ9", "Bombardier CRJ900"},
    {"CRJX", "Bombardier CRJ1000"}, {"DH8A", "De Havilland Dash 8-100"}, {"DH8B", "De Havilland Dash 8-200"},
    {"DH8C", "De Havilland Dash 8-300"}, {"DH8D", "De Havilland Dash 8 Q400"},
    {"DHC6", "De Havilland DHC-6 Twin Otter"},
    // ATR
    {"AT43", "ATR 42-300"}, {"AT45", "ATR 42-500"}, {"AT46", "ATR 42-600"}, {"AT72", "ATR 72-200"},
    {"AT75", "ATR 72-500"}, {"AT76", "ATR 72-600"},
    // GA / light
    {"C172", "Cessna 172 Skyhawk"}, {"C152", "Cessna 152"}, {"C182", "Cessna 182 Skylane"},
    {"C206", "Cessna 206"}, {"C208", "Cessna Grand Caravan"}, {"C25A", "Cessna Citation CJ2"},
    {"C25B", "Cessna Citation CJ3"}, {"C25C", "Cessna Citation CJ4"}, {"C550", "Cessna Citation II"},
    {"C56X", "Cessna Citation Excel/XLS"}, {"C680", "Cessna Citation Sovereign"},
    {"C750", "Cessna Citation X"}, {"PA28", "Piper Cherokee"}, {"P28A", "Piper Cherokee"},
    {"PA34", "Piper Seneca"}, {"SR22", "Cirrus SR22"}, {"BE20", "Beechcraft King Air 200"},
    {"BE9L", "Beechcraft King Air 90"}, {"P180", "Piaggio P180 Avanti"},
    // Business jets
    {"GLF4", "Gulfstream IV"}, {"GLF5", "Gulfstream V"}, {"GLF6", "Gulfstream G650"},
    {"GLEX", "Bombardier Global Express"}, {"CL30", "Bombardier Challenger 300"},
    {"CL60", "Bombardier Challenger 600"}, {"FA7X", "Dassault Falcon 7X"}, {"FA8X", "Dassault Falcon 8X"},
    {"F900", "Dassault Falcon 900"}, {"LJ45", "Learjet 45"}, {"H25B", "Hawker 800/850"},
    // Helicopters
    {"EC35", "Eurocopter EC135"}, {"EC45", "Eurocopter EC145"}, {"AS50", "Eurocopter AS350 Squirrel"},
    {"A109", "Agusta A109"}, {"A139", "AgustaWestland AW139"}, {"R44", "Robinson R44"},
    {"S76", "Sikorsky S-76"}, {"H160", "Airbus H160"},
    // Military
    {"C130", "Lockheed C-130 Hercules"}, {"E3TF", "Boeing E-3 Sentry AWACS"},
    {"RC135", "Boeing RC-135"}, {"K35R", "Boeing KC-135R Stratotanker"}, {"F16", "F-16 Fighting Falcon"},
    {"F35", "F-35 Lightning II"}, {"TYPH", "Eurofighter Typhoon"}, {"TUCA", "Embraer Tucano"},
    {"HAWK", "BAE Hawk"}, {"VC10", "Vickers VC10"},
};

struct AirlineEntry {
    const char *prefix;  // 3-letter ICAO callsign prefix
    const char *name;
    uint32_t color;
};

const AirlineEntry AIRLINES[] = {
    {"BAW", "British Airways", 0x075AAA}, {"SHT", "British Airways", 0x075AAA},
    {"EZY", "easyJet", 0xFF6600}, {"RYR", "Ryanair", 0x073590}, {"LOG", "Loganair", 0x582C83},
    {"BEE", "Flybe", 0x5C2D91}, {"WZZ", "Wizz Air", 0xC6007E}, {"VIR", "Virgin Atlantic", 0xE10A17},
    {"TOM", "TUI Airways", 0xE4032E}, {"EXS", "Jet2.com", 0xE2231A}, {"DLH", "Lufthansa", 0x05164D},
    {"SWR", "Swiss", 0xCC0000}, {"AUA", "Austrian Airlines", 0xCC0000}, {"KLM", "KLM", 0x00A1DE},
    {"AFR", "Air France", 0x002157}, {"IBE", "Iberia", 0xD7192D}, {"VLG", "Vueling", 0xF5A623},
    {"SAS", "Scandinavian Airlines", 0x003057}, {"FIN", "Finnair", 0x0F1689}, {"NAX", "Norwegian", 0xD91C24},
    {"THY", "Turkish Airlines", 0xC70A0C}, {"UAE", "Emirates", 0xD71921}, {"QTR", "Qatar Airways", 0x5C0632},
    {"ETD", "Etihad Airways", 0xBD8B13}, {"SVA", "Saudia", 0x00693C}, {"AAL", "American Airlines", 0x0078D2},
    {"UAL", "United Airlines", 0x002244}, {"DAL", "Delta Air Lines", 0x003366}, {"JBU", "JetBlue", 0x00205B},
    {"SWA", "Southwest Airlines", 0x304CB2}, {"ACA", "Air Canada", 0xD22730}, {"CPA", "Cathay Pacific", 0x006564},
    {"SIA", "Singapore Airlines", 0x1B3E94}, {"ANA", "All Nippon Airways", 0x13448F},
    {"JAL", "Japan Airlines", 0xBE0000}, {"QFA", "Qantas", 0xE40C0C}, {"EIN", "Aer Lingus", 0x006272},
    {"LOT", "LOT Polish Airlines", 0x11225E}, {"TAP", "TAP Air Portugal", 0x00A19A},
    {"BEL", "Brussels Airlines", 0x00539F}, {"ITY", "ITA Airways", 0x008C45},
    {"PGT", "Pegasus Airlines", 0xFEDB00}, {"WIF", "Wideroe", 0x00854A},
    {"RJA", "Royal Jordanian", 0x6E1E3B}, {"GFA", "Gulf Air", 0x8A1538}, {"MSR", "EgyptAir", 0x004B87},
    {"ETH", "Ethiopian Airlines", 0x078930},
};

}  // namespace

String lookupAircraftType(const String &icaoCode) {
    for (const auto &t : TYPES) {
        if (icaoCode == t.code) {
            return t.name;
        }
    }
    return icaoCode;
}

bool lookupAirline(const String &callsign, AirlineInfo &out) {
    if (callsign.length() < 3) {
        return false;
    }
    String prefix = callsign.substring(0, 3);
    for (const auto &a : AIRLINES) {
        if (prefix == a.prefix) {
            out.name = a.name;
            out.color = a.color;
            return true;
        }
    }
    return false;
}
