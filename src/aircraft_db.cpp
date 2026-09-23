#include "aircraft_db.h"

#include <iterator>

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
    {"E75L", "Embraer E175"}, {"E75S", "Embraer E175"}, {"E190", "Embraer E190"}, {"E195", "Embraer E195"},
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
    {"C206", "Cessna 206"}, {"C208", "Cessna 208 Caravan"}, {"C25A", "Cessna Citation CJ2"},
    {"C25B", "Cessna Citation CJ3"}, {"C25C", "Cessna Citation CJ4"}, {"C550", "Cessna Citation II"},
    {"C56X", "Cessna Citation Excel/XLS"}, {"C680", "Cessna Citation Sovereign"},
    {"C750", "Cessna Citation X"}, {"P28A", "Piper Cherokee"},
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
    // Civil aircraft whose type code the military table also lists, so a
    // civil-registered one doesn't pick up a military variant's name.
    {"A310", "Airbus A310"}, {"B350", "Beechcraft King Air 350"}, {"B190", "Beechcraft 1900"},
    {"SW4", "Swearingen Metroliner"}, {"LJ35", "Learjet 35"}, {"C560", "Cessna Citation V"},
    {"SR20", "Cirrus SR20"}, {"E550", "Embraer Praetor 600"}, {"TBM7", "Socata TBM 700"},
    {"AT8T", "Air Tractor AT-802"}, {"DG1T", "DG Flugzeugbau DG-1000T"}, {"B412", "Bell 412"},
    {"S92", "Sikorsky S-92"}, {"A119", "Leonardo AW119 Koala"}, {"EC25", "Airbus EC225 Super Puma"},
    {"AS32", "Airbus AS332 Super Puma"}, {"AS55", "Airbus AS355 Ecureuil 2"},
    {"AS65", "Airbus AS365 Dauphin"}, {"GAZL", "Aerospatiale Gazelle"}, {"MI8", "Mil Mi-8"},
};

// Military types, consulted first for anything adsb.lol flags as military.
// Every designator here was checked against the ICAO doc 8643 list rather
// than written from memory - the two entries this table replaces, "RC135"
// and "TYPH", were both wrong and could never have matched (the real codes
// are R135 and EUFI).
const TypeEntry MIL_TYPES[] = {
    // Transports and tankers
    {"C130", "Lockheed C-130 Hercules"}, {"C30J", "Lockheed C-130J Super Hercules"},
    {"C17", "Boeing C-17 Globemaster III"}, {"C5M", "Lockheed C-5M Super Galaxy"}, {"A400", "Airbus A400M Atlas"},
    {"C27J", "Leonardo C-27J Spartan"}, {"C295", "Airbus C295 Persuader"}, {"CN35", "Airbus CN-235 Persuader"},
    {"C160", "Transall C-160"}, {"K35R", "Boeing KC-135R Stratotanker"}, {"K35E", "Boeing KC-135E Stratotanker"},
    {"C135", "Boeing WC-135 Constant Phoenix"}, {"B762", "Boeing KC-46 Pegasus"},
    {"A332", "Airbus A330 MRTT Voyager"}, {"A310", "Airbus A310 MRTT"}, {"B737", "Boeing C-40 Clipper"},
    {"DHC6", "De Havilland UV-18 Twin Otter"}, {"SW4", "Swearingen C-26 Metroliner"},
    {"B190", "Beechcraft C-12J Huron"}, {"LJ35", "Learjet C-21A"},
    // Surveillance, patrol and command
    {"P8", "Boeing P-8 Poseidon"}, {"P3", "Lockheed P-3 Orion"}, {"P1", "Kawasaki P-1"},
    {"R135", "Boeing RC-135 Rivet Joint"}, {"E3TF", "Boeing E-3 Sentry AWACS"},
    {"E3CF", "Boeing E-3 Sentry AWACS"}, {"E737", "Boeing E-7 Wedgetail"}, {"E6", "Boeing E-6 Mercury"},
    {"E2", "Grumman E-2 Hawkeye"}, {"U2", "Lockheed U-2 Dragon Lady"}, {"B350", "Beechcraft King Air 350 Shadow"},
    {"BE20", "Beechcraft C-12 Huron"}, {"C560", "Cessna UC-35 Citation"}, {"GLF5", "Gulfstream C-37A"},
    {"E550", "Embraer EMB-550 Praetor"},
    // Combat
    {"EUFI", "Eurofighter Typhoon"}, {"F15", "McDonnell Douglas F-15 Eagle"},
    {"F16", "General Dynamics F-16 Fighting Falcon"}, {"F18H", "McDonnell Douglas F/A-18 Hornet"},
    {"F18S", "Boeing F/A-18 Super Hornet"}, {"F22", "Lockheed Martin F-22 Raptor"},
    {"F35", "Lockheed Martin F-35 Lightning II"}, {"A10", "Fairchild A-10 Thunderbolt II"},
    {"B1", "Rockwell B-1 Lancer"}, {"B2", "Northrop Grumman B-2 Spirit"}, {"B52", "Boeing B-52 Stratofortress"},
    {"TOR", "Panavia Tornado"}, {"RFAL", "Dassault Rafale"}, {"MIR2", "Dassault Mirage 2000"},
    {"JAGR", "SEPECAT Jaguar"},
    // Trainers
    {"HAWK", "BAE Systems Hawk"}, {"TEX2", "Beechcraft T-6 Texan II"}, {"T38", "Northrop T-38 Talon"},
    {"T34T", "Beechcraft T-34C Turbo Mentor"}, {"TUCA", "Embraer EMB-312 Tucano"}, {"E314", "Embraer EMB-314 Super Tucano"},
    {"BT7", "Boeing T-7 Red Hawk"}, {"SR20", "Cirrus T-53"}, {"DG1T", "DG Flugzeugbau DG-1000T"},
    // Helicopters and tilt-rotor
    {"H60", "Sikorsky H-60 Black Hawk"}, {"H47", "Boeing CH-47 Chinook"}, {"H64", "Boeing AH-64 Apache"},
    {"H53", "Sikorsky CH-53 Sea Stallion"}, {"H53S", "Sikorsky CH-53E Super Stallion"},
    {"V22", "Bell Boeing V-22 Osprey"}, {"EH10", "AgustaWestland AW101 Merlin"},
    {"LYNX", "Leonardo AW159 Wildcat"}, {"NH90", "NHIndustries NH90"}, {"AS65", "Aerospatiale AS565 Panther"},
    {"AS32", "Aerospatiale AS532 Cougar"}, {"AS55", "Aerospatiale AS555 Fennec"},
    {"EC25", "Airbus EC225 Super Puma"}, {"EC45", "Airbus UH-72 Lakota"}, {"EC35", "Airbus H135 Juno"},
    {"MI8", "Mil Mi-8 Hip"}, {"GAZL", "Aerospatiale SA342 Gazelle"}, {"S92", "Sikorsky S-92"},
    {"A119", "Leonardo AW119 Koala"}, {"SUCO", "Bell AH-1Z Viper"}, {"UH1", "Bell UH-1 Iroquois"},
    {"UH1Y", "Bell UH-1Y Venom"}, {"B412", "Bell 412"},
    // Uncrewed
    {"Q9", "General Atomics MQ-9 Reaper"}, {"Q4", "Northrop Grumman RQ-4 Global Hawk"},
    {"Q1", "General Atomics MQ-1 Predator"},
    // Other
    {"AT8T", "Air Tractor AT-802"}, {"TBM7", "Socata TBM 700"},
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

namespace {

const char *findType(const TypeEntry *table, size_t count, const String &code) {
    for (size_t i = 0; i < count; i++) {
        if (code == table[i].code) {
            return table[i].name;
        }
    }
    return nullptr;
}

}  // namespace

String lookupAircraftType(const String &icaoCode, bool military) {
    const TypeEntry *first = military ? MIL_TYPES : TYPES;
    size_t firstCount = military ? std::size(MIL_TYPES) : std::size(TYPES);
    const TypeEntry *second = military ? TYPES : MIL_TYPES;
    size_t secondCount = military ? std::size(TYPES) : std::size(MIL_TYPES);

    const char *hit = findType(first, firstCount, icaoCode);
    if (hit == nullptr) {
        hit = findType(second, secondCount, icaoCode);
    }
    return hit != nullptr ? String(hit) : icaoCode;
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
