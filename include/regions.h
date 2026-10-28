#pragma once

#include <stdint.h>

// The places the clock and the words treat apart - the UK's zone for home in
// Scotland and England, and the mainland United States' (clock_zone.h); a
// height read aloud as a flight level from 18,000ft over the United States,
// rather than 6,000ft (sound.cpp) - and everywhere else. From Natural Earth's
// map units, by tools/gen_regions.py.
enum class Region : uint8_t { ELSEWHERE, SCOTLAND, ENGLAND, USA };

// Which of them the airport with IATA code `code` is in.
Region airportRegion(const char *code);

// Which of them something at (lat, lon) is about: the one the nearest airport
// within REGION_AIRPORT_NM is in - whose airspace it is most likely in, over
// the land or off the coast - or, with none that near, the one whose outline
// it is inside.
Region regionNear(float lat, float lon);
