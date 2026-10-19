#pragma once

// The geoid's height - mean sea level - above the WGS84 ellipsoid at (lat,
// lon), in metres: what a GNSS height, above the ellipsoid, is less to be
// above the sea. From the EGM96 model, read between the degrees about the
// point; within a few metres of it. See tools/gen_geoid.py.
float geoidM(double lat, double lon);
