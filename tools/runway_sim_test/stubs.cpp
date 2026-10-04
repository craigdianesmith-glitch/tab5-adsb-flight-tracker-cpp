#include "airports.h"
#include "adsb_client.h"
float haversineNm(double lat1, double lon1, double lat2, double lon2) {
    double r = M_PI / 180, dlat = (lat2 - lat1) * r, dlon = (lon2 - lon1) * r;
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1 * r) * cos(lat2 * r) * sin(dlon / 2) * sin(dlon / 2);
    return 2 * 3440.065 * asin(sqrt(a));
}
const std::vector<Airport> &airportsWithin(double lat, double lon, float rangeNm) {
    static std::vector<Airport> v{{"GLA", 55.8719, -4.43306}};
    static std::vector<Airport> none;
    return haversineNm(lat, lon, 55.8719, -4.43306) <= rangeNm ? v : none;
}
