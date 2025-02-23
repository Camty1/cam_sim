#ifndef CONVERSIONS_H
#define CONVERSIONS_H
#include <cmath>
#include <tuple>

#include "wgs84.h"

namespace CamSim::Conversions {

double deg_to_rad(const double angle_deg);

double rad_to_deg(const double angle_rad);

std::tuple<const double, const double, const double> lla_to_geocentric_rad(
    const double lattitude_rad,
    const double longitude_rad,
    const double altitude_m);

std::tuple<const double, const double, const double> lla_to_geocentric_deg(
    const double lattitude_deg,
    const double longitude_deg,
    const double altitude_m);

}

#endif
