#include "conversions.h"

#include <cmath>

namespace CamSim::Math {

double deg_to_rad(const double angle_deg)
{
    return angle_deg * M_PI / 180.0;
}

double rad_to_deg(const double angle_rad)
{
    return angle_rad * 180.0 * M_1_PI;
}

std::tuple<const double, const double, const double> lla_to_geocentric_rad(
    const double lattitude_rad,
    const double longitude_rad,
    const double altitude_m)
{
    const double radius_of_curvature = Model::WGS84Ellipsoid::radius_of_curvature(lattitude_rad);
    const double p = (radius_of_curvature + altitude_m) * std::cos(lattitude_rad);
    const double z =
        (radius_of_curvature * (1 - Model::WGS84Ellipsoid::eccentricity_squared) + altitude_m) *
        std::sin(lattitude_rad);
    const double radius_m = std::sqrt(p * p + z * z);
    const double phi_rad = std::asin(z / radius_m);

    return std::tuple<const double, const double, const double>{longitude_rad, phi_rad, radius_m};
}

std::tuple<const double, const double, const double> lla_to_geocentric_deg(
    const double lattitude_deg,
    const double longitude_deg,
    const double altitude_m)
{
    const double lattitude_rad = deg_to_rad(lattitude_deg);
    const double longitude_rad = deg_to_rad(longitude_deg);

    const auto [theta_rad, phi_rad, radius_m] =
        lla_to_geocentric_rad(lattitude_rad, longitude_rad, altitude_m);

    const double phi_deg = rad_to_deg(phi_rad);

    return std::tuple<const double, const double, const double>{longitude_deg, phi_deg, radius_m};
}

}
