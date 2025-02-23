#ifndef WGS84_H
#define WGS84_H
#include <cmath>

namespace CamSim::Model {

class WGS84Ellipsoid
{
public:
    static constexpr double semi_major_axis = 6378137.0;
    static constexpr double inv_flattening = 298.257223563;
    static constexpr double flattening = 1 / inv_flattening;
    static constexpr double eccentricity_squared = flattening * (2 - flattening);

    static double radius_of_curvature(const double lattitude_rad)
    {
        const double sin_lattitude_rad = std::sin(lattitude_rad);
        return semi_major_axis /
               std::sqrt(1 - eccentricity_squared * sin_lattitude_rad * sin_lattitude_rad);
    }
};

}

#endif
