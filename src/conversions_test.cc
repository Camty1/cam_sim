#include "conversions.h"

#include <gtest/gtest.h>
#include <vector>

namespace CamSim::Conversions {

TEST(lla_to_geocentric_rad_test, lattitude_comparison)
{
}

TEST(lla_to_geocentric_deg_test, latitude_comparison)
{
    const double f = 0.00335281066474748071984552861852;
    const std::vector<double> latitude_deg_values = {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0};

    for (const double& lattitude_deg : latitude_deg_values)
    {
        const double expected_phi_deg =
            rad_to_deg(std::atan((1 - f * f) * std::tan(deg_to_rad(lattitude_deg))));
        const auto& [theta_deg, test_phi_deg, radius_m] = lla_to_geocentric_deg(lattitude_deg, 6.7, 0.0);

        ASSERT_EQ(expected_phi_deg, test_phi_deg);
    }
}

}
