#include <iostream>

#include "spherical_harmonic_models.h"

int main(int argc, char** argv)
{
    // CamSim::Model::WorldMagneticModel wmm{};
    // const CamSim::Time::Timestamp timestamp = CamSim::Time::Timestamp::from_decimal_year(2022.5);
    // double theta = 4.1887902048;
    // double phi = -1.3951289589;
    // double radius = 6457402.3484473705;

    // double x_prime = wmm.get_x_prime(theta, phi, radius, timestamp, 12);
    // std::cout << x_prime << std::endl;
    // double y_prime = wmm.get_y_prime(theta, phi, radius, timestamp, 12);
    // double z_prime = wmm.get_z_prime(theta, phi, radius, timestamp, 12);
    // std::cout << "(x', y', z') = (" << x_prime << ", " << y_prime << ", " << z_prime << ")"
    //           << std::endl;
    //
    for (int i = 0; i < 201; i++)
    {
        const double x = i / 100.0 - 1.0;
        for (int m = 0; m < 6; m++)
        {
            double l = CamSim::Math::semi_normalized_legendre(5, m, x);
            std::cout << std::setw(18) << std::setprecision(12) << l << " ";
        }
        std::cout << std::endl;
    }
}
