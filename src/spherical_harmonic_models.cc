#include "spherical_harmonic_models.h"

#include <iomanip>

namespace CamSim::Model {

void SphericalHarmonicModel::load_coefficients(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open coefficients file at '" + path + "'");
    }

    int l, m;
    double g, h, g_dot, h_dot;
    while (file >> l >> m >> g >> h >> g_dot >> h_dot)
    {
        coefficients[l].push_back(SphericalHarmonicCoefficients{
            .l = l, .m = m, .g = g, .h = h, .g_dot = g_dot, .h_dot = h_dot});
    }
}

WorldMagneticModel::WorldMagneticModel()
{
    coefficients.resize(max_order + 1);
    load_coefficients();
}

void WorldMagneticModel::load_coefficients()
{
    SphericalHarmonicModel::load_coefficients("/home/cwolfe/sb/projects/sim/coeffs/WMM.COF");
}

double WorldMagneticModel::get_potential(
    const double theta,
    const double phi,
    const double radius,
    const Time::Timestamp timestamp,
    const int order) const
{
    double potential = 0.0;
    const double decimal_year = timestamp.get_decimal_year();

    for (int l = 1; l <= order; l++)
    {
        double inner = 0.0;
        for (int m = 0; m <= l; m++)
        {
            const SphericalHarmonicCoefficients& coeffs = coefficients[l][m];
            const double g = coeffs.get_g(decimal_year, epoch);
            const double h = coeffs.get_h(decimal_year, epoch);

            std::cout << "g: " << g << ", h: " << h << std::endl;
            inner += (g * std::cos((double)m * theta) + h * std::sin((double)m * theta)) *
                     Math::semi_normalized_legendre(l, m, std::sin(phi));
        }
        potential += std::pow(geomagnetic_radius / radius, (double)(l + 1)) * inner;
    }

    return geomagnetic_radius * potential;
}

double WorldMagneticModel::get_x_prime(
    const double theta,
    const double phi,
    const double radius,
    const Time::Timestamp timestamp,
    const int order) const
{
    double x_prime = 0.0;
    const double decimal_year = timestamp.get_decimal_year();

    for (int l = 1; l <= order; l++)
    {
        double inner = 0.0;
        for (int m = 0; m <= l; m++)
        {
            std::cout << l << ", " << m << ": " << std::endl;
            const SphericalHarmonicCoefficients& coeffs = coefficients[l][m];
            const double g = coeffs.get_g(decimal_year, epoch);
            const double h = coeffs.get_h(decimal_year, epoch);
            std::cout << "(l, m) = (" << l << ", " << m << "), g = " << std::setprecision(20) << g
                      << ", h = " << h << std::endl;
            inner += (g * std::cos((double)m * theta) + h * std::sin((double)m * theta)) *
                     Math::semi_normalized_legendre_sin_deriv(l, m, phi);
        }
        x_prime += std::pow(geomagnetic_radius / radius, (double)(l + 2));
    }

    return -x_prime;
}

double WorldMagneticModel::get_y_prime(
    const double theta,
    const double phi,
    const double radius,
    const Time::Timestamp timestamp,
    const int order) const
{
    double y_prime = 0.0;
    const double decimal_year = timestamp.get_decimal_year();

    for (int l = 1; l <= order; l++)
    {
        double inner = 0.0;
        for (int m = 0; m <= l; m++)
        {
            std::cout << l << ", " << m << ": " << std::endl;
            const SphericalHarmonicCoefficients& coeffs = coefficients[l][m];
            const double g = coeffs.get_g(decimal_year, epoch);
            const double h = coeffs.get_h(decimal_year, epoch);
            inner += (double)m *
                     (g * std::sin((double)m * theta) - h * std::cos((double)m * theta)) *
                     Math::semi_normalized_legendre(l, m, std::sin(phi));
        }
        y_prime += std::pow(geomagnetic_radius / radius, (double)(l + 2));
    }

    return y_prime / std::cos(phi);
}

double WorldMagneticModel::get_z_prime(
    const double theta,
    const double phi,
    const double radius,
    const Time::Timestamp timestamp,
    const int order) const
{
    double z_prime = 0.0;
    const double decimal_year = timestamp.get_decimal_year();

    for (int l = 1; l <= order; l++)
    {
        double inner = 0.0;
        for (int m = 0; m <= l; m++)
        {
            std::cout << l << ", " << m << ": " << std::endl;
            const SphericalHarmonicCoefficients& coeffs = coefficients[l][m];
            const double g = coeffs.get_g(decimal_year, epoch);
            const double h = coeffs.get_h(decimal_year, epoch);
            inner += (g * std::cos((double)m * theta) + h * std::sin((double)m * theta)) *
                     Math::semi_normalized_legendre(l, m, std::sin(phi));
        }
        z_prime += (double)(l + 1) * std::pow(geomagnetic_radius / radius, (double)(l + 2));
    }

    return -z_prime;
}

}
