#ifndef SPHERICAL_HARMONIC_MODELS_H
#define SPHERICAL_HARMONIC_MODELS_H
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <gsl/gsl_sf_legendre.h>
#include <gsl/gsl_vector.h>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <vector>

#include "math.h"
#include "time.h"

namespace CamSim::Model {

struct SphericalHarmonicCoefficients
{
    int l;
    int m;
    double g;
    double h;
    double g_dot;
    double h_dot;

    friend std::ostream& operator<<(std::ostream& os, const SphericalHarmonicCoefficients& coeff)
    {
        os << std::string("SphericalHarmonicCoefficients(l = ") << std::to_string(coeff.l)
           << std::string(", m = ") << std::to_string(coeff.m) << std::string(", g = ")
           << std::to_string(coeff.g) << std::string(", h = ") << std::to_string(coeff.h)
           << std::string(", g_dot = ") << std::to_string(coeff.g_dot) << std::string(", h_dot = ")
           << std::to_string(coeff.h_dot) << std::string(")") << std::endl;

        return os;
    }

    double get_g(const double decimal_year, const double decimal_year_epoch) const
    {
        std::cout << "g: " << g << " " << g_dot << " " << decimal_year << " " << decimal_year_epoch
                  << std::endl;
        return g + g_dot * (decimal_year - decimal_year_epoch);
    }

    double get_h(const double decimal_year, const double decimal_year_epoch) const
    {
        std::cout << "h: " << h << " " << h_dot << " " << decimal_year << " " << decimal_year_epoch
                  << std::endl;
        return h + h_dot * (decimal_year - decimal_year_epoch);
    }
};

class SphericalHarmonicModel
{
protected:
    const double epoch = 0;
    const int max_order = 0;

    void load_coefficients(const std::string& path);
    double potential_value(int l, int m, double g, double h);

    std::vector<std::vector<SphericalHarmonicCoefficients>> coefficients;
    double geomagnetic_radius = 6371200.0;
};

class WorldMagneticModel : public SphericalHarmonicModel
{
public:
    WorldMagneticModel();
    double get_potential(
        const double theta,
        const double phi,
        const double r,
        const Time::Timestamp timestamp,
        const int order) const;

    double get_x_prime(
        const double theta,
        const double phi,
        const double radius,
        const Time::Timestamp timestamp,
        const int order) const;

    double get_y_prime(
        const double theta,
        const double phi,
        const double radius,
        const Time::Timestamp timestamp,
        const int order) const;

    double get_z_prime(
        const double theta,
        const double phi,
        const double radius,
        const Time::Timestamp timestamp,
        const int order) const;

protected:
    const double epoch = 2020.0;
    const int max_order = 12;

    void load_coefficients();
};

class EarthGravitationalModel : public SphericalHarmonicModel
{
public:
    double get_potential(
        const double theta,
        const double phi,
        const double r,
        const Time::Timestamp timestamp,
        const int order);

protected:
    const double epoch = 2008.0;
    const int max_order = 2190;

    void load_coefficients();
};

}

#endif
