#include "math.h"

#include <gsl/gsl_sf_legendre.h>

namespace CamSim::Math {

double factorial(const int n)
{
    double out = 1.0;
    for (int i = 2; i <= n; i++)
    {
        out *= (double)i;
    }

    return out;
}

double semi_normalized_legendre(const int l, const int m, const double x)
{
    double legendre = gsl_sf_legendre_Plm(l, m, x);

    if (m == 0)
    {
        return legendre;
    }

    return std::sqrt(2.0 * factorial(l - m) / factorial(l + m)) * legendre;
}

double semi_normalized_legendre_sin_deriv(const int l, const int m, const double x)
{
    double sin_x = std::sin(x);

    return (l + 1) * std::tan(x) * semi_normalized_legendre(l, m, sin_x) -
           std::sqrt((l + 1) * (l + 1) - m * m) * semi_normalized_legendre(l + 1, m, sin_x) / std::cos(x);
}

}
