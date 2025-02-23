#ifndef MATH_H
#define MATH_H
#include <cmath>
#include <tuple>

#include "wgs84.h"

namespace CamSim::Math {

double factorial(int n);

double semi_normalized_legendre(const int l, const int m, const double x);

double semi_normalized_legendre_sin_deriv(const int l, const int m, const double x);

}

#endif
