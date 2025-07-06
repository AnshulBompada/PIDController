//
// Created by User on 7/5/2025.
//

#include "MathUtils.h"
#include <cmath>


double MathUtils::lowPassFilter(double value, double filter) {
    return (value < 0) ? -std::max(value, filter) : std::max(value, filter);
}

double MathUtils::epsilonEquals(double a, double b, double epsilon) {
    return std::abs(a - b) <= epsilon;
}

double MathUtils::epsilonEquals(double a, double b) {
    return MathUtils::epsilonEquals(a, b, 10e-16);
}

double MathUtils::inputModulus(double input, double minInput, double maxInput) {
    double modulus = maxInput - minInput;

    // Wrap input if it's above the maximum input
    int numMax = (int) ((input - minInput) / modulus);
    input -= numMax * modulus;

    // Wrap input if it's below the minimum input
    int numMin = (int) ((input - maxInput) / modulus);
    input -= numMin * modulus;

    return input;
}