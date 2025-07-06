//
// Created by User on 7/5/2025.
//

#ifndef MATHUTILS_H
#define MATHUTILS_H



namespace  MathUtils {
    /* Basically just a clamp*/
    double lowPassFilter(double value, double filter);

    double epsilonEquals(double a, double b, double epsilon);

    double epsilonEquals(double a, double b);

    double inputModulus(double input, double minInput, double maxInput);
};



#endif //MATHUTILS_H
