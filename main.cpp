#include <iostream>
#include "PIDController.h"

int main() {
    PIDController controller = PIDController(5.0, 5.0, 5.0, 0.02);
    controller.setErrorTolerance(1);
    controller.enableContinuousWrap(0, 100);
    double measure = 0.0;
    while (measure < 510) {
        std:: cout << controller.calculate(500, measure) << std::endl;
        measure += 10;
    }

    return 0;
}
