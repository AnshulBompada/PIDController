#include <iostream>
#include "PIDController.h"
#include "MathUtils.h"

int main() {
    // PIDController controller = PIDController(5.0, 5.0, 5.0, 0.02);
    // controller.setErrorTolerance(1);
    // controller.enableContinuousWrap(0, 100);
    // double measure = 0.0;
    // while (measure < 510) {
    //     std:: cout << controller.calculate(500, measure) << std::endl;
    //     measure += 10;
    // }

    std::cout << "Case 1: " << MathUtils::inputModulus(-1260, -180, 180) << std::endl;
    std::cout << "Case 2: " << MathUtils::inputModulus(-900, -180, 180) << std::endl;
    std::cout << "Case 3: " << MathUtils::inputModulus(-540, -180, 180) << std::endl;
    std::cout << "Case 4: " << MathUtils::inputModulus(-180, -180, 180) << std::endl;
    std::cout << "Case 5: " << MathUtils::inputModulus(180, -180, 180) << std::endl;
    std::cout << "Case 6: " << MathUtils::inputModulus(540, -180, 180) << std::endl;
    std::cout << "Case 7: " << MathUtils::inputModulus(900, -180, 180) << std::endl;
    std::cout << "Case 8: " << MathUtils::inputModulus(1260, -180, 180) << std::endl;

    return 0;
}
