//
// Created by User on 7/4/2025.
//

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "../../Program Files/JetBrains/CLion 2024.1.3/bin/mingw/x86_64-w64-mingw32/include/math.h"


class PIDController {
public:
    PIDController(double pP, double pI, double pD, double pDT);

    double getkP();
    void setkP(double pP);

    double getkI();
    void setkI(double pI);

    double getkD();
    void setkD(double pD);

    double getDt();
    void setDt(double pDt);

    /* INTEGRATION RANGE */
    void setIRange(double pIRange);
    void setIRange(double pINegativeR, double pIPositiveR);

    double getIPositiveRange();
    void setIPositiveRange(double pIPositiveR);

    double getINegativeRange();
    void setINegativeRange(double pINegativeR);

    double getInIRange();

    /* INTEGRATION ACCUMULATION LIMITS(WIND-UP) */
    void setAccumLimits(double accumLimit);
    void setAccumLimits(double pINegativeAL, double pIPositiveAL);

    double getIPositiveAccumLimit();
    void setIPositiveAccumLimit(double pIPositiveAL);

    double getINegativeAccumLimit();
    void setINegativeAccumLimit(double pINegativeAL);

    double getIAccumulation();
    void reset();

    /* D-FILTER */
    double getDFilter();
    void setDFilter(double pDFilter);

    /* CONTINUOS WRAP */
    bool getContinuousWrap();
    double getLowerDiscontinuity();
    double getHigherDiscontinuity();

    void enableContinuousWrap(double pLowerDisconinuity, double pHigherDiscontinuity);
    void disableContinuousWrap();

    double getReference();
    void setReference(double pReference);

    double getMeasure();
    void setMeasure(double pMeasure);

    double getError();
    double getROCError();

    double getErrorTolerance();
    void setErrorTolerance(double pTol);

    double getROCTolerance();
    void setROCTolerance(double pTol);

    bool inErrorTolerance();
    bool inROCTolerance();

    double calculate(double pReference, double pMeasure);
    double calculate();
private:
    double kP;

    double kI;
    double iPositiveRange = 0;
    double iNegativeRange= 0;
    double iPositiveAccumLimit = 0;
    double iNegativeAccumLimit = 0;

    double kD;
    double dFilter = 0;

    double dt;

    bool continousWrap = false;
    double lowerDiscontinuity = 0.0;
    double higherDiscontinuity = 0.0;

    double reference = 0;
    double measure = 0;
    double prevError = NAN;

    double errorTolerance = 0;
    double rOCTolerance = 0;

    double rOCError = 0;
    double iAccum = 0;
};



#endif //PIDCONTROLLER_H
