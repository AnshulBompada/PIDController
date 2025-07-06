//
// Created by User on 7/4/2025.
//

#include "PIDController.h"

#include <stddef.h>
#include <cmath>
#include "MathUtils.h"

PIDController::PIDController(double pP, double pI, double pD, double pDT) {
    PIDController::kP = pP;
    PIDController::kI = pI;
    PIDController::kD = pD;
    PIDController::dt = pDT;
}

double PIDController::getkP() {
    return  PIDController::kP;
}

void PIDController::setkP(double pP) {
    PIDController::kP = pP;
}

double PIDController::getkI() {
    return  PIDController::kI;
}

void PIDController::setkI(double pI) {
    PIDController::kI = pI;
}

double PIDController::getkD() {
    return PIDController::kD;
}

void PIDController::setkD(double pD) {
    PIDController::kD = pD;
}

double PIDController::getDt() {
    return PIDController::kD;
}

void PIDController::setDt(double pDt) {
    PIDController::dt = pDt;
}

void PIDController::setIRange(double pIRange) {
    PIDController::setIRange(-pIRange, pIRange);
}

void PIDController::setIRange(double pINegativeR, double pIPositiveR) {
    PIDController::setINegativeRange(pINegativeR);
    PIDController::setIPositiveRange(pIPositiveR);
}

double PIDController::getIPositiveRange() {
    return PIDController::iPositiveRange;
}

void PIDController::setIPositiveRange(double pIPositiveR) {
    PIDController::iPositiveRange = pIPositiveR;
}

double PIDController::getINegativeRange() {
    return PIDController::iNegativeRange;
}

void PIDController::setINegativeRange(double pINegativeR) {
    PIDController::iNegativeRange = pINegativeR;
}

double PIDController::getInIRange() {
    return getError() > getINegativeRange() && getError() <= getIPositiveRange();
}

/* INTEGRATION ACCUMULATION LIMITS */
void PIDController::setAccumLimits(double pAccumLimit) {
    PIDController::setAccumLimits(-pAccumLimit, pAccumLimit);
}

void PIDController::setAccumLimits(double pINegativeAL, double pIPositiveAL) {
    PIDController::setINegativeAccumLimit(-pINegativeAL);
    PIDController::setIPositiveAccumLimit(pIPositiveAL);
}

double PIDController::getIPositiveAccumLimit() {
    return PIDController::iPositiveAccumLimit;
}

void PIDController::setIPositiveAccumLimit(double pIPositiveAL) {
    iPositiveAccumLimit = pIPositiveAL;
}

double PIDController::getINegativeAccumLimit() {
    return  PIDController::iNegativeAccumLimit;
}

void PIDController::setINegativeAccumLimit(double pINegativeAL) {
    iPositiveAccumLimit = pINegativeAL;
}

double PIDController::getIAccumulation() {
    return PIDController::iAccum;
}

void PIDController::reset() {
    PIDController::iAccum = 0;
    PIDController::prevError = NAN;
    PIDController::rOCTolerance = 0;
}

/* D-FILTER */
double PIDController::getDFilter() {
    return PIDController::dFilter;
}

void PIDController::setDFilter(double pDFilter) {
    PIDController::dFilter = pDFilter;
}

/* CONTINOUS WRAP */
bool PIDController::getContinuousWrap() {
    return PIDController::continousWrap;
}

double PIDController::getLowerDiscontinuity() {
    return PIDController::lowerDiscontinuity;
}

double PIDController::getHigherDiscontinuity() {
    return  PIDController::higherDiscontinuity;
}

void PIDController::enableContinuousWrap(double pLowerDisconinuity, double pHigherDiscontinuity) {
    PIDController::continousWrap = true;
    PIDController::lowerDiscontinuity = pLowerDisconinuity;
    PIDController::higherDiscontinuity = pHigherDiscontinuity;
}

void PIDController::disableContinuousWrap() {
    PIDController::continousWrap = false;
}


double PIDController::getReference() {
    return  PIDController::reference;
}

void PIDController::setReference(double pReference) {
    PIDController::reference = pReference;
}

double PIDController::getMeasure() {
    return  PIDController::measure;
}

void PIDController::setMeasure(double pMeasure) {
    PIDController::measure = pMeasure;
}

double PIDController::getError() {
    if (continousWrap) {
        double errorBound = (higherDiscontinuity - lowerDiscontinuity) / 2.0;
        return MathUtils::inputModulus(reference - measure, -errorBound, errorBound);
    }

    return reference - measure;
}

double PIDController::getROCError() {
    return rOCError;
}

double PIDController::getErrorTolerance() {
    return PIDController::errorTolerance;
}

void PIDController::setErrorTolerance(double pTol) {
    PIDController::errorTolerance = pTol;
}

double PIDController::getROCTolerance() {
    return PIDController::rOCTolerance;
}

void PIDController::setROCTolerance(double pTol) {
    PIDController::rOCTolerance = pTol;
}

bool PIDController::inErrorTolerance() {
    return std::abs(PIDController::getError()) < getErrorTolerance();
}

bool PIDController::inROCTolerance() {
    return std::abs(PIDController::getError()) < getErrorTolerance();
}

double PIDController::calculate(double pReference, double pMeasure) {
    PIDController::setReference(pReference);
    PIDController::setMeasure(pMeasure);
    return calculate();
}

double PIDController::calculate() {
    if(PIDController::prevError == NAN) {
        prevError = getError();
    }


    double pOutput = getkP() * getError();

    if(
        !MathUtils::epsilonEquals(PIDController::iPositiveRange,0)
        ||
        !MathUtils::epsilonEquals(PIDController::iNegativeRange,0)) {
        if(getInIRange()) PIDController::iAccum += getError() * dt;
        else PIDController::iAccum = 0;
    } else PIDController::iAccum += getError() * dt;

    if(
        (!MathUtils::epsilonEquals(PIDController::iPositiveAccumLimit, 0)
        ||
        !MathUtils::epsilonEquals(PIDController::iNegativeAccumLimit, 0))
        &&
        !MathUtils::epsilonEquals(PIDController::getkI(), 0)) {
        if(getIAccumulation() < getINegativeAccumLimit())
            PIDController::iAccum = getINegativeAccumLimit() / PIDController::getkI();
        else if(getIAccumulation() > getIPositiveAccumLimit())
            PIDController::iAccum = getIPositiveAccumLimit() / PIDController::getkI();
    }

    double iError = iAccum * PIDController::getkI();

    prevError = getError();

    double errorROC = (getError() - prevError) / dt;
    if(dFilter != 0) {
        errorROC = MathUtils::lowPassFilter(errorROC, getDFilter());
    }

    PIDController::rOCError = errorROC;

    double dOutput = PIDController::getkD() * errorROC;

    return pOutput + iError + dOutput;
}