//
// Created by User on 7/4/2025.
//

#include "PIDController.h"

#include <stddef.h>
#include <cmath>
#include "MathUtils.h"

PIDController::PIDController(double pP, double pI, double pD, double pDT) {
    kP = pP;
    kI = pI;
    kD = pD;
    dt = pDT;
}

double PIDController::getkP() {
    return  kP;
}

void PIDController::setkP(double pP) {
    kP = pP;
}

double PIDController::getkI() {
    return  kI;
}

void PIDController::setkI(double pI) {
    kI = pI;
}

double PIDController::getkD() {
    return kD;
}

void PIDController::setkD(double pD) {
    kD = pD;
}

double PIDController::getDt() {
    return dt;
}

void PIDController::setDt(double pDt) {
    dt = pDt;
}

void PIDController::setIRange(double pIRange) {
    setIRange(-pIRange, pIRange);
}

void PIDController::setIRange(double pINegativeR, double pIPositiveR) {
    setINegativeRange(pINegativeR);
    setIPositiveRange(pIPositiveR);
}

double PIDController::getIPositiveRange() {
    return iPositiveRange;
}

void PIDController::setIPositiveRange(double pIPositiveR) {
    iPositiveRange = pIPositiveR;
}

double PIDController::getINegativeRange() {
    return iNegativeRange;
}

void PIDController::setINegativeRange(double pINegativeR) {
    iNegativeRange = pINegativeR;
}

bool PIDController::getInIRange() {
    return getError() > getINegativeRange() && getError() <= getIPositiveRange();
}

/* INTEGRATION ACCUMULATION LIMITS */
void PIDController::setAccumLimits(double pAccumLimit) {
    setAccumLimits(-pAccumLimit, pAccumLimit);
}

void PIDController::setAccumLimits(double pINegativeAL, double pIPositiveAL) {
    setINegativeAccumLimit(-pINegativeAL);
    setIPositiveAccumLimit(pIPositiveAL);
}

double PIDController::getIPositiveAccumLimit() {
    return iPositiveAccumLimit;
}

void PIDController::setIPositiveAccumLimit(double pIPositiveAL) {
    iPositiveAccumLimit = pIPositiveAL;
}

double PIDController::getINegativeAccumLimit() {
    return  iNegativeAccumLimit;
}

void PIDController::setINegativeAccumLimit(double pINegativeAL) {
    iNegativeAccumLimit = pINegativeAL;
}

double PIDController::getIAccumulation() {
    return iAccum;
}

void PIDController::reset() {
    iAccum = 0;
    prevError = NAN;
    rOCTolerance = 0;
}

/* D-FILTER */
double PIDController::getDFilter() {
    return dFilter;
}

void PIDController::setDFilter(double pDFilter) {
    dFilter = pDFilter;
}

/* CONTINOUS WRAP */
bool PIDController::getContinuousWrap() {
    return continousWrap;
}

double PIDController::getLowerDiscontinuity() {
    return lowerDiscontinuity;
}

double PIDController::getHigherDiscontinuity() {
    return  higherDiscontinuity;
}

void PIDController::enableContinuousWrap(double pLowerDisconinuity, double pHigherDiscontinuity) {
    continousWrap = true;
    lowerDiscontinuity = pLowerDisconinuity;
    higherDiscontinuity = pHigherDiscontinuity;
}

void PIDController::disableContinuousWrap() {
    continousWrap = false;
}


double PIDController::getReference() {
    return  reference;
}

void PIDController::setReference(double pReference) {
    reference = pReference;
}

double PIDController::getMeasure() {
    return measure;
}

void PIDController::setMeasure(double pMeasure) {
    measure = pMeasure;
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
    return errorTolerance;
}

void PIDController::setErrorTolerance(double pTol) {
    errorTolerance = pTol;
}

double PIDController::getROCTolerance() {
    return rOCTolerance;
}

void PIDController::setROCTolerance(double pTol) {
    rOCTolerance = pTol;
}

bool PIDController::inErrorTolerance() {
    return std::abs(getError()) < getErrorTolerance();
}

bool PIDController::inROCTolerance() {
    return std::abs(getROCError()) < getROCTolerance();
}

double PIDController::calculate(double pReference, double pMeasure) {
    setReference(pReference);
    setMeasure(pMeasure);
    return calculate();
}

double PIDController::calculate() {
    if(std::isnan(prevError)) {
        prevError = getError();
    }

    double pOutput = getkP() * getError();

    if(
        !MathUtils::epsilonEquals(iPositiveRange,0)
        ||
        !MathUtils::epsilonEquals(iNegativeRange,0)) {
        if(getInIRange()) iAccum += getError() * dt;
        else iAccum = 0;
    } else iAccum += getError() * dt;

    if(
        (!MathUtils::epsilonEquals(iPositiveAccumLimit, 0)
        ||
        !MathUtils::epsilonEquals(iNegativeAccumLimit, 0))
        &&
        !MathUtils::epsilonEquals(getkI(), 0)) {
        if(getIAccumulation() < getINegativeAccumLimit())
            iAccum = getINegativeAccumLimit() / getkI();
        else if(getIAccumulation() > getIPositiveAccumLimit())
            iAccum = getIPositiveAccumLimit() / getkI();
    }

    double iError = iAccum * getkI();

    prevError = getError();

    double errorROC = (getError() - prevError) / dt;
    if(dFilter != 0) {
        errorROC = MathUtils::lowPassFilter(errorROC, getDFilter());
    }

    rOCError = errorROC;

    double dOutput = getkD() * errorROC;

    return pOutput + iError + dOutput;
}