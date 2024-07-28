#pragma once

#include "main.h"

// taken from https://orange-depot.github.io/posts/Basic-PID/

class PID {
    public:
    // PID constructor
    PID(double kP, double kI, double kD):
        kP(kP),
        kI(kI),
        kD(kD) {}

    // update pid loop
    double update(const double error) {
        // integral
        integral += error;

        // derivative
        const double derivative = error - prevError;
        prevError = error;

        // calculate output
        return error * kP + integral * kI + derivative * kD;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }

    protected:
    double kP;
    double kI;
    double kD;
    double integral = 0;
    double prevError = 0;
};