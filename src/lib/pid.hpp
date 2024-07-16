#pragma once

#include "main.h"

class PID {
    public:
    // constructor
    PID(float kP, float kI, float kD):
        kP(kP),
        kI(kI),
        kD(kD) {}

    // update pid loop
    float update(const float error) {
        // integral
        integral += error;

        // derivative
        const float derivative = error - prevError;
        prevError = error;

        // calculate output
        return error * kP + integral * kI + derivative * kD;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }

    protected:
    float kP;
    float kI;
    float kD;
    float integral = 0;
    float prevError = 0;

};