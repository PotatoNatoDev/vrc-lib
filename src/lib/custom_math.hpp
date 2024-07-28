#pragma once

#include "main.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// converts a rotation value from the imu (unbounded) to a heading value (0-360)
inline double rotationToHeading(double rotation) {
    // limits rotation to (-360-360)
    double heading = fmodf(rotation, 360);

    // if its a negative value, add 360 to make it between (0-360)
    if (heading < 0) {
        heading += 360;
    }

    return heading;
}

// Finds the shortest angle to get from one angle to another. Positive indicates right, negative indicates left.
inline double closestAngle(double target, double actual) {
    // convert the paramaters to a heading
    double targetHeading = rotationToHeading(target);
    double actualHeading = rotationToHeading(actual);
    
    // find the difference between the target and current heading
    double headingDifference = targetHeading - actualHeading;

    // if the difference is more than 180 degrees, we rotate the other way
    if(fabs(headingDifference) > 180) {
        headingDifference -= sgn(headingDifference) * 360;
    }

    return headingDifference;
}