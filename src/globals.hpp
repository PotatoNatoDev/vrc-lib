#pragma once

#include "main.h"
#include <math.h>

// wheel circumference
const double wheelCircumference = 2.75 * M_PI;
const double drivetrainRatio = 48.0/60.0; // 48:60 -> 48/60 = 0.8

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

// 6 motor drive train initialization
inline pros::MotorGroup lmotors({1, 2, 3}); // left drivetrain motors
inline pros::MotorGroup rmotors({-4, -5, -6}); // right drivetrain motors

// subsystem definitions
inline pros::Motor intake(7);

// pneumatics definitions
inline pros::adi::DigitalOut piston1('A');
inline pros::adi::DigitalOut piston2('B');

// sensor definitions
inline pros::Imu inertial(8);
inline pros::adi::DigitalIn limit_switch('C');
