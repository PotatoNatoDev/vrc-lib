#pragma once

#include "main.h"
#include <math.h>

// wheel circumference
const double wheelCircumference = 2.75 * M_PI;
const double drivetrainRatio = 60.0/48.0; // driven gear:driving gear

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

// 6 motor drive train initialization
inline pros::MotorGroup lmotors({-9, -12, -11}); // left drivetrain motors
inline pros::MotorGroup rmotors({10, 19, 20}); // right drivetrain motors

// subsystem definitions
inline pros::Motor intake(7);

// pneumatics definitions
inline pros::adi::DigitalOut piston1('A');
inline pros::adi::DigitalOut piston2('B');

// sensor definitions
inline pros::Imu inertial(2);
inline pros::adi::DigitalIn limit_switch('C');
