#include "opcontrol.hpp"
#include "../globals.hpp"
#include "pid.hpp"
#include "custom_math.hpp"
#include "main.h"
#include "pros/rtos.h"

// normal arcade drivetrain control
void arcade_drive_regular() {
    // get controller joysticks
    int yAxis = master.get_analog(ANALOG_LEFT_Y);
    int xAxis = master.get_analog(ANALOG_RIGHT_X);
    
    // move drivetrain
    lmotors.move(yAxis + xAxis);
    rmotors.move(yAxis - xAxis);
}

PID drivetrainPID(2,0,0); // tune this.
double drivetrainForwardDirection;
float bufferTime;
// arcade drivetrain control with pid to make it drive straight
void arcade_drive_pid(float buffer) {
    int yAxis = master.get_analog(ANALOG_LEFT_Y);
    int xAxis = master.get_analog(ANALOG_RIGHT_X);
    
    // if there is movement on the xAxis, move like normal arcade drive
    // if theres no movement on the xAxis, still run this case for a few milliseconds (determined by float buffer) to allow the robot's momentum to continue
    if(abs(xAxis) > 1 || pros::c::millis() < bufferTime) {
        // move drivetrain normally if we are turning
        lmotors.move(yAxis + xAxis);
        rmotors.move(yAxis - xAxis);
        // reset drivetrain pid and make target heading current heading
        drivetrainForwardDirection = inertial.get_heading();
        drivetrainPID.reset();
        if(abs(xAxis) > 1) bufferTime = pros::c::millis() + buffer; // only add to buffer time if we're actually turning
    } else {
        // if we are driving straight enable PID to correct for error
        double error = closestAngle(drivetrainForwardDirection, inertial.get_heading());
        if(std::isnan(error)) error = 0; // prevents an error where at the start error will be NaN
        double turnCorrection = drivetrainPID.update(error);
        lmotors.move(yAxis + turnCorrection);
        rmotors.move(yAxis - turnCorrection);
    }
}

// code to initalization opcontrol properly
void opcontrol_init() {
    lmotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rmotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    drivetrainForwardDirection = inertial.get_heading();
    drivetrainPID.reset();
}

// driver control code
void opcontrol_loop() {
    
    // driving controls
    //arcade_drive_regular();
    arcade_drive_pid(100);
    pros::delay(20);
}