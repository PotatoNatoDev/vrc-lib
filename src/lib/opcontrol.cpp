#include "opcontrol.hpp"
#include "../globals.hpp"
#include "pid.hpp"
#include "custom_math.hpp"
#include "main.h"


// code to initalization opcontrol properly
void opcontrol_init() {

}

// normal arcade drivetrain control
void arcade_drive_regular() {
    // get controller joysticks
    int yAxis = master.get_analog(ANALOG_LEFT_Y);
    int xAxis = master.get_analog(ANALOG_RIGHT_X);
    
    // move drivetrain
    lmotors.move(yAxis + xAxis);
    rmotors.move(yAxis - xAxis);
}

PID drivetrainPID(2,0,0);
bool drivingStraight = false;
double drivetrainForwardDirection;
// arcade drivetrain control with pid to make it drive straight
void arcade_drive_pid() {
    int yAxis = master.get_analog(ANALOG_LEFT_Y);
    int xAxis = master.get_analog(ANALOG_RIGHT_X);
    
    // if there is movement on the xAxis, move like normal arcade drive
    if(abs(xAxis) > 1) {
        // move drivetrain normally
        lmotors.move(yAxis + xAxis);
        rmotors.move(yAxis - xAxis);
        drivingStraight = false;
    } else {
        // if we were previously turning
        if(drivingStraight == false) {
            // set the new direction we're facing as the forward direction.
            drivetrainForwardDirection = inertial.get_heading();
            drivetrainPID.reset();
            drivingStraight = true;
        }

        double error = closestAngle(drivetrainForwardDirection, inertial.get_heading());
        double turnCorrection = drivetrainPID.update(error);
        lmotors.move(yAxis + turnCorrection);
        rmotors.move(yAxis - turnCorrection);
    }
}

// driver control code
void opcontrol_loop() {
    
    // driving controls
    arcade_drive_regular();
    //arcade_drive_pid();
    pros::delay(20);
}