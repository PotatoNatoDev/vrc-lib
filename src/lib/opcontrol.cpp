#include "opcontrol.hpp"
#include "../globals.hpp"
#include "pid.hpp"
#include "main.h"

// code to initalization opcontrol properly
void opcontrol_init() {

}

// normal drivetrain control
void arcade_drive_regular() {
    // get controller joysticks
    int yAxis = master.get_analog(ANALOG_LEFT_Y);
    int xAxis = master.get_analog(ANALOG_RIGHT_X);
    
    // move drivetrain
    lmotors.move(yAxis + xAxis);
    rmotors.move(yAxis - xAxis);
}



PID drivetrainPID(1,1,1);
bool drivingStraight = false;
// drivetrain control with pid to make it drive straight
void arcade_drive_pid() {
    int yAxis = master.get_analog(ANALOG_LEFT_Y);
    int xAxis = master.get_analog(ANALOG_RIGHT_X);
    
    // if there is movement on the xAxis, move like normal arcade drive
    if(abs(xAxis) > 1) {
        // move drivetrain normally
        lmotors.move(yAxis + xAxis);
        rmotors.move(yAxis - xAxis);
    } else {
        // there is no movement on the xAxis, drive forward/backward with PID to correct for slight turning

    }
}

// driver control code
void opcontrol_loop() {
    
    // driving controls
    arcade_drive_regular();
    //arcade_drive_pid();
}