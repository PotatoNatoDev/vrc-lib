#include "autoncontrol.hpp"
#include "../globals.hpp"
#include "main.h"
#include "pid.hpp"
#include "custom_math.hpp"

/* Moves both drivetrain sides with one function
   - int leftPower: the power provided to the left drivetrain motors
   - int rightPower: the power provided to the right drivetrain motors  */
void move(double leftPower, double rightPower) {
    lmotors.move(leftPower);
    rmotors.move(rightPower);
}

/* A function to determine to get the robot to drive forward or turn
   - double forwardSpeed: determines how fast the robot moves forward
   - double turnSpeed: determines in which direction and speed the robot turns*/
void move_steering(double forwardSpeed, double turnSpeed) {
    move(forwardSpeed + turnSpeed, forwardSpeed - turnSpeed);
}



PID turnPID(1,1,1); // tune these values for turning
/* Turns the robot to a certain heading relative to the field (field oriented).
    - double targetHeading: the direction you want the robot to face
    - double safeZone: how far the robot can face from the target heading
    - float safeTime: How long the robot has to be in the safezone before it can move on
    - float timeOut: the maximum amount of time the robot should take the turn before we should just move on*/
void turn(double targetHeading, double safeZone, float safeTime, float timeOut) {
    
    // note you might have to change inertial.get_heading() to rotationToHeading(get_rotation()) depending on how you oriented the imu
    double error = closestAngle(targetHeading, inertial.get_heading());
    
    float safeTimer = pros::c::millis() + safeTime; // this variable holds the time we need to stay in the safe zone
    float exitTime = pros::c::millis() + timeOut; // this variable holds the time we exit if we take too long

    turnPID.reset(); // reset the PID

    // loops while we haven't been in the safezone long enough & we haven't timed out for taking too long
    while(pros::c::millis() < safeTimer && pros::c::millis() < exitTime) {
        // find the error between our current heading and the target heading
        error = closestAngle(targetHeading, inertial.get_heading());
        
        // update PID and turn
        move_steering(0, turnPID.update(error));

        // if we aren't in the safezone yet, increase the timer
        // this means if we are in the safezone, the timer won't increase.
        // if it doesn't increase long enough, pros::c::millis() will be more than the safeTimer.
        // this will break the while loop

        if(fabs(inertial.get_heading() - targetHeading) > safeZone) {
            safeTimer = pros::c::millis() + safeTime;
        }
    }
    // stop motors
    move(0,0);
    turnPID.reset(); // reset PID (again to be safe)
    pros::delay(100); // delay so any movement from the motors settle before moving on.
}