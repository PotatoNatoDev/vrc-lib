#include "autoncontrol.hpp"
#include "../globals.hpp"
#include "main.h"


/* Moves both drivetrain sides with one function
   - int leftPower: the power provided to the left drivetrain motors
   - int rightPower: the power provided to the right drivetrain motors  */
void move(double leftPower, double rightPower) {
    lmotors.move(leftPower);
    rmotors.move(rightPower);
}

/* A function to determine the steering of the robot with one variable. 
   Implemented from https://github.com/Nomnomburger/vrclib/
   - int steering: Steering value, where 0 steering is going straight, 50
   steering is a one wheel turn, 100 steering is an 0on-spot turn, and anything
   in between is a two-wheeled turn.
   - int speed:    Power value. Determines how fast the robot moves.  */
void move_steering(float steering, float speed) {
    if (steering < 0) {  // left
        move(((50 + steering) / 50) * speed, speed);
    } else {  // right
        move(speed, ((50 - steering) / 50) * speed);
    }
}