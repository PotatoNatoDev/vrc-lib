#include "autoncontrol.hpp"
#include "../globals.hpp"
#include "main.h"
#include "pid.hpp"
#include "custom_math.hpp"
#include "pros/rtos.h"

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

// finds the average encoder value between all the drivetrain motors
double avg_encoder() {
    double total;
    int numLMotors = lmotors.get_position_all().size();
    int numRMotors = rmotors.get_position_all().size();
    for(int i = 0; i < numLMotors; i++) {
        total += lmotors.get_position(i);
    }
    for(int i = 0; i < numRMotors; i++) {
        total += rmotors.get_position(i);
    }
    return total / (numLMotors + numRMotors);
}

PID turnPID(7,0,20); // tune these values for turning
/* Turns the robot to a certain heading relative to the field (field oriented).
    - `double targetHeading`: the direction you want the robot to face
    - `int forceDirection`: force the robot to turn a certain direction (left = -1, right = 1)
    - `double safeZone`: how far the robot can face from the target heading
    - `float safeTime`: How long the robot has to be in the safezone before it can move on
    - `float timeOut`: the maximum amount of time the robot will attempt to turn before moving on*/
void turn(double targetHeading, int forceDirection, double safeZone, float safeTime, float timeOut) {
    // note you might have to change inertial.get_heading() to rotationToHeading(get_rotation()) depending on how you oriented the imu
    double error = closestAngle(targetHeading, inertial.get_heading());
    
    float safeTimer = pros::c::millis() + safeTime; // this variable holds the time we need to stay in the safe zone
    float exitTime = pros::c::millis() + timeOut; // this variable holds the time we exit if we take too long

    turnPID.reset(); // reset the PID

    bool ignoreForceTurn = false;
    // loops while we haven't been in the safezone long enough & we haven't timed out
    while(pros::c::millis() < safeTimer && pros::c::millis() < exitTime) {
        // find the error between our current heading and the target heading
        error = closestAngle(targetHeading, inertial.get_heading());

        if(ignoreForceTurn == false) {
            if(forceDirection > 0 && error < 0) error += 360; // force a right turn if specified
            else if(forceDirection < 0 && error > 0) error -= 360; // force a left turn if specified
        }

        master.print(0,0,"error: %f", error);

        move_steering(0, turnPID.update(error)); // update PID and turn

        // if we aren't in the safezone yet, increase the timer
        // this means if we are in the safezone, the timer won't increase.
        // if it doesn't increase long enough, pros::c::millis() will be more than the safeTimer.
        // this will break the while loop
        if(fabs(error) > safeZone) safeTimer = pros::c::millis() + safeTime;
        else ignoreForceTurn = true; // to prevent overshooting when forcing a direction
        pros::delay(20);
    }
    // stop motors
    move(0,0);
    turnPID.reset(); // reset PID (again to be safe)
    pros::delay(100); // delay so any movement from the motors settle before moving on.
}

PID goStraightPID(2,0,0); //NOTE TO SELF: we might be able to reuse turn PID
/*  Move the robot forward in a straight line. Uses motion profiling for forward momentum,
    and uses a PID controller to ensure it goes straight.
    - `double distance`:    The distance that the robot moves in inches.
    - `int power`:         The maximum power that the robot moves at. this power
    is maintained while the robot is cruising, and decreases when the robot
    decelerates.
    - `int minPower`:      The minimum power that the robot will decelerate to.
    - `int decelZone`:     The zone in which the robot starts decelerating.
    - `double decelRate`:   The rate at which the robot decelerates. 
    - `float timeOut`: the maximum amount of time the robot will attempt to move before moving on*/
void go_straight(double distance, int power, int momentum, int minPower, int decelZone, double decelRate, float timeOut) {
    lmotors.tare_position_all();
    rmotors.tare_position_all();

    double initialHeading = inertial.get_heading();
    
    double distanceDegrees = distance; // NOTE TO SELF: MAKE IT CONVERT FROM INCHES TO DEGREES
    double decelDegrees = decelZone; // NOTE TO SELF: MAKE IT CONVERT FROM INCHES TO DEGREES

    float exitTime = pros::c::millis() + timeOut; // this variable holds the time we exit if we take too long
    
    double currentPower = power * 10;
    
    while(abs(avg_encoder()) < distanceDegrees - momentum && pros::c::millis() < exitTime) {
        double remainingDistance = distanceDegrees - abs(avg_encoder());

        if(remainingDistance < decelDegrees) {
            currentPower = currentPower * (1 - decelRate);
            currentPower = fabs(currentPower) > minPower ? currentPower : minPower * sgn(power);
        } else {
            currentPower = currentPower * 1.2;
            if (fabs(currentPower) > power) currentPower = power;
        }

        double error = closestAngle(initialHeading, inertial.get_heading());
        master.print(0,0, "drive error: %f", error);

        move_steering(currentPower, goStraightPID.update(error));
        pros::delay(20);
    }

    double remainingDistance = distanceDegrees - abs(avg_encoder());
    master.print(0,0, "final position: %f", remainingDistance);
    
    move(0,0);
    pros::delay(100);
}