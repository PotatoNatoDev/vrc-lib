#include "autoncontrol.hpp"
#include "../globals.hpp"
#include "main.h"
#include "pid.hpp"
#include "custom_math.hpp"
#include <vector>

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
    // create a vector with all of the drivetrain encoders in it.
    std::vector<double> positions = lmotors.get_position_all();
    std::vector<double> rpositions = rmotors.get_position_all();
    positions.insert(positions.end(), rpositions.begin(), rpositions.end());

    // add all of the encoder values together
    double total;
    for(int i = 0; i < positions.size(); i++) total += positions[i];

    // get the average by dividing the total by the total number of drivetrain motors
    return total / (double)positions.size();
}

PID goStraightPID(1,1,1); //NOTE TO SELF: we might be able to reuse turn PID
/*  Move the robot forward in a straight line. Uses motion profiling for forward momentum,
    and uses a PID controller to ensure it goes straight.
    - `float distance`:    The distance that the robot moves in inches.
    - `int power`:         The maximum power that the robot moves at. this power
    is maintained while the robot is cruising, and decreases when the robot
    decelerates.
    - `int minPower`:      The minimum power that the robot will decelerate to.
    - `int decelZone`:     The zone in which the robot starts decelerating.
    - `float decelRate`:   The rate at which the robot decelerates. */
void go_straight(float distance, int power, int momentum, int minPower, int decelZone, float decelRate, float timeOut) {
    lmotors.tare_position();
    rmotors.tare_position();

    double initialRotation = inertial.get_heading();

    
}


PID turnPID(1,1,1); // tune these values for turning
/* Turns the robot to a certain heading relative to the field (field oriented).
    - `double targetHeading`: the direction you want the robot to face
    - `double safeZone`: how far the robot can face from the target heading
    - `float safeTime`: How long the robot has to be in the safezone before it can move on
    - `float timeOut`: the maximum amount of time the robot should take the turn before we should just move on*/
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