#pragma once

void move(double leftPower, double rightPower);
void move_steering(double forwardSpeed, double turnSpeed);
void turn(double targetHeading, int forceDirection = 0, double safeZone = 1, float safeTime = 80, float timeOut = 999999);
void go_straight(double distance, int power, int momentum = 0, int minPower = 30, int decelZone = 3, double decelRate = 0.1, float timeOut = 999999);