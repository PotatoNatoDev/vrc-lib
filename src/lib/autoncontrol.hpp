#pragma once

void move(double leftPower, double rightPower);
void move_steering(double forwardSpeed, double turnSpeed);
void turn(double targetHeading, double safeZone = 1, float safeTime = 1000, float timeOut = 10000);