#pragma once

void move(double leftPower, double rightPower);
void move_steering(double forwardSpeed, double turnSpeed);
void turn(double targetHeading, double safeZone = 1, float safeTime = 1000, float timeOut = 999999);
void go_straight(float distance, int power, int momentum = 127, int minPower = 30, int decelZone = 1, float decelRate = 0.8, float timeOut = 999999);