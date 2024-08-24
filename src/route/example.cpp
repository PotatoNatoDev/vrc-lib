#include "routes.hpp"
#include "../lib/autoncontrol.hpp"

void auton_example() {
    turn(90);
    go_straight(100, 100);
    turn(45);
    go_straight(80, 100);
    turn(-90);
    go_straight(100, 100);
    turn(225);
    go_straight(80, 100);
    turn(0);
}