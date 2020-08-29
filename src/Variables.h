#pragma once

// dinfine structures and variables
const double pi = 3.141592653589793238462643383279502884197169;

struct Odometry
{
    double pos_x = 0;
    double pos_y = 0;
    double rot = 0;
    int previous_encoder_L = 0;
    int previous_encoder_R = 0;
};

struct Robot_intrinsics
{
    double wheel_diameter = 0.102;
    double wheel_base = 0.181;
    int counts_per_turns = 312;
    double step_size = (wheel_diameter * pi)/counts_per_turns;
};
