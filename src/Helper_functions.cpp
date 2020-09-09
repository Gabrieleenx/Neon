#include "Variables.h"
#include <cmath>

void update_odometry(Odometry* odometry, Robot_intrinsics* robot_intrinsics, int count_left, int count_right){
    
    int diff_left = count_left - odometry->previous_encoder_L;
    int diff_right = count_right - odometry->previous_encoder_R;
    double rotation_diff = (diff_right - diff_left)*(robot_intrinsics->step_size)/(robot_intrinsics->wheel_base);
    double dist =  (robot_intrinsics->step_size)*(diff_right + diff_left)/2.0;

    odometry->pos_y += dist * cos((odometry->rot) + rotation_diff/2);
    odometry->pos_x += -dist * sin((odometry->rot) + rotation_diff/2);
    odometry->rot += rotation_diff;

    odometry->previous_encoder_L = count_left;
    odometry->previous_encoder_R = count_right;
}












