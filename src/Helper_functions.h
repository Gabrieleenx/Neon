#pragma once

// forward function declarations 
void update_odometry(Odometry* odometry, Robot_intrinsics* robot_intrinsics, int count_left, int count_right);

void update_orientation_diff(Orientation_diff* orientation_diff, Odometry* odometry_current, Odometry* odometry_previous);

void initilize_horizondal_distance_vector(Camera_intrinsics* c_i);

void update_depth_data(Depth_data* depth_data, Camera_intrinsics* camera_intrinsics, const std::vector<uint8_t> &image);

void update_local_map(Depth_data* depth_data, Orientation_diff* orientation_diff, Local_map* local_map);

void normalize_weights(Particle* particles, int num_particles);

void reset_local_map(Local_map* local_map);