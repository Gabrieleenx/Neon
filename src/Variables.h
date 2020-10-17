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

struct Orientation_diff
{
    double pos_x = 0;
    double pos_y = 0;
    double rot = 0;
};

struct Robot_intrinsics
{
    double wheel_diameter = 0.102;
    double wheel_base = 0.181;
    int counts_per_turns = 312;
    double step_size = (wheel_diameter * pi)/counts_per_turns;
};

struct Camera_intrinsics
{
    const static int res_h = 640;
    const static int res_v = 480;
    int horizon_px = 240;

    const static int horizontal_down_sampling_factor = 1;
    const static int vertical_mean_factor = 8;

    double focal_point = 385.0/horizontal_down_sampling_factor;
    double horizontal_distance[res_h];
 
};

struct Depth_data
{
    const static int horizontal_down_sampling_factor = 1;
    const static int vertical_mean_factor = 8;

    Depth_data(int res_){
         const static int res = res_/horizontal_down_sampling_factor;
    }
    const static int res = 640/horizontal_down_sampling_factor;
    int depth_y[res];
    int depth_x[res];
    double cloudpoints[res][2];
    int cloudpoints_idx = 0;
    double cloudpoints_rot_t[res][2];
};

struct Local_map
{   
    constexpr static double resoulution = 0.05;
    double map_conv = 1/resoulution;
    const static int size_m = 10;
    const static int grid_size = size_m/resoulution;
    constexpr static double start_pos_x = 5;
    constexpr static double start_pos_y = 5;
    int8_t map[grid_size*grid_size] = {0};
    double pos_x = 5;
    double pos_y = 5;
    double rot = 0;
    int map_elements = grid_size*grid_size;
};

struct Particle
{
    constexpr static double resoulution = 0.05;
    const static int map_size_x = 15;
    const static int map_size_y = 15;
    const static int map_num_grid_x = map_size_x/resoulution; 
    const static int map_num_grid_y = map_size_y/resoulution; 
    const static int map_num_elements = map_num_grid_x * map_num_grid_y;
    double pos_x = 7.5;
    double pos_y = 7.5;
    double rot_z = 0.0;
    double weight = 1.0/50; 
    uint8_t map[map_num_elements] =  {0};
    Particle(){
        for (int i = 0; i < map_num_elements; i++){
            map[i] = 50;
        }
    }

};
