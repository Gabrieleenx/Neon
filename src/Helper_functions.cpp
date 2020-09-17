#include <cstdint>
#include "Variables.h"
#include <cmath>
#include <iostream> 
#include <vector>

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
};

void update_orientation_diff(Orientation_diff* orientation_diff, Odometry* odometry_current, Odometry* odometry_previous){
    orientation_diff->pos_x = odometry_current->pos_x - odometry_previous->pos_x;
    orientation_diff->pos_y = odometry_current->pos_y - odometry_previous->pos_y;
    orientation_diff->rot = odometry_current->rot - odometry_previous->rot;
};

void initilize_horizondal_distance_vector(Camera_intrinsics* c_i){
    for(int i = 0; i < (c_i->res_h); i++){
        c_i->horizontal_distance[i] = ((i+1)-(c_i->res_h/2))/(c_i->focal_point);
    }
}


int calc_mean(const std::vector<uint8_t> &image, Camera_intrinsics* camera_intrinsics, int h){

    int int8_to_int;
    int int_sum = 0;
    int int8_idx_l;
    int devide_sum = 0;
    int int8_index = 2*(camera_intrinsics->res_h)*(camera_intrinsics->horizon_px - camera_intrinsics->vertical_mean_factor)
                            + 2*h*camera_intrinsics->horizontal_down_sampling_factor; // idex in one dimetional vector

    for(int i = 0; i < (2*camera_intrinsics->vertical_mean_factor+1);  i++){
        int8_idx_l = int8_index + i*2*camera_intrinsics->res_h; // 640*2       

        for(int j = 0; j < camera_intrinsics->horizontal_down_sampling_factor; j++) {
            int8_to_int = (image[int8_idx_l+1] << 8 )| image[int8_idx_l]; // int16 to int, 
            int8_idx_l += 2; // two int8 to build int16 
            int_sum += int8_to_int; // sum all 16 values
            devide_sum += (int8_to_int != 0) ? 1 : 0; // how many values that are not 0
        }
    }

    return (devide_sum != 0) ? int_sum/devide_sum : 0; // returns mean
}


void update_depth_data(Depth_data* depth_data, Camera_intrinsics* camera_intrinsics, const std::vector<uint8_t> &image){

    for(int i = 0; i < depth_data->res; i++){
        depth_data->depth_y[i] = calc_mean(image, camera_intrinsics, i);
        depth_data->depth_x[i] = depth_data->depth_y[i] * camera_intrinsics->horizontal_distance[i];
    }
    
    depth_data->cloudpoints_idx = 0; 
    for(int i = 0; i < depth_data->res; i++){
        if (depth_data->depth_y[i] != 0){
            depth_data->cloudpoints[depth_data->cloudpoints_idx][0] = depth_data->depth_x[i] / 1000.0;
            depth_data->cloudpoints[depth_data->cloudpoints_idx][1] = depth_data->depth_y[i] / 1000.0;
            depth_data->cloudpoints_idx += 1;
        }
    }

}

void rotate_points(double new_point[][2], double point[][2], int num_points, double z){
    double Rot_M[2][2];
    Rot_M[0][0] = cos(z);
    Rot_M[0][1] = -sin(z);
    Rot_M[1][0] = sin(z);
    Rot_M[1][1] = cos(z);

    for(int i = 0; i < num_points; i++){
        new_point[i][0] = Rot_M[0][0] * point[i][0] + Rot_M[0][1] * point[i][1];
        new_point[i][1] = Rot_M[1][0] * point[i][0] + Rot_M[1][1] * point[i][1];
    }
}

void Translate_points(double new_point[][2], double point[][2], int num_points, double x, double y){
    for(int i = 0; i < num_points; i++){
        new_point[i][0] = point[i][0] + x;
        new_point[i][1] = point[i][1] + y;
    }
}

double norm(double x1, double x2, double y1, double y2){
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

int index_conversion(int x, int y, Local_map* local_map){
    return x*(local_map->grid_size) + y;
}

void update_local_map(Depth_data* depth_data, Orientation_diff* orientation_diff, Local_map* local_map){
    int indx[2];
    int index_after_conv;
    local_map->pos_x += orientation_diff->pos_x;
    local_map->pos_y += orientation_diff->pos_y;
    local_map->rot += orientation_diff->rot;

    // rotate points
    rotate_points(depth_data->cloudpoints_rot_t, depth_data->cloudpoints, depth_data->cloudpoints_idx, local_map->rot);

    // Translate points
    Translate_points(depth_data->cloudpoints_rot_t, depth_data->cloudpoints_rot_t, depth_data->cloudpoints_idx
                        , (local_map->pos_x + local_map->size_m/2), (local_map->pos_y + local_map->size_m/2));

    for(int i = 0; i < depth_data->cloudpoints_idx; i++){
        double norm_ = norm(depth_data->cloudpoints[i][0], 0.0, depth_data->cloudpoints[i][1], 0.0);
        
        if(norm_ > 3){
            continue;
        }

        indx[0] = round((depth_data->cloudpoints_rot_t[i][0]) * local_map->map_conv);
        indx[1] = round((depth_data->cloudpoints_rot_t[i][1]) * local_map->map_conv);
        index_after_conv = index_conversion(indx[0], indx[1], local_map);
        if (index_after_conv < 0 || index_after_conv >= local_map->map_elements){
            //cout << "index out of bound " << index_after_cov << "\n";
            continue;
        }
        else
        {
            // index to map
            local_map->map[index_after_conv] += 10;

            // clip map. 
            if(local_map->map[index_after_conv] > 50){
                local_map->map[index_after_conv] = 50;
            }
        }
        
    }

    //ray cast

}























