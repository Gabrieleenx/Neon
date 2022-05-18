#include <cstdint>
#include "Variables.h"
#include <cmath>
#include <iostream> 
#include <vector>
#include <random>

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

void rotation_matrix(double matrix[2][2], double z){
    matrix[0][0] = cos(z);
    matrix[0][1] = -sin(z);
    matrix[1][0] = sin(z);
    matrix[1][1] = cos(z);
}


void rotate_points(double new_point[][2], double point[][2], int num_points, double z){
    double Rot_M[2][2];
    rotation_matrix(Rot_M, z);

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
    //return x*(local_map->grid_size) + y;
    return y*(local_map->grid_size) + x;
}

/*
void reverse_index_conversion(int index_out[3], Particle* particle, int index_in){
    int x;
    int y;
    x = index_in/(particle->map_num_grid_x);
    y = index_in-x*(particle->map_num_grid_x);
    index_out[0] = y;
    index_out[1] = x;
}
*/

void update_local_map(Depth_data* depth_data, Orientation_diff* orientation_diff, Local_map* local_map){
    int indx[2];
    int index_after_conv;
    float max_visibility = 3;
    local_map->pos_x += orientation_diff->pos_x;
    local_map->pos_y += orientation_diff->pos_y;
    local_map->rot += orientation_diff->rot;

    // rotate points
    rotate_points(depth_data->cloudpoints_rot_t, depth_data->cloudpoints, depth_data->cloudpoints_idx, local_map->rot);

    // Translate points
    Translate_points(depth_data->cloudpoints_rot_t, depth_data->cloudpoints_rot_t, depth_data->cloudpoints_idx
                        , (local_map->pos_x), (local_map->pos_y));

    for(int i = 0; i < depth_data->cloudpoints_idx; i++){
        double norm_ = norm(depth_data->cloudpoints[i][0], 0.0, depth_data->cloudpoints[i][1], 0.0);
        
        if(norm_ > max_visibility){
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


        //ray cast
        int ray_iteration = round((norm_*(local_map->map_conv)));
        double ray_dx = (depth_data->cloudpoints_rot_t[i][0] - local_map->pos_x)/(norm_*(local_map->map_conv));
        double ray_dy = (depth_data->cloudpoints_rot_t[i][1] - local_map->pos_y)/(norm_*(local_map->map_conv));
        for (int j = 1; j < ray_iteration; j++){

            indx[0] = round((local_map->pos_x + ray_dx*j) * (local_map->map_conv));
            indx[1] = round((local_map->pos_y + ray_dy*j) * (local_map->map_conv));
            index_after_conv = index_conversion(indx[0], indx[1], local_map);
            if (index_after_conv < 0 || index_after_conv >= local_map->map_elements){
                continue;
            }
            else{
                // index to map
                local_map->map[index_after_conv] += -3;

                // clip map. 
                if(local_map->map[index_after_conv] < -50){
                    local_map->map[index_after_conv] = -50;
                }
            }
        } 
    }

}


void normalize_weights(Particle* particles, int num_particles){
    float sum_weight = 0;
    for (int i = 0; i < num_particles; i++){
        sum_weight += particles[i].weight;
    }
    for (int i = 0; i < num_particles; i++){
        particles[i].weight = particles[i].weight / sum_weight;
    }
}

void reset_local_map(Local_map* local_map){
    local_map->pos_x = local_map->start_pos_x;
    local_map->pos_y = local_map->start_pos_y;
    local_map->rot = 0;
    for(int i = 0; i < local_map->map_elements; i++){
        local_map->map[i] = 0;
    }   
}

void updatate_particle_map(Particle* particle, Local_map* local_map){
    int local_map_index[2];
    int particle_map_index;
    int local_map_start_index_x = (local_map->start_pos_x)*(local_map->map_conv);
    int local_map_start_index_y = (local_map->start_pos_y)*(local_map->map_conv);
    int Tx;
    int Ty;
    double rotated_index[2];
    double Rot_M[2][2];
    particle->scan_score = 0;
    particle->num_scan_checs = 0;
    rotation_matrix(Rot_M, particle->rot_z);
    int new_value;
    for (int i = 0; i < local_map->map_elements; i++){
        if (local_map->map[i] == 0){
            continue;
        }
        // get x and y index
        local_map_index[1] = i/(local_map->grid_size);
        local_map_index[0] = i - local_map_index[1]*(local_map->grid_size);
        // subtact starting position 
        local_map_index[0] += -local_map_start_index_x;
        local_map_index[1] += -local_map_start_index_y;
        // rotate index
        rotated_index[0] = Rot_M[0][0] * local_map_index[0] + Rot_M[0][1] * local_map_index[1];
        rotated_index[1] = Rot_M[1][0] * local_map_index[0] + Rot_M[1][1] * local_map_index[1];

        // particle map index 
        Tx = rotated_index[0] + particle->pos_x / particle->resolution;
        Ty = rotated_index[1] + particle->pos_y / particle->resolution;
        //particle_map_index = (local_map_index[0] + Tx)*(particle->map_num_grid_x) + local_map_index[1] + Ty;
        particle_map_index = Ty*(particle->map_num_grid_y) + Tx;
        //std::cout << "particle_map_index " << particle_map_index << std::endl;
        // check if inside map
        if(particle_map_index < 0 || particle_map_index > particle->map_num_elements){
            continue;
        }

        // for obstructed space
        if (local_map->map[i] > 20){
            particle->scan_score += particle->map[particle_map_index] - particle->initial_value;
            particle->num_scan_checs += 1;
        }
        // for unobstucted space, not sure if this one helps or makes it worse
        
        else if (local_map->map[i] < -35)
        {
            particle->scan_score += -0.05*(particle->map[particle_map_index] - particle->initial_value);
            particle->num_scan_checs += 1;
        }
        
        // add to map 
        if (particle->add_points == 0){
            continue;
        }
        new_value = particle->map[particle_map_index] + 0.5*local_map->map[i];
        if (new_value < 0){
            particle->map[particle_map_index] = 0;
        }
        else if (new_value > 100){
            particle->map[particle_map_index] = 100;
        }
        else{
            particle->map[particle_map_index] = new_value;
        }
    }

}


void resample(Particle* particles, int num_particles, int* best_particle_idx){

    double resample_list[num_particles+1];
    int resample_index[num_particles];
    double sum_ = 0;
    resample_list[0] = 0;
    for(int i = 0; i < num_particles; i++){
        sum_ += particles[i].weight;
        resample_list[i+1] = sum_;
        //std::cout << "sum = " << sum_ << std::endl;
    }

    std::random_device rd; // create random random seed
    std::mt19937 gen(rd()); // put the seed in the random generator 
    std::uniform_real_distribution<double> distr(0.0, 1.0); // create a distrobution

    for(int i = 0; i < num_particles; i++){
        double rand_val = distr(gen);
        for (int j = 1; j < num_particles+1; j++){
            if(rand_val > resample_list[j-1] && rand_val < resample_list[j]){
                resample_index[i] = j-1;
                break;
            }
        }
    }
    // organize to minimize copying data.
    int resample_index_org[num_particles] = { 1 };
    int resample_index_taken[num_particles] = { 0 };
    for (int i = 0; i < num_particles; i++){
        resample_index_taken[resample_index[i]] = 1;
        resample_index_org[resample_index[i]] = resample_index[i];
    }
    for (int i = 0; i < num_particles; i++){
        if(resample_index[i] == resample_index_org[resample_index[i]]){
            for(int j = 0; j < num_particles; j++){
                if (resample_index_taken[j] == 0){
                    resample_index_org[j] = resample_index[i];
                    resample_index_taken[j] = 1;
                    break;
                }
            }
        }
        
    }

    // copy data
    for(int i = 0; i < num_particles; i++){
        if (resample_index_org[i] == i){
            continue;
        }
        else{
            particles[i] = particles[resample_index_org[i]];
        }
    }

    // save index with higest weight 
    double last_high_weight = 0;
    for(int i = 0; i < num_particles; i++){
        if (particles[i].weight > last_high_weight){
            *best_particle_idx = i;
            last_high_weight = particles[i].weight;
        }
    }

    std::cout << " high = " << last_high_weight << " best index " <<  *best_particle_idx << std::endl;
}










