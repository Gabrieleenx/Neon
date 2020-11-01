#include "ros/ros.h"

#include "std_msgs/String.h"
/*
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>*/
#include <sensor_msgs/Image.h>
#include <neon/Encoder_count.h>

#include <signal.h>
#include <thread>
#include <mutex>
#include <math.h>
#include <random>
#include <iostream> 
#include <chrono>  // for timer
#include <vector>

#include "Variables.h"
#include "Helper_functions.h" 

// mutex for when copying data from local map to particles 
std::mutex mtx_odometry;
std::mutex mtx_local_map;

class Slam{
  private:
    
    // class global variables
    Odometry odometry;
    Odometry odometry_previous;
    Orientation_diff orientation_diff;
    Camera_intrinsics camera_intrinsics;
    Robot_intrinsics robot_intrinsics;
    Depth_data depth_data{camera_intrinsics.res_h};
    Local_map local_map;
    Local_map local_map_copy;
    const static int num_particles = 50; 
    Particle* particles = new Particle[num_particles];
    ros::Subscriber depth_sub;
    ros::Subscriber encoder_sub;

    void callback(const sensor_msgs::Image::ConstPtr& image_data){

      mtx_odometry.lock();
      update_orientation_diff(&orientation_diff, &odometry, &odometry_previous);
      odometry_previous = odometry;
      mtx_odometry.unlock();
      
      update_depth_data(&depth_data, &camera_intrinsics, image_data->data);

      mtx_local_map.lock();
      update_local_map(&depth_data, &orientation_diff, &local_map);
      mtx_local_map.unlock();

      local_map_updated = 1;
      
    }

    void callback_odometry(const neon::Encoder_count::ConstPtr& encoder_data){

      mtx_odometry.lock();
      update_odometry(&odometry, &robot_intrinsics, encoder_data->left, encoder_data->right);
      mtx_odometry.unlock();

    }

  public:
    // class global variables
    
    int local_map_updated = 0;

    Slam(ros::NodeHandle *nh){

      initilize_horizondal_distance_vector(&camera_intrinsics);

      depth_sub = nh->subscribe("/camera/depth/image_rect_raw", 2, &Slam::callback, this);
      encoder_sub = nh->subscribe("/encoder_count", 2, &Slam::callback_odometry, this);

      normalize_weights(particles, num_particles);

    }

    void particle_filter_update(){

      mtx_local_map.lock();
      local_map_copy = local_map;
      reset_local_map(&local_map);
      mtx_local_map.unlock();

      std::random_device rd; // create random random seed
      std::mt19937 gen(rd()); // put the seed inte the random generator 
      std::normal_distribution<float> pos_d(0, 0.03); // create a distrobution
      std::normal_distribution<float> rot_d(0, 0.02); // create a distrobution
      for (int i = 0; i < num_particles; i++){
        // update map
        updatate_particle_map(&particles[i], &local_map_copy);
        // update position
        particles[i].pos_x +=  cos(particles[i].rot_z)*(local_map_copy.pos_x - local_map_copy.start_pos_x) - 
          sin(particles[i].rot_z)*(local_map_copy.pos_y - local_map_copy.start_pos_y) + pos_d(gen); 
      
        particles[i].pos_y +=  cos(particles[i].rot_z)*(local_map_copy.pos_y - local_map_copy.start_pos_y) + 
          sin(particles[i].rot_z)*(local_map_copy.pos_x - local_map_copy.start_pos_x) + pos_d(gen); 

        particles[i].rot_z += local_map_copy.rot + rot_d(gen);
      }

      // update weights
      int smallest_score = 100000000;
      for (int i = 0; i < num_particles; i++){
        if (smallest_score > particles[i].scan_score){
          smallest_score = particles[i].scan_score;
        }
      }
      for (int i = 0; i < num_particles; i++){
        particles[i].weight = particles[i].weight * (particles[i].scan_score - 0.99*smallest_score)*
          (particles[i].scan_score - 0.99*smallest_score) / particles[i].num_scan_checs;
      }
      // normalize weights
      normalize_weights(particles, num_particles);
      // resample

      // moved? add to map

      // best position
    }

    void delete_memory(){
      delete[] particles;
    }
};


  

int main(int argc, char** argv){
  ros::init(argc, argv, "Slam");

  ros::NodeHandle nh;
  
  // for multithreading ros
  ros::AsyncSpinner spinner(0);
  spinner.start();

  Slam slam(&nh);

  ros::Rate loop_rate(0.5);

    uint32_t seq = 1;

    while (ros::ok()){
        //publish_cloud_points(slam, publish_point_cloud, seq);
        //ros::spinOnce();
        if(slam.local_map_updated == 1){
            slam.particle_filter_update();
        }
        loop_rate.sleep();
        ++seq;
    }

  ros::waitForShutdown();

  return 0;
}