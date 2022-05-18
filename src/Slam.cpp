#include "ros/ros.h"

#include "std_msgs/String.h"
/*
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>*/
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <neon/Encoder_count.h>
#include <tf/transform_broadcaster.h>

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
    const static int num_particles = 500; 
    Particle* particles = new Particle[num_particles];
    ros::Subscriber depth_sub;
    ros::Subscriber encoder_sub;
    ros::Publisher publish_map;
    tf::TransformBroadcaster broadcaster;
    tf::Quaternion quaternion;
    int best_particle_idx = 0;

    void callback(const sensor_msgs::Image::ConstPtr& image_data){
      std::cout << "image in" << std::endl;
      mtx_odometry.lock();
      update_orientation_diff(&orientation_diff, &odometry, &odometry_previous);
      odometry_previous = odometry;
      mtx_odometry.unlock();
      
      update_depth_data(&depth_data, &camera_intrinsics, image_data->data);

      mtx_local_map.lock();
      update_local_map(&depth_data, &orientation_diff, &local_map);
      mtx_local_map.unlock();

            // best position
      double rosEstZ = particles[best_particle_idx].rot_z + local_map_copy.rot;
      quaternion.setRPY(0,0, rosEstZ);

      double posEstX = cos(particles[best_particle_idx].rot_z)*(local_map_copy.pos_x - local_map_copy.start_pos_x) - 
                      sin(particles[best_particle_idx].rot_z)*(local_map_copy.pos_y - local_map_copy.start_pos_y) - 
                      particles[best_particle_idx].map_size_x/2.0;
      double posEstY = cos(particles[best_particle_idx].rot_z)*(local_map_copy.pos_y - local_map_copy.start_pos_y) + 
                      sin(particles[best_particle_idx].rot_z)*(local_map_copy.pos_x - local_map_copy.start_pos_x) - 
                      particles[best_particle_idx].map_size_y/2.0;
      std::cout << "Position x local" <<local_map.pos_x - local_map.start_pos_x<< " Position y " << local_map.pos_y - local_map.start_pos_y << " rot " << particles[best_particle_idx].rot_z  << std::endl;

      broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(quaternion, tf::Vector3(posEstX, posEstY, 0)),
          ros::Time::now(),"world", "robot"));

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
      //publish_map = nh->advertise<sensor_msgs::PointCloud>("/PointCloud", 1);
      publish_map = nh->advertise<nav_msgs::OccupancyGrid>("/OccupancyGrid", 1);
      normalize_weights(particles, num_particles);

    }

    void particle_filter_update(){

      mtx_local_map.lock();
      local_map_copy = local_map;
      reset_local_map(&local_map);
      mtx_local_map.unlock();

      std::random_device rd; // create random random seed
      std::mt19937 gen(rd()); // put the seed inte the random generator 
      std::normal_distribution<float> pos_d(0, 0.05); // create a distrobution
      std::normal_distribution<float> rot_d(0, 0.1); // create a distrobution
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
          (particles[i].scan_score - 0.99*smallest_score) / particles[i].num_scan_checs + 0.0000001;
      }
      // normalize weights
      normalize_weights(particles, num_particles);
      // resample
      resample(particles, num_particles, &best_particle_idx);

      // moved? add to map
      std::cout << "Position x " <<particles[best_particle_idx].pos_x << " Position y " <<particles[best_particle_idx].pos_y << " rot " << particles[best_particle_idx].rot_z  << " best " <<  particles[best_particle_idx].scan_score << std::endl;


    }

    void publish_cloud_points(uint32_t seq){
      // creates pointcloud message and publish it. 
      //particle_map();
      /*
      sensor_msgs::PointCloud msg;
      msg.header.frame_id = "/world";
      msg.header.stamp = ros::Time::now();
      msg.header.seq = seq;
      geometry_msgs::Point32 point32;
      std::cout << "publish map " << std::endl;

      // PUBLISH MAP
      for(int i = 0; i < particles[0].map_num_elements; i+=4){


        for(int i = 0; i < particles[best_particle_idx].map_num_elements; i++){

          if (particles[best_particle_idx].map[i] > 80){
              int index_map_pub[3];
              reverse_index_conversion(index_map_pub, &particles[best_particle_idx], i);

              point32.x = index_map_pub[0]*particles[best_particle_idx].resolution - 7.5;
              point32.y = index_map_pub[1]*particles[best_particle_idx].resolution - 7.5;
              point32.z = 0;
              msg.points.push_back(point32);
          }     
        }
      }
      publish_map.publish(msg);
      */
      // publish map
      nav_msgs::OccupancyGrid msg;
      msg.header.frame_id = "/world";
      msg.header.stamp = ros::Time::now();
      msg.header.seq = seq;
      msg.info.map_load_time = ros::Time::now();
      msg.info.resolution = particles[best_particle_idx].resolution;
      msg.info.width = particles[best_particle_idx].map_num_grid_x;
      msg.info.height = particles[best_particle_idx].map_num_grid_y;
      msg.info.origin.position.x = -7.5;
      msg.info.origin.position.y = -7.5; 
      msg.info.origin.position.z = 0.0;

      msg.info.origin.orientation.x = 0;
      msg.info.origin.orientation.y = 0;
      msg.info.origin.orientation.z = 0;
      msg.info.origin.orientation.w = 1;
      std::vector<int8_t> mapMsg;
      mapMsg.resize(90000);
      for (int i = 0; i < mapMsg.size() ; i++){
        mapMsg[i] = particles[best_particle_idx].map[i];
      }

      msg.data = mapMsg;
      //geometry_msgs::Point32 point32;

      publish_map.publish(msg);
      std::cout << "publish map " << std::endl;
      


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
        if(slam.local_map_updated == 1){
            slam.particle_filter_update();
        }
        slam.publish_cloud_points(seq);
        ros::spinOnce();
        loop_rate.sleep();
        ++seq;
    }

  ros::waitForShutdown();

  return 0;
}