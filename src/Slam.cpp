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
    ros::Subscriber depth_sub;
    ros::Subscriber encoder_sub;

    void callback(const sensor_msgs::Image::ConstPtr& image_data){

      mtx_odometry.lock();
      update_orientation_diff(&orientation_diff, &odometry, &odometry_previous);
      odometry_previous = odometry;
      mtx_odometry.unlock();
      
      update_depth_data(&depth_data, &camera_intrinsics, image_data->data);

      update_local_map(&depth_data, &orientation_diff, &local_map);
      
    }

    void callback_odometry(const neon::Encoder_count::ConstPtr& encoder_data){

      mtx_odometry.lock();
      update_odometry(&odometry, &robot_intrinsics, encoder_data->left, encoder_data->right);
      mtx_odometry.unlock();

    }

  public:
     // class global variables
    Slam(ros::NodeHandle *nh){

      initilize_horizondal_distance_vector(&camera_intrinsics);

      depth_sub = nh->subscribe("/camera/depth/image_rect_raw", 2, &Slam::callback, this);
      encoder_sub = nh->subscribe("/encoder_count", 2, &Slam::callback_odometry, this);

    }

};


  

int main(int argc, char** argv){
  ros::init(argc, argv, "Slam");

  ros::NodeHandle nh;
  
  // for multithreading ros
  ros::AsyncSpinner spinner(0);
  spinner.start();

  Slam slam(&nh);

  ros::waitForShutdown();

  return 0;
}