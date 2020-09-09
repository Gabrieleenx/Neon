#include "ros/ros.h"

#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <neon/Encoder_count.h>

#include <signal.h>
#include <thread>
#include <mutex>
#include <math.h>
#include <random>
#include <iostream> 
#include <chrono>  // for timer

#include "Variables.h"
#include "Helper_functions.h" 

// Timestamps not synced between ros on nano and camera..

class Slam{
  private:
    // class global variables
    Odometry odometry;
    Robot_intrinsics robot_intrinsics;
    //ros::Subscriber depth_sub;
    ros::Subscriber encoder_sub;
    //, const sensor_msgs::Image::ConstPtr& image_data, 
    void callback(const neon::Encoder_count::ConstPtr& encoder_data){
      // Solve all of perception here...
      std::cout << "calllback Left encoder" << encoder_data->left << " Right encoder " << encoder_data->right << std::endl;
      update_odometry(&odometry, &robot_intrinsics, encoder_data->left, encoder_data->right);
      std::cout << "Left pos " << odometry.pos_x << " Right pos " << odometry.pos_y << " Rotation " << odometry.rot << std::endl; 
    }

  public:
    Slam(ros::NodeHandle *nh){

    //depth_sub = nh->subscribe("/camera/depth/image_rect_raw", 2, &Slam::callback, this);
    encoder_sub = nh->subscribe("/encoder_count", 2, &Slam::callback, this);

    /*
    message_filters::Subscriber<sensor_msgs::Image> image_sub(*nh, "/camera/depth/image_rect_raw", 1);
    message_filters::Subscriber<neon::Encoder_count> encoder_count_sub(*nh, "/encoder_count", 1);
    std::cout << "hello" << std::endl;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, neon::Encoder_count> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub, encoder_count_sub);
    sync.registerCallback(boost::bind(&Slam::callback, this, _1, _2));
    */
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