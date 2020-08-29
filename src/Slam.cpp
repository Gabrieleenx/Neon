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
#include <iostream> // for cout
#include <chrono>  // for timer

#include "Variables.h"
#include "Helper_functions.h" 


class Slam{
  private:
    // class global variables
    Odometry odometry;
    Robot_intrinsics robot_intrinsics;

    void callback(const sensor_msgs::Image::ConstPtr& image_data, const neon::Encoder_count::ConstPtr& encoder_data){
      // Solve all of perception here...
      update_odometry(&odometry, &robot_intrinsics, encoder_data->left, encoder_data->right);

    }

  public:
    Slam(ros::NodeHandle *nh){
    message_filters::Subscriber<sensor_msgs::Image> image_sub(*nh, "/camera/depth/image_rect_raw", 1);
    message_filters::Subscriber<neon::Encoder_count> encoder_count_sub(*nh, "/encoder_count", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, neon::Encoder_count> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, encoder_count_sub);
    sync.registerCallback(boost::bind(&Slam::callback, this, _1, _2));
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