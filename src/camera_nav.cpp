#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv){
//init section
  ros::init(argc,argv,"camera_nav");
  ros::NodeHandle nh;

  ros::spin();
}